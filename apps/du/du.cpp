/*
 *
 * Copyright 2021-2023 Software Radio Systems Limited
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include "srsran/pcap/pcap.h"
#include "srsran/support/build_info/build_info.h"
#include "srsran/support/cpu_features.h"
#include "srsran/support/signal_handler.h"
#include "srsran/support/tsan_options.h"

#include "srsran/f1u/local_connector/f1u_local_connector.h"

#include "srsran/support/io_broker/io_broker_factory.h"
#include "adapters/f1ap_adapter.h"

#include "srsran/support/backtrace.h"
#include "srsran/support/config_parsers.h"
#include "srsran/support/executors/task_worker_pool.h"

#include "du_appconfig.h"
#include "du_appconfig_cli11_schema.h"
#include "du_appconfig_translators.h"
#include "du_appconfig_validators.h"

#include "helpers/du_console_helper.h"

#include "fapi_factory.h"
#include "lib/du_high/du_high.h"
#include "lib/du_high/du_high_executor_strategies.h"
#include "lib/pcap/dlt_pcap_impl.h"
#include "lib/pcap/mac_pcap_impl.h"
#include "phy_factory.h"
#include "radio_notifier_sample.h"
#include "srsran/du/du_cell_config_helpers.h"
#include "srsran/fapi/logging_decorator_factories.h"
#include "srsran/fapi_adaptor/phy/phy_fapi_adaptor_factory.h"
#include "srsran/phy/adapters/phy_error_adapter.h"
#include "srsran/phy/adapters/phy_rg_gateway_adapter.h"
#include "srsran/phy/adapters/phy_rx_symbol_adapter.h"
#include "srsran/phy/adapters/phy_rx_symbol_request_adapter.h"
#include "srsran/phy/adapters/phy_timing_adapter.h"
#include "srsran/phy/lower/lower_phy_controller.h"
#include "srsran/phy/lower/lower_phy_factory.h"
#include "srsran/phy/upper/upper_phy_timing_notifier.h"
#include "srsran/radio/radio_factory.h"
#include "srsran/support/sysinfo.h"
#include <atomic>
#include <csignal>
#include <unordered_map>

using namespace srsran;

/// \file
/// \brief Application of a co-located gNB with combined distributed unit (DU) and centralized unit (CU).
///
/// This application runs a DU with the F1 connection between it and the CU going over a real SCTP connection.
///
/// \cond

/// From TS38.104 Section 5.3.2 Table 5.3.2-1. Default 20MHz FR1.
static std::string config_file;

static std::atomic<bool> is_running = {true};
// F1AP configuration.
static srsran::sctp_network_gateway_config f1ap_nw_config;
const std::string                          srsgnb_version = "0.1";

static void populate_cli11_generic_args(CLI::App& app)
{
  app.set_version_flag("-v,--version", "srsGNB version " + srsgnb_version);
  app.set_config("-c,", config_file, "Read config from file", false);
}

/// This function takes the populated appconfig and generates (sub)-component configurations.
static void compute_derived_args(const du_appconfig& gnb_params)
{
  /// Simply set the respective values in the appconfig.
  f1ap_nw_config.connect_address = gnb_params.f1ap_cfg.cu_addr;
  f1ap_nw_config.connect_port    = gnb_params.f1ap_cfg.port;
  f1ap_nw_config.bind_address    = gnb_params.f1ap_cfg.bind_addr;
  f1ap_nw_config.ppid            = F1AP_PPID;
  if (gnb_params.f1ap_cfg.sctp_rto_initial >= 0) {
    f1ap_nw_config.rto_initial = gnb_params.f1ap_cfg.sctp_rto_initial;
  }
  if (gnb_params.f1ap_cfg.sctp_rto_min >= 0) {
    f1ap_nw_config.rto_min = gnb_params.f1ap_cfg.sctp_rto_min;
  }
  if (gnb_params.f1ap_cfg.sctp_rto_max >= 0) {
    f1ap_nw_config.rto_max = gnb_params.f1ap_cfg.sctp_rto_max;
  }
  if (gnb_params.f1ap_cfg.sctp_init_max_attempts >= 0) {
    f1ap_nw_config.init_max_attempts = gnb_params.f1ap_cfg.sctp_init_max_attempts;
  }
  if (gnb_params.f1ap_cfg.sctp_max_init_timeo >= 0) {
    f1ap_nw_config.max_init_timeo = gnb_params.f1ap_cfg.sctp_max_init_timeo;
  }
}

namespace {

/// Dummy implementation of the mac_result_notifier.
class phy_dummy : public mac_result_notifier
{
  mac_cell_result_notifier& cell;

public:
  explicit phy_dummy(mac_cell_result_notifier& cell_) : cell(cell_) {}

  mac_cell_result_notifier& get_cell(du_cell_index_t cell_index) override { return cell; }
};

class fapi_slot_last_message_dummy : public fapi::slot_last_message_notifier
{
public:
  void on_last_message(slot_point slot) override {}
};

/// Manages the workers of the app.
struct worker_manager {
  worker_manager(const du_appconfig& appcfg)
  {
    lower_phy_thread_profile lower_phy_profile = lower_phy_thread_profile::blocking;
    if (appcfg.rf_driver_cfg.device_driver != "zmq") {
      lower_phy_profile = appcfg.expert_phy_cfg.lphy_executor_profile;
    }

    create_executors(
        appcfg.rf_driver_cfg.device_driver == "zmq", lower_phy_profile, appcfg.expert_phy_cfg.nof_ul_threads);
  }

  void stop()
  {
    for (auto& worker : workers) {
      worker.second->stop();
    }
    for (auto& pool : worker_pools) {
      pool.second->stop();
    }
  }

  /*
  du ctrl exec points to general ctrl_worker
  du ue exec points to the general ue_worker

  The handler side is responsible for executor dispatching:
  - f1ap_cu::handle_message calls cu-cp ctrl exec
  */

  std::unique_ptr<task_executor> gtpu_pdu_exec;
  std::unique_ptr<task_executor> du_ctrl_exec;
  std::unique_ptr<task_executor> du_ue_exec;
  std::unique_ptr<task_executor> du_cell_exec;
  std::unique_ptr<task_executor> lower_phy_tx_exec;
  std::unique_ptr<task_executor> lower_phy_rx_exec;
  std::unique_ptr<task_executor> lower_phy_dl_exec;
  std::unique_ptr<task_executor> lower_phy_ul_exec;
  std::unique_ptr<task_executor> lower_prach_exec;
  std::unique_ptr<task_executor> upper_dl_exec;
  std::unique_ptr<task_executor> upper_pusch_exec;
  std::unique_ptr<task_executor> upper_pucch_exec;
  std::unique_ptr<task_executor> upper_prach_exec;
  std::unique_ptr<task_executor> radio_exec;

  std::unordered_map<std::string, std::unique_ptr<task_executor>> task_execs;
  optional<pcell_ue_executor_mapper>                              ue_exec_mapper;
  optional<cell_executor_mapper>                                  cell_exec_mapper;

private:
  std::unordered_map<std::string, std::unique_ptr<task_worker>>      workers;
  std::unordered_map<std::string, std::unique_ptr<task_worker_pool>> worker_pools;

  // helper method to create workers
  template <typename... Args>
  void create_worker(const std::string& name, Args&&... args)
  {
    auto ret = workers.insert(std::make_pair(name, std::make_unique<task_worker>(name, std::forward<Args>(args)...)));
    srsran_assert(ret.second, "Unable to create worker {}.", name);
  }
  // helper method to create worker pool
  void create_worker_pool(const std::string&          name,
                          size_t                      nof_workers,
                          size_t                      queue_size,
                          os_thread_realtime_priority prio = os_thread_realtime_priority::no_realtime())
  {
    auto ret = worker_pools.insert(
        std::make_pair(name, std::make_unique<task_worker_pool>(nof_workers, queue_size, name, prio)));
    srsran_assert(ret.second, "Unable to create worker pool {}.", name);
  }

  void create_executors(bool blocking_mode_active, lower_phy_thread_profile lower_phy_profile, unsigned nof_ul_workers)
  {
    static const uint32_t task_worker_queue_size = 2048;

    // Instantiate workers
    create_worker("gnb_ctrl", task_worker_queue_size);
    create_worker("gnb_ue", 512);
    create_worker("du_cell",
                  task_worker_queue_size,
                  os_thread_realtime_priority::max() - 2,
                  os_sched_affinity_bitmask{},
                  std::chrono::microseconds{10});
    if (blocking_mode_active) {
      create_worker("phy_worker", task_worker_queue_size, os_thread_realtime_priority::max());
    } else {
      create_worker("phy_prach", task_worker_queue_size, os_thread_realtime_priority::max() - 2);
      create_worker("upper_phy_dl", task_worker_queue_size, os_thread_realtime_priority::max() - 10);
      create_worker_pool(
          "upper_phy_ul", nof_ul_workers, task_worker_queue_size, os_thread_realtime_priority::max() - 20);
    }
    create_worker("radio", task_worker_queue_size);

    // Instantiate task executors
    gtpu_pdu_exec = std::make_unique<task_worker_executor>(*workers.at("gnb_ue"), false);
    du_ctrl_exec  = std::make_unique<task_worker_executor>(*workers.at("gnb_ctrl"));
    du_ue_exec    = std::make_unique<task_worker_executor>(*workers.at("gnb_ue"));
    du_cell_exec  = std::make_unique<task_worker_executor>(*workers.at("du_cell"));
    if (blocking_mode_active) {
      task_worker& phy_worker = *workers.at("phy_worker");
      lower_phy_tx_exec       = std::make_unique<task_worker_executor>(phy_worker);
      lower_phy_rx_exec       = std::make_unique<task_worker_executor>(phy_worker);
      lower_phy_dl_exec       = std::make_unique<task_worker_executor>(phy_worker);
      lower_phy_ul_exec       = std::make_unique<task_worker_executor>(phy_worker);
      lower_prach_exec        = std::make_unique<task_worker_executor>(phy_worker);
      upper_dl_exec           = std::make_unique<task_worker_executor>(phy_worker);
      upper_pusch_exec        = std::make_unique<task_worker_executor>(phy_worker);
      upper_pucch_exec        = std::make_unique<task_worker_executor>(phy_worker);
      upper_prach_exec        = std::make_unique<task_worker_executor>(phy_worker);
    } else {
      lower_prach_exec = std::make_unique<task_worker_executor>(*workers.at("phy_prach"));
      upper_dl_exec    = std::make_unique<task_worker_executor>(*workers.at("upper_phy_dl"));
      upper_pusch_exec = std::make_unique<task_worker_pool_executor>(*worker_pools.at("upper_phy_ul"));
      upper_pucch_exec = std::make_unique<task_worker_pool_executor>(*worker_pools.at("upper_phy_ul"));
      upper_prach_exec = std::make_unique<task_worker_executor>(*workers.at("phy_prach"));
    }

    switch (lower_phy_profile) {
      case lower_phy_thread_profile::blocking: {
        fmt::print("Lower PHY in executor blocking mode.\n");
        task_worker& phy_worker = *workers.at("phy_worker");
        lower_phy_tx_exec       = std::make_unique<task_worker_executor>(phy_worker);
        lower_phy_rx_exec       = std::make_unique<task_worker_executor>(phy_worker);
        lower_phy_dl_exec       = std::make_unique<task_worker_executor>(phy_worker);
        lower_phy_ul_exec       = std::make_unique<task_worker_executor>(phy_worker);
        break;
      }
      case lower_phy_thread_profile::single: {
        fmt::print("Lower PHY in single executor mode.\n");
        create_worker("lower_phy", 128, os_thread_realtime_priority::max());
        task_worker& lower_phy_worker = *workers.at("lower_phy");
        lower_phy_tx_exec             = std::make_unique<task_worker_executor>(lower_phy_worker);
        lower_phy_rx_exec             = std::make_unique<task_worker_executor>(lower_phy_worker);
        lower_phy_dl_exec             = std::make_unique<task_worker_executor>(lower_phy_worker);
        lower_phy_ul_exec             = std::make_unique<task_worker_executor>(lower_phy_worker);
        break;
      }
      case lower_phy_thread_profile::dual: {
        fmt::print("Lower PHY in dual executor mode.\n");
        create_worker("lower_phy_dl", 128, os_thread_realtime_priority::max());
        create_worker("lower_phy_ul", 2, os_thread_realtime_priority::max() - 1);
        lower_phy_tx_exec = std::make_unique<task_worker_executor>(*workers.at("lower_phy_dl"));
        lower_phy_rx_exec = std::make_unique<task_worker_executor>(*workers.at("lower_phy_ul"));
        lower_phy_dl_exec = std::make_unique<task_worker_executor>(*workers.at("lower_phy_dl"));
        lower_phy_ul_exec = std::make_unique<task_worker_executor>(*workers.at("lower_phy_ul"));
        break;
      }
      case lower_phy_thread_profile::quad: {
        fmt::print("Lower PHY in quad executor mode.\n");
        create_worker("lower_phy_tx", 128, os_thread_realtime_priority::max());
        create_worker("lower_phy_rx", 1, os_thread_realtime_priority::max() - 2);
        create_worker("lower_phy_dl", 128, os_thread_realtime_priority::max() - 1);
        create_worker("lower_phy_ul", 128, os_thread_realtime_priority::max() - 3);
        lower_phy_tx_exec = std::make_unique<task_worker_executor>(*workers.at("lower_phy_tx"));
        lower_phy_rx_exec = std::make_unique<task_worker_executor>(*workers.at("lower_phy_rx"));
        lower_phy_dl_exec = std::make_unique<task_worker_executor>(*workers.at("lower_phy_dl"));
        lower_phy_ul_exec = std::make_unique<task_worker_executor>(*workers.at("lower_phy_ul"));
        break;
      }
    }

    radio_exec = std::make_unique<task_worker_executor>(*workers.at("radio"));

    // Executor mappers.
    ue_exec_mapper.emplace(pcell_ue_executor_mapper{du_ue_exec.get()});
    cell_exec_mapper.emplace(cell_executor_mapper{{du_cell_exec.get()}, blocking_mode_active});
  }
};

} // namespace

static lower_phy_configuration create_lower_phy_configuration(baseband_gateway*             bb_gateway,
                                                              lower_phy_rx_symbol_notifier* rx_symbol_notifier,
                                                              lower_phy_timing_notifier*    timing_notifier,
                                                              lower_phy_error_notifier*     error_notifier,
                                                              task_executor&                tx_executor,
                                                              task_executor&                rx_executor,
                                                              task_executor&                dl_executor,
                                                              task_executor&                ul_executor,
                                                              task_executor&                prach_executor,
                                                              const du_appconfig&          app_cfg)
{
  lower_phy_configuration phy_config = generate_ru_config(app_cfg);

  phy_config.bb_gateway           = bb_gateway;
  phy_config.error_notifier       = error_notifier;
  phy_config.rx_symbol_notifier   = rx_symbol_notifier;
  phy_config.timing_notifier      = timing_notifier;
  phy_config.tx_task_executor     = &tx_executor;
  phy_config.rx_task_executor     = &rx_executor;
  phy_config.dl_task_executor     = &dl_executor;
  phy_config.ul_task_executor     = &ul_executor;
  phy_config.prach_async_executor = &prach_executor;

  return phy_config;
}

static void local_signal_handler()
{
  is_running = false;
}

static fapi::prach_config generate_prach_config_tlv(const std::vector<du_cell_config>& cell_cfg)
{
  srsran_assert(cell_cfg.size() == 1, "Currently supporting one cell");

  const du_cell_config& cell = cell_cfg.front();

  fapi::prach_config config     = {};
  config.prach_res_config_index = 0;
  config.prach_sequence_length  = fapi::prach_sequence_length_type::long_sequence;
  config.prach_scs              = prach_subcarrier_spacing::kHz1_25;
  config.prach_ul_bwp_pusch_scs = cell.scs_common;
  config.restricted_set         = restricted_set_config::UNRESTRICTED;
  config.num_prach_fd_occasions = cell.ul_cfg_common.init_ul_bwp.rach_cfg_common.value().rach_cfg_generic.msg1_fdm;
  config.prach_config_index =
      cell.ul_cfg_common.init_ul_bwp.rach_cfg_common.value().rach_cfg_generic.prach_config_index;
  config.prach_format           = prach_format_type::zero;
  config.num_prach_td_occasions = 1;
  config.num_preambles          = 1;
  config.start_preamble_index   = 0;

  // Add FD occasion info.
  fapi::prach_fd_occasion_config& fd_occasion = config.fd_occasions.emplace_back();
  fd_occasion.prach_root_sequence_index = cell.ul_cfg_common.init_ul_bwp.rach_cfg_common.value().prach_root_seq_index;
  fd_occasion.prach_freq_offset =
      cell.ul_cfg_common.init_ul_bwp.rach_cfg_common.value().rach_cfg_generic.msg1_frequency_start;
  fd_occasion.prach_zero_corr_conf =
      cell.ul_cfg_common.init_ul_bwp.rach_cfg_common.value().rach_cfg_generic.zero_correlation_zone_config;

  return config;
}

static fapi::carrier_config generate_carrier_config_tlv(const du_appconfig& config)
{
  // Deduce common numerology and grid size for DL and UL.
  unsigned numerology       = to_numerology_value(config.common_cell_cfg.common_scs);
  unsigned grid_size_bw_prb = band_helper::get_n_rbs_from_bw(
      config.common_cell_cfg.channel_bw_mhz,
      config.common_cell_cfg.common_scs,
      band_helper::get_freq_range(band_helper::get_band_from_dl_arfcn(config.common_cell_cfg.dl_arfcn)));

  fapi::carrier_config fapi_config = {};

  // NOTE; for now we only need to fill the nof_prb_ul_grid and nof_prb_dl_grid for the common SCS.
  fapi_config.dl_grid_size             = {};
  fapi_config.dl_grid_size[numerology] = grid_size_bw_prb;
  fapi_config.ul_grid_size             = {};
  fapi_config.ul_grid_size[numerology] = grid_size_bw_prb;

  return fapi_config;
}

static std::unique_ptr<radio_session>
build_radio(task_executor& executor, radio_notification_handler& radio_handler, const du_appconfig& config)
{
  std::unique_ptr<radio_factory> factory = create_radio_factory(config.rf_driver_cfg.device_driver);
  if (!factory) {
    return nullptr;
  }

  // Create radio configuration. Assume 1 sector per stream.
  radio_configuration::radio radio_config = generate_radio_config(config, factory->get_configuration_validator());
  return factory->create(radio_config, executor, radio_handler);
}

int main(int argc, char** argv)
{
  // Set signal handler.
  register_signal_handler(local_signal_handler);

  // Enable backtrace.
  enable_backtrace();

  // Setup size of byte buffer pool.
  init_byte_buffer_segment_pool(524288);

  // Setup and configure config parsing.
  CLI::App app("srsGNB application");
  app.config_formatter(create_yaml_config_parser());
  app.allow_config_extras(CLI::config_extras_mode::error);
  // Fill the generic application arguments to parse.
  populate_cli11_generic_args(app);

  du_appconfig gnb_cfg;
  // Configure CLI11 with the gNB application configuration schema.
  configure_cli11_with_du_appconfig_schema(app, gnb_cfg);

  // Parse arguments.
  CLI11_PARSE(app, argc, argv);

  // Check the modified configuration.
  if (!validate_appconfig(gnb_cfg)) {
    report_error("Invalid configuration detected.\n");
  }

  // Compute derived parameters.
  compute_derived_args(gnb_cfg);

  // Set up logging.
  srslog::sink* log_sink = (gnb_cfg.log_cfg.filename == "stdout") ? srslog::create_stdout_sink()
                                                                  : srslog::create_file_sink(gnb_cfg.log_cfg.filename);
  if (log_sink == nullptr) {
    report_error("Could not create application main log sink.\n");
  }
  srslog::set_default_sink(*log_sink);
  srslog::init();

  // Set log-level of app and all non-layer specific components to app level.
  srslog::basic_logger& gnb_logger = srslog::fetch_basic_logger("DU_APP");
  gnb_logger.set_level(srslog::str_to_basic_level(gnb_cfg.log_cfg.all_level));
  for (const auto& id : {"GNB", "ALL", "SCTP-GW", "IO-EPOLL", "UDP-GW", "PCAP"}) {
    auto& logger = srslog::fetch_basic_logger(id, false);
    logger.set_level(srslog::str_to_basic_level(gnb_cfg.log_cfg.lib_level));
    logger.set_hex_dump_max_size(gnb_cfg.log_cfg.hex_max_size);
  }

  // Set component-specific logging options.
  for (const auto& id : {"DU", "DU-MNG", "UE-MNG", "DU-F1"}) {
    auto& du_logger = srslog::fetch_basic_logger(id, false);
    du_logger.set_level(srslog::str_to_basic_level(gnb_cfg.log_cfg.du_level));
    du_logger.set_hex_dump_max_size(gnb_cfg.log_cfg.hex_max_size);
  }

  // Set layer-specific logging options.
  auto& phy_logger = srslog::fetch_basic_logger("PHY", false);
  phy_logger.set_level(srslog::str_to_basic_level(gnb_cfg.log_cfg.phy_level));
  phy_logger.set_hex_dump_max_size(gnb_cfg.log_cfg.hex_max_size);

  for (const auto& id : {"MAC", "SCHED"}) {
    auto& mac_logger = srslog::fetch_basic_logger(id, true);
    mac_logger.set_level(srslog::str_to_basic_level(gnb_cfg.log_cfg.mac_level));
    mac_logger.set_hex_dump_max_size(gnb_cfg.log_cfg.hex_max_size);
  }

  auto& rlc_logger = srslog::fetch_basic_logger("RLC", false);
  rlc_logger.set_level(srslog::str_to_basic_level(gnb_cfg.log_cfg.rlc_level));
  rlc_logger.set_hex_dump_max_size(gnb_cfg.log_cfg.hex_max_size);

  auto& f1ap_logger = srslog::fetch_basic_logger("F1AP", false);
  f1ap_logger.set_level(srslog::str_to_basic_level(gnb_cfg.log_cfg.f1ap_level));
  f1ap_logger.set_hex_dump_max_size(gnb_cfg.log_cfg.hex_max_size);

  auto& f1u_logger = srslog::fetch_basic_logger("F1-U", false);
  f1u_logger.set_level(srslog::str_to_basic_level(gnb_cfg.log_cfg.f1u_level));
  f1u_logger.set_hex_dump_max_size(gnb_cfg.log_cfg.hex_max_size);

  auto& pdcp_logger = srslog::fetch_basic_logger("PDCP", false);
  pdcp_logger.set_level(srslog::str_to_basic_level(gnb_cfg.log_cfg.pdcp_level));
  pdcp_logger.set_hex_dump_max_size(gnb_cfg.log_cfg.hex_max_size);

  auto& sdap_logger = srslog::fetch_basic_logger("SDAP", false);
  sdap_logger.set_level(srslog::str_to_basic_level(gnb_cfg.log_cfg.sdap_level));
  sdap_logger.set_hex_dump_max_size(gnb_cfg.log_cfg.hex_max_size);

  auto& gtpu_logger = srslog::fetch_basic_logger("GTPU", false);
  gtpu_logger.set_level(srslog::str_to_basic_level(gnb_cfg.log_cfg.gtpu_level));
  gtpu_logger.set_hex_dump_max_size(gnb_cfg.log_cfg.hex_max_size);

  auto& rf_logger = srslog::fetch_basic_logger("RF", false);
  rf_logger.set_level(srslog::str_to_basic_level(gnb_cfg.log_cfg.radio_level));

  auto& fapi_logger = srslog::fetch_basic_logger("FAPI", true);
  fapi_logger.set_level(srslog::str_to_basic_level(gnb_cfg.log_cfg.fapi_level));

  // Log build info
  gnb_logger.info("Built in {} mode using {}", get_build_mode(), get_build_info());

  // Check and log included CPU features and check support by current CPU
  if (cpu_supports_included_features()) {
    gnb_logger.debug("Required CPU features: {}", get_cpu_feature_info());
  } else {
    // Quit here until we complete selection of the best matching implementation for the current CPU at runtime.
    gnb_logger.error("The CPU does not support the required CPU features that were configured during compile time: {}",
                     get_cpu_feature_info());
    report_error("The CPU does not support the required CPU features that were configured during compile time: {}\n",
                 get_cpu_feature_info());
  }

  // Check some common causes of performance issues and
  // print a warning if required.
  check_cpu_governor(gnb_logger);
  check_drm_kms_polling(gnb_logger);

  // Set layer-specific pcap options.
  std::unique_ptr<dlt_pcap> f1ap_p = std::make_unique<dlt_pcap_impl>(PCAP_F1AP_DLT, "F1AP");
  if (gnb_cfg.pcap_cfg.f1ap.enabled) {
    f1ap_p->open(gnb_cfg.pcap_cfg.f1ap.filename.c_str());
  }
  std::unique_ptr<mac_pcap> mac_p = std::make_unique<mac_pcap_impl>();
  if (gnb_cfg.pcap_cfg.mac.enabled) {
    mac_p->open(gnb_cfg.pcap_cfg.mac.filename.c_str());
  }

  worker_manager workers{gnb_cfg};

  // Create manager of timers for DU, which will be driven by the PHY slot ticks.
  timer_manager app_timers{256};

  // Create F1-U connector
  std::unique_ptr<f1u_local_connector> f1u_conn = std::make_unique<f1u_local_connector>();

  // Create IO broker.
  std::unique_ptr<io_broker> epoll_broker = create_io_broker(io_broker_type::epoll);

  // Create F1AP adapter.
  std::unique_ptr<srsran::f1ap_network_adapter> f1ap_adapter =
      std::make_unique<srsran::f1ap_network_adapter>("F1AP", *epoll_broker, *f1ap_p);

  gnb_logger.info("Connecting to CU ({})..", f1ap_nw_config.connect_address, f1ap_nw_config.connect_port);
  std::unique_ptr<sctp_network_gateway> sctp_gateway =
      create_sctp_network_gateway({f1ap_nw_config, *f1ap_adapter, *f1ap_adapter});
    
  // Connect F1AP adapter to SCTP network gateway.
  f1ap_adapter->connect_gateway(sctp_gateway.get(), sctp_gateway.get());

  gnb_logger.info("CU connection established");

  // Create console helper object for commands and metrics printing.
  du_console_helper console(*epoll_broker);
  console.on_app_starting();

  // Create radio logger.
  std::unique_ptr<radio_notification_handler> radio_event_logger;
  if (rf_logger.warning.enabled()) {
    radio_event_logger = std::make_unique<radio_notification_handler_logger>(nullptr, rf_logger);
  }

  // Create radio.
  radio_notification_handler_counter radio_event_counter(std::move(radio_event_logger));
  auto                               radio = build_radio(*workers.radio_exec, radio_event_counter, gnb_cfg);
  if (radio == nullptr) {
    report_error("Unable to create radio session.\n");
  }
  gnb_logger.info("Radio driver '{}' created successfully", gnb_cfg.rf_driver_cfg.device_driver);

  // Create lower and upper PHY adapters.
  phy_error_adapter             phy_err_printer("info");
  phy_rx_symbol_adapter         phy_rx_adapter;
  phy_rg_gateway_adapter        rg_gateway_adapter;
  phy_timing_adapter            phy_time_adapter;
  phy_rx_symbol_request_adapter phy_rx_symbol_req_adapter;

  // Create lower PHY.
  lower_phy_configuration   lower_phy_config = create_lower_phy_configuration(&radio->get_baseband_gateway(),
                                                                            &phy_rx_adapter,
                                                                            &phy_time_adapter,
                                                                            &phy_err_printer,
                                                                            *workers.lower_phy_tx_exec,
                                                                            *workers.lower_phy_rx_exec,
                                                                            *workers.lower_phy_dl_exec,
                                                                            *workers.lower_phy_ul_exec,
                                                                            *workers.lower_prach_exec,
                                                                            gnb_cfg);
  static constexpr unsigned max_nof_prach_concurrent_requests = 11;
  auto                      lower = create_lower_phy(lower_phy_config, max_nof_prach_concurrent_requests);
  report_fatal_error_if_not(lower, "Unable to create lower PHY.");
  gnb_logger.info("Lower PHY created successfully");

  auto upper = create_upper_phy(gnb_cfg,
                                &rg_gateway_adapter,
                                workers.upper_dl_exec.get(),
                                workers.upper_pucch_exec.get(),
                                workers.upper_pusch_exec.get(),
                                workers.upper_prach_exec.get(),
                                &phy_rx_symbol_req_adapter);
  report_fatal_error_if_not(upper, "Unable to create upper PHY.");
  gnb_logger.info("Upper PHY created successfully");

  // Make connections between upper and lower PHYs.
  phy_rx_adapter.connect(&upper->get_rx_symbol_handler());
  phy_time_adapter.connect(&upper->get_timing_handler());
  rg_gateway_adapter.connect(&lower->get_rg_handler());
  phy_rx_symbol_req_adapter.connect(&lower->get_request_handler());

  // Create FAPI adaptors.
  std::vector<du_cell_config> du_cfg = generate_du_cell_config(gnb_cfg);
  unsigned                    sector = du_cfg.size() - 1;
  subcarrier_spacing          scs    = du_cfg.front().scs_common;

  auto phy_adaptor = build_phy_fapi_adaptor(sector,
                                            scs,
                                            scs,
                                            upper->get_downlink_processor_pool(),
                                            upper->get_downlink_resource_grid_pool(),
                                            upper->get_uplink_request_processor(),
                                            upper->get_uplink_resource_grid_pool(),
                                            upper->get_uplink_slot_pdu_repository(),
                                            upper->get_downlink_pdu_validator(),
                                            upper->get_uplink_pdu_validator(),
                                            generate_prach_config_tlv(du_cfg),
                                            generate_carrier_config_tlv(gnb_cfg));
  report_fatal_error_if_not(phy_adaptor, "Unable to create PHY adaptor.");
  upper->set_rx_results_notifier(phy_adaptor->get_rx_results_notifier());
  upper->set_timing_notifier(phy_adaptor->get_timing_notifier());

  fapi_slot_last_message_dummy                      last_msg_dummy;
  std::unique_ptr<fapi::slot_message_gateway>       logging_slot_gateway;
  std::unique_ptr<fapi::slot_data_message_notifier> logging_slot_data_notifier;
  std::unique_ptr<fapi::slot_time_message_notifier> logging_slot_time_notifier;
  std::unique_ptr<fapi_adaptor::mac_fapi_adaptor>   mac_adaptor;
  if (gnb_cfg.log_cfg.fapi_level == "debug") {
    // Create gateway loggers and intercept MAC adaptor calls.
    logging_slot_gateway = fapi::create_logging_slot_gateway(phy_adaptor->get_slot_message_gateway());
    report_fatal_error_if_not(logging_slot_gateway, "Unable to create logger for slot data notifications.");
    mac_adaptor = build_mac_fapi_adaptor(0, scs, *logging_slot_gateway, last_msg_dummy);

    // Create notification loggers.
    logging_slot_data_notifier = fapi::create_logging_slot_data_notifier(mac_adaptor->get_slot_data_notifier());
    report_fatal_error_if_not(logging_slot_data_notifier, "Unable to create logger for slot data notifications.");
    logging_slot_time_notifier = fapi::create_logging_slot_time_notifier(mac_adaptor->get_slot_time_notifier());
    report_fatal_error_if_not(logging_slot_time_notifier, "Unable to create logger for slot time notifications.");

    // Connect the PHY adaptor with the loggers to intercept PHY notifications.
    phy_adaptor->set_slot_time_message_notifier(*logging_slot_time_notifier);
    phy_adaptor->set_slot_data_message_notifier(*logging_slot_data_notifier);
  } else {
    mac_adaptor = build_mac_fapi_adaptor(0, scs, phy_adaptor->get_slot_message_gateway(), last_msg_dummy);
    report_fatal_error_if_not(mac_adaptor, "Unable to create MAC adaptor.");
    phy_adaptor->set_slot_time_message_notifier(mac_adaptor->get_slot_time_notifier());
    phy_adaptor->set_slot_data_message_notifier(mac_adaptor->get_slot_data_notifier());
  }
  gnb_logger.info("FAPI adaptors created successfully");

  // DU QoS config
  const std::map<five_qi_t, du_qos_config>& du_qos_cfg = generate_du_qos_config(gnb_cfg);
  for (const auto& it : du_qos_cfg) {
    gnb_logger.debug("QoS RLC configuration: 5QI={} RLC={}", it.first, it.second.rlc);
  }

  // Cell configuration.
  phy_dummy phy(mac_adaptor->get_cell_result_notifier());

  // BENTODO: F1U! GTPU?
  srs_du::du_high_configuration du_hi_cfg = {};
  du_hi_cfg.du_mng_executor               = workers.du_ctrl_exec.get();
  du_hi_cfg.ue_executors                  = &*workers.ue_exec_mapper;
  du_hi_cfg.cell_executors                = &*workers.cell_exec_mapper;
  du_hi_cfg.f1ap_notifier                 = f1ap_adapter.get();
  du_hi_cfg.f1u_gw                        = f1u_conn->get_f1u_du_gateway();
  du_hi_cfg.phy_adapter                   = &phy;
  du_hi_cfg.timers                        = &app_timers;
  du_hi_cfg.cells                         = du_cfg;
  du_hi_cfg.metrics_notifier              = &console.get_metrics_notifier();
  du_hi_cfg.sched_cfg                     = generate_scheduler_expert_config(gnb_cfg);
  du_hi_cfg.qos                           = du_qos_cfg;
  du_hi_cfg.pcap                          = mac_p.get();
  if (gnb_cfg.test_mode_cfg.test_ue.rnti != INVALID_RNTI) {
    du_hi_cfg.test_cfg.test_ue = srs_du::du_test_config::test_ue_config{gnb_cfg.test_mode_cfg.test_ue.rnti,
                                                                        gnb_cfg.test_mode_cfg.test_ue.pdsch_active,
                                                                        gnb_cfg.test_mode_cfg.test_ue.pusch_active};
  }

  srs_du::du_high du_obj(du_hi_cfg);
  f1ap_adapter->connect_f1ap(&du_obj.get_f1ap_message_handler(), NULL); // BENTODO: Add event notifier!
  gnb_logger.info("DU-High created successfully");

  // Start execution.
  gnb_logger.info("Starting DU-High...");
  du_obj.start();
  gnb_logger.info("DU-High started successfully");

  // Give some time to the MAC to start.
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Configure the DU slot handler.
  du_cell_index_t cell_id = to_du_cell_index(0);
  mac_adaptor->set_cell_slot_handler(du_obj.get_slot_handler(cell_id));
  mac_adaptor->set_cell_rach_handler(du_obj.get_rach_handler(cell_id));
  mac_adaptor->set_cell_pdu_handler(du_obj.get_pdu_handler(cell_id));
  mac_adaptor->set_cell_crc_handler(du_obj.get_control_information_handler(cell_id));

  // Start processing.
  gnb_logger.info("Starting lower PHY...");
  radio->start();
  lower->get_controller().start();
  gnb_logger.info("Lower PHY started successfully");

  console.set_cells(du_hi_cfg.cells);
  console.on_app_running();

  unsigned count = 0;
  while (is_running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if ((++count) == 10) {
      radio_event_counter.print();
      count = 0;
    }
  }

  console.on_app_stopping();

  mac_p->close();

  gnb_logger.info("Stopping radio...");
  radio->stop();
  gnb_logger.info("Radio notify_stop successfully");

  gnb_logger.info("Stopping lower PHY...");
  lower->get_controller().stop();
  gnb_logger.info("Lower PHY notify_stop successfully");

  gnb_logger.info("Closing DU-high...");
  du_obj.stop();
  gnb_logger.info("DU-high closed successfully");

  gnb_logger.info("Closing network connections...");
  f1ap_adapter->disconnect_gateway();
  gnb_logger.info("Network connections closed successfully");

  gnb_logger.info("Stopping executors...");
  workers.stop();
  gnb_logger.info("Executors notify_stop successfully");

  srslog::flush();

  return 0;
}

/// \endcond
