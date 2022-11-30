/*
 *
 * Copyright 2013-2022 Software Radio Systems Limited
 *
 * By using this file, you agree to the terms and conditions set
 * forth in the LICENSE file which can be found at the top level of
 * the distribution.
 *
 */

#pragma once

#include "adapters/cu_up_processor_adapters.h"
#include "adapters/du_processor_adapters.h"
#include "adapters/f1c_adapters.h"
#include "adapters/ngc_adapters.h"
#include "cu_up_processor_impl.h"
#include "du_processor_impl.h"
#include "task_schedulers/cu_up_task_scheduler.h"
#include "task_schedulers/du_task_scheduler.h"
#include "task_schedulers/ue_task_scheduler.h"
#include "ue_manager.h"
#include "srsgnb/cu_cp/cu_cp.h"
#include "srsgnb/cu_cp/cu_cp_configuration.h"
#include "srsgnb/f1c/cu_cp/f1c_cu.h"
#include "srsgnb/support/async/async_task_loop.h"
#include "srsgnb/support/executors/task_executor.h"
#include "srsgnb/support/executors/task_worker.h"
#include "srsgnb/support/timers.h"
#include <memory>
#include <unordered_map>

namespace srsgnb {
namespace srs_cu_cp {

class cu_cp final : public cu_cp_interface
{
public:
  explicit cu_cp(const cu_cp_configuration& cfg_);
  ~cu_cp();

  void start();
  void stop();

  // DU interface
  size_t                  get_nof_dus() const override;
  size_t                  get_nof_ues() const override;
  f1c_message_handler&    get_f1c_message_handler(du_index_t du_index) override;
  f1c_statistics_handler& get_f1c_statistics_handler(du_index_t du_index) override;

  // CU-CP CU-UP interface
  size_t              get_nof_cu_ups() const override;
  e1_message_handler& get_e1_message_handler(cu_up_index_t cu_up_index) override;

  // NG interface
  ngc_message_handler& get_ngc_message_handler() override;
  ngc_event_handler&   get_ngc_event_handler() override;

  bool amf_is_connected() override { return amf_connected; };

  // DU connection notifier
  void on_new_du_connection() override;

  // DU handler
  void handle_du_remove_request(const du_index_t du_index) override;
  void handle_rrc_ue_creation(du_index_t du_index, ue_index_t ue_index, rrc_ue_interface* rrc_ue) override;

  // CU-UP connection notifier
  void on_new_cu_up_connection() override;

  // CU-UP handler
  void handle_cu_up_remove_request(const cu_up_index_t cu_up_index) override;

  // ngc_connection_notifier
  void on_amf_connection() override;
  void on_amf_connection_drop() override;

private:
  /// \brief Adds a DU processor object to the CU-CP.
  /// \return The DU index of the added DU processor object.
  du_index_t add_du();

  /// \brief Removes the specified DU processor object from the CU-CP.
  /// \param[in] du_index The index of the DU processor to delete.
  void remove_du(du_index_t du_index);

  /// \brief Find a DU object.
  /// \param[in] du_index The index of the DU processor object.
  /// \return The DU processor object.
  du_processor_interface& find_du(du_index_t du_index);

  /// \brief Get the next available index from the DU processor database.
  /// \return The DU index.
  du_index_t get_next_du_index();

  /// \brief Adds a CU-UP processor object to the CU-CP.
  /// \return The CU-UP index of the added CU-UP processor object.
  cu_up_index_t add_cu_up();

  /// \brief Removes the specified CU-UP processor object from the CU-CP.
  /// \param[in] cu_up_index The index of the CU-UP processor to delete.
  void remove_cu_up(cu_up_index_t cu_up_index);

  /// \brief Find a CU-UP object.
  /// \param[in] cu_up_index The index of the CU-UP processor object.
  /// \return The CU-UP processor object.
  cu_up_processor_interface& find_cu_up(cu_up_index_t cu_up_index);

  /// \brief Get the next available index from the CU-UP processor database.
  /// \return The CU-UP index.
  cu_up_index_t get_next_cu_up_index();

  cu_cp_configuration cfg;
  timer_manager       timers;

  // Handler for DU tasks.
  async_task_sequencer main_ctrl_loop;

  // logger
  srslog::basic_logger& logger = srslog::fetch_basic_logger("CU-CP");

  // Components
  std::unique_ptr<ngc_interface> ngc_entity;

  slotted_array<std::unique_ptr<du_processor_interface>, MAX_NOF_DUS>       du_db;
  slotted_array<std::unique_ptr<cu_up_processor_interface>, MAX_NOF_CU_UPS> cu_up_db;

  ue_manager ue_mng;

  // UE task scheduler
  ue_task_scheduler ue_task_sched;

  // DU task scheduler
  du_task_scheduler du_task_sched;

  // CU-UP task scheduler
  cu_up_task_scheduler cu_up_task_sched;

  // DU processor to CU-CP adapters
  du_processor_to_cu_cp_task_scheduler du_processor_task_sched;
  du_processor_cu_cp_adapter           du_processor_ev_notifier;

  // CU-UP processor to CU-CP adapter
  cu_up_processor_to_cu_cp_task_scheduler cu_up_processor_task_sched;

  // F1C to CU-CP adapter
  f1c_cu_cp_adapter f1c_ev_notifier;

  // CU-UP processor to CU-CP adapter
  cu_up_processor_cu_cp_adapter cu_up_processor_ev_notifier;

  // RRC UE to NGC adapter
  rrc_ue_ngc_adapter rrc_ue_ngc_notifier;

  // NGC to RRC UE adapter array
  slotted_array<ngc_rrc_ue_adapter, MAX_NOF_CU_UES> ngc_rrc_ue_ev_notifiers;

  std::atomic<bool> amf_connected = {false};
};

} // namespace srs_cu_cp
} // namespace srsgnb
