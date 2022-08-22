/*
 *
 * Copyright 2013-2022 Software Radio Systems Limited
 *
 * By using this file, you agree to the terms and conditions set
 * forth in the LICENSE file which can be found at the top level of
 * the distribution.
 *
 */

#include "../../lib/pdcp/pdcp_entity_impl.h"
#include "srsgnb/pdcp/pdcp_config.h"
#include "srsgnb/support/timers.h"
#include <gtest/gtest.h>
#include <queue>

using namespace srsgnb;

template <std::size_t N>
byte_buffer make_byte_buffer_and_log(const std::array<uint8_t, N>& tv)
{
  byte_buffer sdu = {tv};
  return sdu;
}

template <std::size_t N>
byte_buffer_slice_chain make_rlc_byte_buffer_and_log(const std::array<uint8_t, N>& tv)
{
  byte_buffer             buf = {tv};
  byte_buffer_slice_chain pdu;
  pdu.push_back(std::move(buf));
  return pdu;
}

/// Mocking class of the surrounding layers invoked by the PDCP.
class pdcp_test_frame : public pdcp_rx_upper_data_notifier,
                        public pdcp_rx_upper_control_notifier,
                        public pdcp_tx_upper_control_notifier,
                        public pdcp_tx_lower_notifier
{
public:
  std::queue<byte_buffer> sdu_queue   = {};
  uint32_t                sdu_counter = 0;

  // PDCP RX upper layer data notifier
  void on_new_sdu(byte_buffer sdu) override
  {
    sdu_queue.push(std::move(sdu));
    sdu_counter++;
  }

  // PDCP RX upper layer control notifier
  void on_integrity_failure() override {}

  // PDCP TX/RX upper layer control notifier
  void on_protocol_failure() override {}

  // PDCP TX upper layer control notifier
  void on_max_hfn_reached() override {}

  // PDCP TX lower layer notifier
  void on_new_pdu(byte_buffer buf) override {}
  void on_discard_pdu(uint32_t pdcp_count) override {}
};

/// Fixture class for PDCP Tx tests
/// It requires TEST_P() and INSTANTIATE_TEST_SUITE_P() to create/spawn tests for each supported SN size
class pdcp_tx_test : public ::testing::Test, public ::testing::WithParamInterface<pdcp_sn_size>
{
protected:
  void SetUp() override
  {
    // init test's logger
    srslog::init();
    logger.set_level(srslog::basic_levels::debug);

    // init RLC logger
    srslog::fetch_basic_logger("PDCP", false).set_level(srslog::basic_levels::debug);
    srslog::fetch_basic_logger("PDCP", false).set_hex_dump_max_size(100);
  }

  void TearDown() override
  {
    // flush logger after each test
    srslog::flush();
  }

  /// \brief Initializes fixture according to size sequence number size
  /// \param sn_size_ size of the sequence number
  void init(pdcp_sn_size sn_size_)
  {
    logger.info("Creating PDCP ({} bit)", to_number(sn_size_));

    sn_size = sn_size_;

    // Set Rx config
    config.rx.sn_size = sn_size;
    // config.rx->t_reassembly      = 35;
    // config.rx->t_status_prohibit = 8;

    // Set Tx config
    config.tx.sn_size = sn_size;
    // config.tx->t_poll_retx     = 45;
    // config.tx->max_retx_thresh = 4;
    // config.tx->poll_pdu        = 4;
    // config.tx->poll_byte       = 25;

    // Create RLC entities
    std::unique_ptr<pdcp_entity> pdcp =
        std::make_unique<pdcp_entity_impl>(0, LCID_SRB1, config, test_frame, test_frame, test_frame, timers);

    // Bind interfaces

    pdcp_rx_lower      = &pdcp->get_rx_lower_interface();
    pdcp_tx_lower      = &pdcp->get_tx_lower_interface();
    pdcp_tx_upper_data = &pdcp->get_tx_upper_data_interface();
  }

  srslog::basic_logger& logger = srslog::fetch_basic_logger("TEST", false);

  pdcp_sn_size    sn_size = GetParam();
  pdcp_config     config  = {};
  timer_manager   timers;
  pdcp_test_frame test_frame = {};

  std::unique_ptr<pdcp_entity>  pdcp;
  pdcp_rx_lower_interface*      pdcp_rx_lower      = nullptr;
  pdcp_tx_lower_interface*      pdcp_tx_lower      = nullptr;
  pdcp_tx_upper_data_interface* pdcp_tx_upper_data = nullptr;
};

TEST_P(pdcp_tx_test, create_new_entity)
{
  init(GetParam());

  ASSERT_NE(pdcp_rx_lower, nullptr);
  ASSERT_NE(pdcp_tx_upper_data, nullptr);
  ASSERT_NE(pdcp_tx_lower, nullptr);
}

///////////////////////////////////////////////////////////////////
// Finally, instantiate all testcases for each supported SN size //
///////////////////////////////////////////////////////////////////
std::string test_param_info_to_string(const ::testing::TestParamInfo<pdcp_sn_size>& info)
{
  fmt::memory_buffer buffer;
  fmt::format_to(buffer, "{}bit", to_number(info.param));
  return fmt::to_string(buffer);
}

INSTANTIATE_TEST_SUITE_P(pdcp_tx_test_all_sn_sizes,
                         pdcp_tx_test,
                         ::testing::Values(pdcp_sn_size::size12bits, pdcp_sn_size::size18bits),
                         test_param_info_to_string);
