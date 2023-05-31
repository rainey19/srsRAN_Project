/*
 *
 * Copyright 2021-2023 Software Radio Systems Limited
 *
 * By using this file, you agree to the terms and conditions set
 * forth in the LICENSE file which can be found at the top level of
 * the distribution.
 *
 */

#pragma once

#include "e2sm_kpm_asn1_packer.h"
#include "procedures/e2_setup_procedure.h"
#include "procedures/e2_subscription_setup_procedure.h"
#include "srsran/asn1/e2ap/e2ap.h"
#include "srsran/e2/e2.h"
#include "srsran/ran/nr_cgi.h"
#include <map>
#include <memory>

namespace srsran {

class e2_event_manager;

class e2_impl final : public e2_interface
{
public:
  e2_impl(timer_factory timers_, e2_message_notifier& e2_pdu_notifier_, e2_subscription_manager& subscription_mngr_);

  /// E2 connection manager functions.
  async_task<e2_setup_response_message> handle_e2_setup_request(const e2_setup_request_message& request) override;

  void handle_e2_setup_response(const e2_setup_response_message& msg) override;

  /// E2_event_ handler functions.
  void handle_connection_loss() override {}

  /// E2 message handler functions.
  void handle_message(const e2_message& msg) override;

private:
  /// \brief Notify about the reception of an initiating message.
  /// \param[in] outcome The received initiating message.
  void handle_initiating_message(const asn1::e2ap::init_msg_s& outcome);

  /// \brief Notify about the reception of an successful outcome.
  /// \param[in] outcome The received successful outcome message.
  void handle_successful_outcome(const asn1::e2ap::successful_outcome_s& outcome);

  /// \brief Notify about the reception of an unsuccessful outcome.
  /// \param[in] outcome The received unsuccessful outcome message.
  void handle_unsuccessful_outcome(const asn1::e2ap::unsuccessful_outcome_s& outcome);

  /// \brief Notify about the reception of an ric subscription request message.
  /// \param[in] msg The received ric subscription request message.
  void handle_ric_subscription_request(const asn1::e2ap::ricsubscription_request_s& msg);

  /// \brief set the allowed ran functions from the e2 setuo response message.
  /// \param[in] msg The received ran_function_id from the e2 setup response message.
  void set_allowed_ran_functions(const uint16_t ran_function_id);

  // void add_e2sm_service(const uint16_t ran_function_id

  srslog::basic_logger&                               logger;
  timer_factory                                       timers;
  e2_message_notifier&                                pdu_notifier;
  std::map<uint16_t, asn1::e2ap::ra_nfunction_item_s> candidate_ran_functions;
  std::map<uint16_t, asn1::e2ap::ra_nfunction_item_s> allowed_ran_functions;
  e2_subscription_manager&                            subscription_mngr;
  e2_subscription_setup_procedure                     subscribe_proc;
  std::unique_ptr<e2_event_manager>                   events;

  unsigned current_transaction_id = 0; // store current E2AP transaction id
};

} // namespace srsran