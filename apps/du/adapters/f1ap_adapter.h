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

#pragma once

#include "lib/f1ap/common/f1ap_asn1_packer.h"
#include "srsran/gateways/sctp_network_gateway_factory.h"
#include "srsran/f1ap/common/f1ap_common.h"
#include "srsran/support/io_broker/io_broker.h"

namespace srsran {

/// \brief F1AP bridge between DU and CU-CP using fast-path message passing.
class f1ap_network_adapter : public f1ap_message_notifier,
                             public f1ap_message_handler,
                             public sctp_network_gateway_control_notifier,
                             public network_gateway_data_notifier
{
public:
  explicit f1ap_network_adapter(const std::string& log_name,
                                io_broker&         broker_,
                                dlt_pcap&          pcap_) :
                                logger(srslog::fetch_basic_logger(log_name)),
                                broker(broker_),
                                pcap(pcap_)
  {
    if (gateway_ctrl_handler != nullptr) {
      broker.unregister_fd(gateway_ctrl_handler->get_socket_fd());
    }
  }

  void connect_gateway(sctp_network_gateway_controller*   gateway_ctrl_handler_,
                       sctp_network_gateway_data_handler* gateway_data_handler_)
  {
    gateway_data_handler = gateway_data_handler_;
    gateway_ctrl_handler = gateway_ctrl_handler_;

    packer = std::make_unique<f1ap_asn1_packer>(*gateway_data_handler, *this);

    if (!gateway_ctrl_handler->create_and_connect()) {
      report_error("Failed to create SCTP gateway.\n");
    }
    broker.register_fd(gateway_ctrl_handler->get_socket_fd(), [this](int fd) { gateway_ctrl_handler->receive(); });
  }

  void connect_f1ap(f1ap_message_handler* f1ap_msg_handler_, f1ap_event_handler* event_handler_)
  {
    f1ap_msg_handler = f1ap_msg_handler_;
    event_handler    = event_handler_;
  }

  void disconnect_gateway()
  {
    report_fatal_error_if_not(gateway_ctrl_handler, "Gateway handler not set.");
    broker.unregister_fd(gateway_ctrl_handler->get_socket_fd());

    gateway_ctrl_handler = nullptr;
    gateway_data_handler = nullptr;

    packer.reset();
  }

private:
  // F1AP calls interface to send (unpacked) F1AP PDUs
  void on_new_message(const f1ap_message& msg) override
  {
    logger.debug("Sending a PDU of type {}", msg.pdu.type().to_string());

    // this is the one from the f1ap class
    if (packer) {
      packer->handle_message(msg);
    } else {
      logger.debug("F1AP ASN1 packer disconnected, dropping msg");
    }
  }

  // SCTP network gateway calls interface to inject received PDUs (ASN1 packed)
  void on_new_pdu(byte_buffer pdu) override
  {
    if (packer) {
      packer->handle_packed_pdu(pdu);
    } else {
      logger.debug("F1AP ASN1 packer disconnected, dropping pdu");
    }
  }

  // The packer calls this interface to inject unpacked F1AP PDUs
  void handle_message(const f1ap_message& msg) override
  {
    logger.debug("Received a PDU of type {}", msg.pdu.type().to_string());
    report_fatal_error_if_not(f1ap_msg_handler, "F1AP handler not set.");
    f1ap_msg_handler->handle_message(msg);
  }

  void on_connection_loss() override
  {
    report_fatal_error_if_not(event_handler, "F1AP handler not set.");
    event_handler->handle_connection_loss();
  }

  void on_connection_established() override
  {
    // TODO: extend event interface to inform about connection establishment
    logger.debug("on_connection_established");
  }

  srslog::basic_logger& logger;
  io_broker&                         broker;
  dlt_pcap&                          pcap;
  std::unique_ptr<f1ap_asn1_packer>  packer;
  sctp_network_gateway_controller*   gateway_ctrl_handler = nullptr;
  sctp_network_gateway_data_handler* gateway_data_handler = nullptr;
  f1ap_message_handler*              f1ap_msg_handler     = nullptr;
  f1ap_event_handler*                event_handler        = nullptr;
};

}; // namespace srsran
