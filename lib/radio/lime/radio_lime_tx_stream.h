/*
 *
 * Copyright 2021-2024 Software Radio Systems Limited
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

#include "radio_lime_exception_handler.h"
#include "radio_lime_sdrdevice.h"
#include "radio_lime_tx_stream_fsm.h"
#include "srsran/gateways/baseband/baseband_gateway_transmitter.h"
#include "srsran/gateways/baseband/buffer/baseband_gateway_buffer_dynamic.h"
#include "srsran/gateways/baseband/buffer/baseband_gateway_buffer_reader.h"
#include "srsran/radio/radio_configuration.h"
#include "srsran/radio/radio_notification_handler.h"
#include "srsran/support/executors/task_executor.h"
#include <mutex>

namespace srsran {

/// Implements a gateway transmitter based on LIME transmit stream.
class radio_lime_tx_stream : public baseband_gateway_transmitter, public lime_exception_handler
{
private:
  /// Receive asynchronous message timeout in seconds.
  static constexpr double RECV_ASYNC_MSG_TIMEOUT_S = 0.001;
  /// Transmit timeout in seconds.
  static constexpr double TRANSMIT_TIMEOUT_S = 0.001;

  /// Indicates the stream identification for notifications.
  unsigned stream_id;
  /// Task executor for asynchronous messages.
  task_executor& async_executor;
  /// Radio notification interface.
  radio_notification_handler& notifier;
  /// Owns the LIME Tx stream.
  lime::SDRDevice* stream;
  std::shared_ptr<LimeHandle> device;
  /// Maximum number of samples in a single packet.
  unsigned max_packet_size;
  /// Protects concurrent stream transmit.
  std::mutex stream_transmit_mutex;
  /// Sampling rate in Hz.
  double srate_hz;
  /// Indicates the number of channels.
  unsigned nof_channels;
  /// Indicates the current internal state.
  radio_lime_tx_stream_fsm state_fsm;
  /// Handle to logger object
  srslog::basic_logger& logger;
  /// Index of the RFIC (for SDRs with multiple chipsets such as X3 or X8)
  uint8_t chipIndex;

  /// \brief Transmits a single baseband block.
  /// \param[out] nof_txd_samples Number of transmitted samples.
  /// \param[in] data             Buffer to transmit.
  /// \param[in] offset           Sample offset in the transmit buffers.
  /// \param[in] time_spec        Transmission timestamp.
  /// \return True if no exception is caught in the transmission process, false otherwise.
  bool transmit_block(unsigned&                             nof_txd_samples,
                      const baseband_gateway_buffer_reader& data,
                      unsigned                              offset,
                      uint64_t                              time_spec);

public:
  /// Describes the necessary parameters to create a Lime transmit stream.
  struct stream_description {
    /// Identifies the stream.
    unsigned id;
    /// Over-the-wire format.
    radio_configuration::over_the_wire_format otw_format;
    /// Sampling rate in Hz.
    double srate_hz;
    /// Stream arguments.
    std::string args;
    /// Indicates the port indexes for the stream.
    std::vector<size_t> ports;
  };

  /// \brief Constructs a Lime transmit stream.
  /// \param[in] usrp Provides the USRP context.
  /// \param[in] description Provides the stream configuration parameters.
  /// \param[in] async_executor_ Provides the asynchronous task executor.
  /// \param[in] notifier_ Provides the radio event notification handler.
  radio_lime_tx_stream(std::shared_ptr<LimeHandle> device_,
                      const stream_description&    description,
                      task_executor&               async_executor_,
                      radio_notification_handler&  notifier_);

  unsigned get_buffer_size() const;

  // See interface for documentation.
  void transmit(const baseband_gateway_buffer_reader&        data,
                const baseband_gateway_transmitter_metadata& metadata) override;

  /// Stop the transmission.
  void stop();

  /// Wait until radio is notify_stop.
  void wait_stop();
};
} // namespace srsran
