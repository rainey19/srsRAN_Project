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
#include "srsran/gateways/baseband/baseband_gateway_receiver.h"
#include "srsran/gateways/baseband/buffer/baseband_gateway_buffer_writer.h"
#include "srsran/radio/radio_configuration.h"
#include "srsran/radio/radio_notification_handler.h"
#include <mutex>

namespace srsran {

/// Implements a gateway receiver based on Lime receive stream.
class radio_lime_rx_stream : public lime_exception_handler, public baseband_gateway_receiver
{
private:
  /// Receive timeout in seconds.
  static constexpr double RECEIVE_TIMEOUT_S = 0.2f;
  /// Set to true for receiving data in a single packet.
  static constexpr bool ONE_PACKET = false;

  /// Defines the Rx stream internal states.
  enum class states { UNINITIALIZED, SUCCESSFUL_INIT, STREAMING, STOP };
  /// Indicates the current stream state.
  std::atomic<states> state = {states::UNINITIALIZED};
  /// Indicates the stream identification for notifications.
  unsigned id;
  /// Sampling rate in hertz.
  double srate_Hz;
  /// Radio notification interface.
  radio_notification_handler& notifier;
  /// Owns the Lime Tx stream.
  lime::SDRDevice* stream;
  std::shared_ptr<LimeHandle> device;
  /// Maximum number of samples in a single packet.
  unsigned max_packet_size;
  /// Indicates the number of channels.
  unsigned nof_channels;
  /// Protects stream from concurrent receive and stop.
  std::mutex stream_mutex;
  /// Handle to logger object
  srslog::basic_logger& logger;
  /// Index of the RFIC (for SDRs with multiple chipsets such as X3 or X8)
  uint8_t chipIndex;

  /// \brief Receives a single block of baseband samples.
  /// \param[out] nof_rxd_samples Indicate the number of samples received in the block.
  /// \param[in,out] buffs Provides the reception buffers.
  /// \param[in] buffer_offset Indicates the data offset in the reception buffers.
  /// \param[in] metadata Provides the reception metadata.
  /// \return True if no exception is caught. Otherwise false.
  bool receive_block(unsigned&                       nof_rxd_samples,
                     baseband_gateway_buffer_writer& buffs,
                     unsigned                        buffer_offset,
                     lime::SDRDevice::StreamMeta&    md);

public:
  /// Describes the necessary parameters to create a Lime transmit stream.
  struct stream_description {
    /// Identifies the stream.
    unsigned id;
    /// Sampling rate in hertz.
    double srate_Hz;
    /// Over-the-wire format.
    radio_configuration::over_the_wire_format otw_format;
    /// Stream arguments.
    std::string args;
    /// Indicates the port indexes for the stream.
    std::vector<size_t> ports;
  };

  /// \brief Constructs a receive Lime stream.
  /// \param[in] usrp Provides the USRP context.
  /// \param[in] description Provides the stream configuration parameters.
  /// \param[in] notifier_ Provides the radio event notification handler.
  radio_lime_rx_stream(std::shared_ptr<LimeHandle> device,
                      const stream_description&    description,
                      radio_notification_handler&  notifier_);

  /// \brief Starts the stream reception.
  /// \param[in] time_spec Indicates the start time of the stream.
  /// \return True if no exception is caught. Otherwise false.
  bool start(const uint64_t time_spec);

  /// Gets the optimal transmitter buffer size.
  unsigned get_buffer_size() const;

  // See interface for documentation.
  metadata receive(baseband_gateway_buffer_writer& data) override;

  /// \brief Stops the reception stream.
  /// \return True if no exception is caught. Otherwise false.
  bool stop();

  /// Wait until radio is notify_stop.
  void wait_stop();
};
} // namespace srsran
