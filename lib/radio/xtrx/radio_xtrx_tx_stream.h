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

#include "radio_xtrx_exception_handler.h"
#include "radio_xtrx_api.h"
#include "radio_xtrx_tx_stream_fsm.h"
#include "srsran/gateways/baseband/baseband_gateway_buffer.h"
#include "srsran/gateways/baseband/baseband_gateway.h"
#include "srsran/radio/radio_configuration.h"
#include "srsran/radio/radio_notification_handler.h"
#include "srsran/support/executors/task_executor.h"
#include <mutex>

namespace srsran {
class radio_xtrx_tx_stream : public xtrx_exception_handler
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
  /// Owns the xtrx Tx stream.
  std::shared_ptr<XTRXHandle> stream;
  /// Protects concurrent stream transmit.
  std::mutex stream_transmit_mutex;
  /// Sampling rate in Hz.
  double srate_hz;
  /// Indicates the number of channels.
  unsigned nof_channels;
  /// Indicates the current internal state.
  radio_xtrx_tx_stream_fsm state_fsm;

  /// \brief Transmits a single baseband block.
  /// \param[out] nof_txd_samples Number of transmitted samples.
  /// \param[in] data Provides the buffers tpo transmit.
  /// \param[in] offset Indicates the sample offset in the transmit buffers.
  /// \param[in] time_spec Indicates the transmission timestamp.
  /// \return True if no exception is caught in the transmission process. Otherwise, false.
  bool transmit_block(unsigned&                   nof_txd_samples,
                      baseband_gateway_buffer&    data,
                      unsigned                    offset,
                      baseband_gateway_timestamp& time_spec);

public:
  /// Describes the necessary parameters to create an xtrx transmit stream.
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

  /// \brief Constructs an xtrx transmit stream.
  /// \param[in] xtrx_handle Provides the xtrx_handle context.
  /// \param[in] description Provides the stream configuration parameters.
  /// \param[in] async_executor_ Provides the asynchronous task executor.
  /// \param[in] notifier_ Provides the radio event notification handler.
  radio_xtrx_tx_stream(std::shared_ptr<XTRXHandle>& xtrx_handle,
                      const stream_description&     description,
                      task_executor&                async_executor_,
                      radio_notification_handler&   notifier_);

  /// \brief Transmits baseband signal.
  /// \param[in] data Provides the baseband buffers to transmit.
  /// \param[in] time_spec Indicates the transmission time.
  /// \return True if the transmission was successful. Otherwise, false.
  bool transmit(baseband_gateway_buffer& data, baseband_gateway_timestamp time_spec);

  /// Stop the transmission.
  void stop();
};
} // namespace srsran
