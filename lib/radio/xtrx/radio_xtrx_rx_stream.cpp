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

#include "radio_xtrx_rx_stream.h"

using namespace srsran;

bool radio_xtrx_rx_stream::receive_block(unsigned&               nof_rxd_samples,
                                        baseband_gateway_buffer& data,
                                        unsigned                 offset,
                                        xtrx_recv_ex_info&       metadata)
{
  // Extract number of samples.
  unsigned num_samples = data.get_nof_samples() - offset;

  // Protect concurrent call of reception and stop.
  std::unique_lock<std::mutex> lock(stream_mutex);

  // Ignore reception if it is not streaming.
  if (state != states::STREAMING) {
    nof_rxd_samples = num_samples;
    return true;
  }

  // Make sure the number of channels is equal.
  report_fatal_error_if_not(data.get_nof_channels() == nof_channels, "Number of channels does not match.");

  // Flatten buffers.
  static_vector<void*, RADIO_MAX_NOF_CHANNELS> buffs_flat_ptr(nof_channels);
  for (unsigned channel = 0; channel != nof_channels; ++channel) {
    buffs_flat_ptr[channel] = (void*)data[channel].subspan(offset, num_samples).data();
  }

  // Safe transmission.
	metadata.samples = num_samples;
	metadata.buffer_count = (unsigned)nof_channels;
	metadata.buffers = buffs_flat_ptr.data();
	metadata.flags = RCVEX_DONT_INSER_ZEROS; //RCVEX_EXTRA_LOG;

  int res = xtrx_recv_sync_ex(stream->dev(), &metadata);
  if (res) {
    fprintf(stderr, "trx_xtrx_read: xtrx_recv_sync count=%d err=%d\n", num_samples, res);
	  return false;
  }
  nof_rxd_samples = metadata.out_samples;

  return true;
}

radio_xtrx_rx_stream::radio_xtrx_rx_stream(std::shared_ptr<XTRXHandle>& xtrx_handle,
                                         const stream_description&      description,
                                         radio_notification_handler&    notifier_) :
  id(description.id),
  notifier(notifier_),
  nof_channels(description.ports.size())
{
  // Build stream arguments.
  stream = xtrx_handle;

  stream->dev_params.rx.hfmt = XTRX_IQ_FLOAT32;
  stream->dev_params.rx.flags = 0;
	stream->dev_params.rx.paketsize = 8192;
  stream->dev_params.dir = XTRX_TRX;
	stream->dev_params.nflags = 0;

  switch (nof_channels) {
    default:
    case 1:
      stream->dev_params.rx.flags |= XTRX_RSP_SISO_MODE;
      stream->dev_params.rx.chs = XTRX_CH_A;
      break;
    case 2:
      stream->dev_params.rx.chs = XTRX_CH_AB;
      if (description.ports[0] == 1 && description.ports[1] == 0)
        stream->dev_params.rx.flags |= XTRX_RSP_SWAP_AB;
      break;
  }

  switch (description.otw_format) {
    case radio_configuration::over_the_wire_format::DEFAULT:
    case radio_configuration::over_the_wire_format::SC16:
      stream->dev_params.rx.wfmt = XTRX_WF_16;
      break;
    case radio_configuration::over_the_wire_format::SC12:
      stream->dev_params.rx.wfmt = XTRX_WF_12;
      break;
    case radio_configuration::over_the_wire_format::SC8:
      stream->dev_params.rx.wfmt = XTRX_WF_8;
      break;
  }

  if (xtrx_set_antenna(stream->dev(), XTRX_RX_AUTO)) {
		throw std::runtime_error("SoapyXTRX::setupStream() set antenna AUTO xtrx_set_antenna() err");
	}

  xtrx_stop(stream->dev(), XTRX_RX);

  state = states::SUCCESSFUL_INIT;
}

bool __attribute__((optimize("O0"))) radio_xtrx_rx_stream::start(const baseband_gateway_timestamp& time_spec)
{
  if (state != states::SUCCESSFUL_INIT) {
    return true;
  }

  stream->dev_params.rx_stream_start = time_spec;

  if (xtrx_run_ex(stream->dev(), &stream->dev_params)) {
    printf("Error: failed to start receive stream %d. %s.", id, get_error_message().c_str());
    return false;
  }

  // Transition to streaming state.
  state = states::STREAMING;

  return true;
}

bool radio_xtrx_rx_stream::receive(baseband_gateway_buffer& buffs, baseband_gateway_timestamp& time_spec)
{
  xtrx_recv_ex_info md;
  unsigned          nsamples            = buffs[0].size();
  unsigned          rxd_samples_total   = 0;

  // Receive stream in multiple blocks
  while (rxd_samples_total < nsamples) {
    unsigned rxd_samples = 0;
    if (!receive_block(rxd_samples, buffs, rxd_samples_total, md)) {
      printf("Error: failed receiving packet. %s.\n", get_error_message().c_str());
      return false;
    }

    // Save timespec for first block.
    if (rxd_samples_total == 0) {
      time_spec = md.out_first_sample;
    }

    // Increment the total amount of received samples.
    rxd_samples_total += rxd_samples;

    // Prepare notification event.
    radio_notification_handler::event_description event = {};
    event.stream_id                                     = id;
    event.channel_id                                    = radio_notification_handler::UNKNOWN_ID;
    event.source                                        = radio_notification_handler::event_source::RECEIVE;
    event.type                                          = radio_notification_handler::event_type::UNDEFINED;

    // Handle error.
    if (md.out_events & RCVEX_EVENT_OVERFLOW) {
      event.type = radio_notification_handler::event_type::OVERFLOW;
    }
    else if (md.out_events & RCVEX_EVENT_FILLED_ZERO) {
      event.type = radio_notification_handler::event_type::UNDERFLOW;
    }
    else if (md.out_events == 0) {
      // No error code (ignored)
    }
    else {
      printf("Error: unhandled error in Rx metadata.");
      return false;
    }

    // Notify if the event type was set.
    if (event.type != radio_notification_handler::event_type::UNDEFINED) {
      notifier.on_radio_rt_event(event);
    }
  }

  // If it reaches here, there is no error.
  return true;
}

bool radio_xtrx_rx_stream::stop()
{
  // Protect concurrent call of stop and reception.
  std::unique_lock<std::mutex> lock(stream_mutex);

  // Transition state to stop before locking to prevent real time priority thread owning the lock constantly.
  state = states::STOP;

  // Try to stop the stream.
  if (xtrx_stop(stream->dev(), XTRX_RX)) {
    printf("Something went wrong stopping RX stream");
    return false;
  }

  return true;
}