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

#include "radio_xtrx_tx_stream.h"

using namespace srsran;

bool __attribute__((optimize("O0"))) radio_xtrx_tx_stream::transmit_block(unsigned&                  nof_txd_samples,
                                         baseband_gateway_buffer&    buffs,
                                         unsigned                    buffer_offset,
                                         baseband_gateway_timestamp& time_spec)
{
  // Prepare metadata.
  xtrx_send_ex_info_t metadata = {0};

  // Extract number of samples.
  unsigned num_samples = buffs[0].size() - buffer_offset;
  // printf("Asking to send %d samples\n", num_samples);

  // Make sure the number of channels is equal.
  report_fatal_error_if_not(buffs.get_nof_channels() == nof_channels, "Number of channels does not match.");

  // Flatten buffers.
  static_vector<void*, RADIO_MAX_NOF_CHANNELS> buffs_flat_ptr(nof_channels);
  for (unsigned channel = 0; channel != nof_channels; ++channel) {
    buffs_flat_ptr[channel] = (void*)buffs[channel].subspan(buffer_offset, num_samples).data();
  }

  metadata.buffer_count = nof_channels;
  metadata.buffers = buffs_flat_ptr.data();
  metadata.flags = XTRX_TX_DONT_BUFFER | XTRX_TX_NO_DISCARD;
  metadata.samples = num_samples;
  metadata.ts = time_spec;

  // Safe transmission.
  int res = safe_execution([this, &metadata, &nof_txd_samples]() {
    int res_ = xtrx_send_sync_ex(stream->dev(), &metadata);
    if (res_) { fprintf(stderr, "trx_xtrx_write: xtrx_send_burst_sync err=%d\n", res_); }
    nof_txd_samples = metadata.out_samples;
  });

  // Notify if there were lates
  if (metadata.out_txlatets)
  {
    radio_notification_handler::event_description event_description = {};
    event_description.stream_id                                     = stream_id;
    event_description.channel_id                                    = 0;
    event_description.source = radio_notification_handler::event_source::TRANSMIT;
    event_description.type   = radio_notification_handler::event_type::LATE;
    event_description.timestamp.emplace(time_spec);
    notifier.on_radio_rt_event(event_description);
  }

  // Notify if idk but it seems bad
  if (metadata.out_flags == XTRX_TX_DISCARDED_TO)
  {
    radio_notification_handler::event_description event_description = {};
    event_description.stream_id                                     = stream_id;
    event_description.channel_id                                    = 0;
    event_description.source = radio_notification_handler::event_source::TRANSMIT;
    event_description.type   = radio_notification_handler::event_type::OTHER;
    event_description.timestamp.emplace(time_spec);
    notifier.on_radio_rt_event(event_description);
  }

  return res;
}

__attribute__((optimize("O0"))) radio_xtrx_tx_stream::radio_xtrx_tx_stream(std::shared_ptr<XTRXHandle>& xtrx_handle,
                                         const stream_description&      description,
                                         task_executor&                 async_executor_,
                                         radio_notification_handler&    notifier_) :
  stream_id(description.id),
  async_executor(async_executor_),
  notifier(notifier_),
  srate_hz(description.srate_hz),
  nof_channels(description.ports.size())
{
  // Build stream arguments.
  stream = xtrx_handle;
  auto params = &stream->dev_params;

  params->tx.hfmt = XTRX_IQ_FLOAT32;
  params->tx.flags = 0;
	params->tx.paketsize = 8192;
  params->tx_repeat_buf = NULL;
  params->dir = XTRX_TRX;
	params->nflags = 0;

  switch (nof_channels)
  {
    default:
    case 1:
      params->tx.flags |= XTRX_RSP_SISO_MODE;
      params->tx.chs = XTRX_CH_A;
      break;
    case 2:
      params->tx.chs = XTRX_CH_AB;
      if (description.ports[0] == 1 && description.ports[1] == 0)
        params->rx.flags |= XTRX_RSP_SWAP_AB;
      break;
  }

  switch (description.otw_format) {
    case radio_configuration::over_the_wire_format::DEFAULT:
    case radio_configuration::over_the_wire_format::SC16:
      params->tx.wfmt = XTRX_WF_16;
      break;
    case radio_configuration::over_the_wire_format::SC12:
      params->tx.wfmt = XTRX_WF_12;
      break;
    case radio_configuration::over_the_wire_format::SC8:
      params->tx.wfmt = XTRX_WF_8;
      break;
  }	

  xtrx_stop(stream->dev(), XTRX_TX);

	if (xtrx_set_antenna(stream->dev(), XTRX_TX_AUTO)) {
		throw std::runtime_error("SoapyXTRX::setupStream() set antenna AUTO xtrx_set_antenna() err");
	}

  if (xtrx_tune_tx_bandwidth(stream->dev(), params->tx.chs, description.srate_hz, NULL)) {
		throw std::runtime_error("SoapyXTRX::setupStream() xtrx_tune_tx_bandwidth err");
  }

	// if (xtrx_run_ex(stream->dev(), params)) {
  //   throw std::runtime_error("SoapyXTRX::setupStream() xtrx_run_ex err");
	// }

  // Notify FSM that it was successfully initialized.
  state_fsm.init_successful();
}

bool radio_xtrx_tx_stream::transmit(baseband_gateway_buffer& data, baseband_gateway_timestamp time_spec)
{
  // Protect stream transmitter.
  std::unique_lock<std::mutex> lock(stream_transmit_mutex);

  unsigned nsamples          = data.get_nof_samples();
  unsigned txd_samples_total = 0;

  // Receive stream in multiple blocks.
  while (txd_samples_total < nsamples) {
    unsigned txd_samples = 0;
    if (!transmit_block(txd_samples, data, txd_samples_total, time_spec)) {
      printf("Error: failed transmitting packet. %s.\n", get_error_message().c_str());
      return false;
    }

    // Save timespec for first block.
    time_spec += txd_samples * srate_hz;

    // Increment the total amount of received samples.
    txd_samples_total += txd_samples;
  }

  // If it reaches here, there is no error.
  return true;
}

void radio_xtrx_tx_stream::stop()
{
  state_fsm.stop();
  if (xtrx_stop(stream->dev(), XTRX_TX)) {
    printf("Something went wrong stopping TX stream");
  }
}