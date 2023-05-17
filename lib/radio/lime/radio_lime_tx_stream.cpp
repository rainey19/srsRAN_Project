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

#include "radio_lime_tx_stream.h"

using namespace srsran;

// TODO: use user_data to pass stream params that include samplerate
void radio_lime_tx_stream::recv_async_msg(bool is_tx, const lime::SDRDevice::StreamStats *stream_stats, void* user_data)
{
  // Handle event.
  radio_notification_handler::event_description event_description = {};
  event_description.stream_id                                     = stream_id;
  event_description.channel_id                                    = 0;
  event_description.source                                        = radio_notification_handler::event_source::TRANSMIT;
  event_description.type                                          = radio_notification_handler::event_type::UNDEFINED;

  if(is_tx)
  {
    if (stream_stats->underrun)
    {
      event_description.type = radio_notification_handler::event_type::UNDERFLOW;
      state_fsm.async_event_late_underflow(stream_stats->timestamp);
    }

    if (stream_stats->overrun || stream_stats->loss)
    {
      event_description.type = radio_notification_handler::event_type::OTHER;
    }

    notifier.on_radio_rt_event(event_description);
  }
  else
  {
    // Tx dropped packets are reported from Rx received packet flags
    if (stream_stats->late)
    {
      event_description.type = radio_notification_handler::event_type::LATE;
      state_fsm.async_event_late_underflow(stream_stats->timestamp);
      notifier.on_radio_rt_event(event_description);
    }
  }
}

bool radio_lime_tx_stream::transmit_block(unsigned&               nof_txd_samples,
                                         baseband_gateway_buffer& buffs,
                                         unsigned                 buffer_offset,
                                         uint64_t                 time_spec)
{
  // Prepare metadata.
  lime::SDRDevice::StreamMeta meta;

  // Extract number of samples.
  unsigned num_samples = buffs[0].size() - buffer_offset;

  // Make sure the number of channels is equal.
  report_fatal_error_if_not(buffs.get_nof_channels() == nof_channels, "Number of channels does not match.");

  // Run states.
  if (!state_fsm.transmit_block(meta, time_spec)) {
    nof_txd_samples = num_samples;
    return true;
  }

  // Flatten buffers.
  static_vector<void*, RADIO_MAX_NOF_CHANNELS> buffs_flat_ptr(nof_channels);
  for (unsigned channel = 0; channel != nof_channels; ++channel) {
    buffs_flat_ptr[channel] = (void**)buffs[channel].subspan(buffer_offset, num_samples).data();
  }

  const void** buffer = const_cast<const void **>(buffs_flat_ptr.data());

  // Safe transmission.
  return safe_execution([this, &buffer, num_samples, &meta, &nof_txd_samples]() {
    nof_txd_samples = stream->StreamTx(0, buffer, num_samples, &meta);
  });
}

bool make_arg_pair(std::string arg, std::pair<std::string, std::string>& pair)
{
  try
  {
    size_t x    = arg.find("=");
    pair.first  = arg.substr(0, x);
    pair.second = arg.substr(x);
    return 1;
  }
  catch (...)
  {
    printf("Error parsing argument: %s\n", arg.c_str());
    return false;
  }
}

bool split_args(std::string args, std::vector<std::pair<std::string, std::string>>& arg_list)
{
  std::string _store_str;
  std::stringstream _stringstream(args);

  while (getline(_stringstream, _store_str, ','))
  {
    std::pair<std::string, std::string> pair;
    if (!make_arg_pair(_store_str, pair))
    {
      printf("Could not parse radio args!\n");
      return false;
    }
    arg_list.push_back(pair);
  }
  return true;
}

radio_lime_tx_stream::radio_lime_tx_stream(std::shared_ptr<LimeHandle> device,
                                          const stream_description&    description,
                                          task_executor&               async_executor_,
                                          radio_notification_handler&  notifier_) :
  stream_id(description.id),
  async_executor(async_executor_),
  notifier(notifier_),
  stream(device->dev()),
  srate_hz(description.srate_hz),
  nof_channels(description.ports.size())
{
  srsran_assert(std::isnormal(srate_hz) && (srate_hz > 0.0), "Invalid sampling rate {}.", srate_hz);

  // int availableTxChannels = LMS_GetNumChannels(stream, lime::Dir::Tx);
  // if (availableTxChannels < nof_channels)
  // {
  //   printf("Error: device supports only %i Tx channels, required %i\n", availableTxChannels, nof_channels);
  //   return;
  // }

  // Build stream arguments.
  lime::SDRDevice::StreamConfig::DataFormat wire_format;
  switch (description.otw_format) {
    case radio_configuration::over_the_wire_format::DEFAULT:
    case radio_configuration::over_the_wire_format::SC16:
      wire_format = lime::SDRDevice::StreamConfig::I16;
      break;
    case radio_configuration::over_the_wire_format::SC12:
      wire_format = lime::SDRDevice::StreamConfig::I12;
      break;
    case radio_configuration::over_the_wire_format::SC8:
    default:
      printf("Error:  failed to create transmit stream %d. invalid OTW format!\n", stream_id);
      return;
  }

  device->GetStreamConfig().linkFormat    = wire_format;
  device->GetStreamConfig().format        = lime::SDRDevice::StreamConfig::F32;
  device->GetStreamConfig().txCount       = nof_channels;
  device->GetStreamConfig().alignPhase    = (nof_channels>1) ? true : false;
  for (unsigned int i=0; i<nof_channels; i++)
  {
    device->GetStreamConfig().txChannels[i] = i;
    device->GetDeviceConfig().channel[i].tx.enabled = 1;
    device->GetDeviceConfig().channel[i].tx.sampleRate = srate_hz;
  }

  // Parse out optional arguments.
  if (!description.args.empty())
  {
    std::vector<std::pair<std::string, std::string>> args;
    split_args(description.args, args);

    for (auto& arg : args)
    {
      if (arg.first == "nrbandwidth")
      {
        unsigned long nr_bw = std::stoul(arg.second, nullptr, 10);
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].tx.lpf = (nr_bw*1e6) / 2;
      }
      else if (arg.first == "lpf")
      {
        unsigned long lpf = std::stoul(arg.second, nullptr, 10);
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].tx.lpf = lpf;
      }
      else if (arg.first == "oversample")
      {
        unsigned long oversample = std::stoul(arg.second, nullptr, 10);
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].tx.oversample = oversample;
      }
      else if (arg.first == "gfir")
      {
        unsigned long gfir = std::stoul(arg.second, nullptr, 10);
        for (unsigned int i=0; i<nof_channels; i++)
        {
          device->GetDeviceConfig().channel[i].tx.gfir.enabled = true;
          device->GetDeviceConfig().channel[i].tx.gfir.bandwidth = gfir;
        }
      }
      else if (arg.first == "calibrate")
      {
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].tx.calibrate = true;
      }
    }
  } 

  // Set max packet size.
  // BENTODO: This might need to be 256?
  max_packet_size = (wire_format == lime::SDRDevice::StreamConfig::I12 ? 1360 : 1020)/nof_channels;

  // Notify FSM that it was successfully initialized.
  state_fsm.init_successful();

  // Create asynchronous task.
  // TODO: doesn't work because callback is non-static member function
  // device->GetStreamConfig().statusCallback = recv_async_msg;
}

void radio_lime_tx_stream::transmit(baseband_gateway_buffer& data, const baseband_gateway_transmitter::metadata& tx_md)
{
  // Protect stream transmitter.
  std::unique_lock<std::mutex> lock(stream_transmit_mutex);

  uint64_t time_spec = tx_md.ts;

  unsigned nsamples          = data.get_nof_samples();
  unsigned txd_samples_total = 0;

  // Receive stream in multiple blocks.
  while (txd_samples_total < nsamples) {
    unsigned txd_samples = 0;
    if (!transmit_block(txd_samples, data, txd_samples_total, time_spec)) {
      printf("Error: failed transmitting packet. %s.\n", get_error_message().c_str());
      return;
    }

    // Save timespec for first block.
    time_spec += txd_samples;

    // Increment the total amount of received samples.
    txd_samples_total += txd_samples;
  }
}

void radio_lime_tx_stream::stop()
{
  state_fsm.stop();
  stream->StreamStop(0);
}

void radio_lime_tx_stream::wait_stop()
{
  state_fsm.wait_stop();
}

unsigned radio_lime_tx_stream::get_buffer_size() const
{
  return max_packet_size;
}