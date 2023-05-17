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

#include "radio_lime_rx_stream.h"

using namespace srsran;

bool radio_lime_rx_stream::receive_block(unsigned&                   nof_rxd_samples,
                                        baseband_gateway_buffer&     data,
                                        unsigned                     offset,
                                        lime::SDRDevice::StreamMeta& md)
{
  if (starting_timestamp != 0)
  {
    md.useTimestamp = true;
    md.timestamp = starting_timestamp;
    starting_timestamp = 0;
  }
  else
  {
    md.useTimestamp = false;
  }
  md.flush = false;

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

  return safe_execution([this, buffs_flat_ptr, num_samples, &md, &nof_rxd_samples]() {
    nof_rxd_samples = stream->StreamRx(0, (void**)buffs_flat_ptr.data(), num_samples, &md);
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

radio_lime_rx_stream::radio_lime_rx_stream(std::shared_ptr<LimeHandle> device_,
                                         const stream_description&     description,
                                         radio_notification_handler&   notifier_) :
  id(description.id),
  srate_Hz(description.srate_Hz),
  notifier(notifier_),
  stream(device_->dev()),
  device(device_),
  nof_channels(description.ports.size())
{
  srsran_assert(std::isnormal(srate_Hz) && (srate_Hz > 0.0), "Invalid sampling rate {}.", srate_Hz);

  // int availableRxChannels = LMS_GetNumChannels(stream, lime::Dir::Rx);
  // if (availableRxChannels < nof_channels)
  // {
  //   printf("Error: device supports only %i Rx channels, required %i\n", availableRxChannels, nof_channels);
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
      printf("Error:  failed to create receive stream %d. invalid OTW format!\n", id);
      return;
  }

  for (unsigned int i=0; i<nof_channels; i++)
  {
    device->GetStreamConfig().rxChannels[i] = i;
    device->GetDeviceConfig().channel[i].rx.enabled = 1;
    device->GetDeviceConfig().channel[i].rx.sampleRate = srate_Hz;
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
          device->GetDeviceConfig().channel[i].rx.lpf = (nr_bw*1e6) / 2;
      }
      else if (arg.first == "lpf")
      {
        unsigned long lpf = std::stoul(arg.second, nullptr, 10);
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].rx.lpf = lpf;
      }
      else if (arg.first == "oversample")
      {
        unsigned long oversample = std::stoul(arg.second, nullptr, 10);
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].rx.oversample = oversample;
      }
      else if (arg.first == "gfir")
      {
        unsigned long gfir = std::stoul(arg.second, nullptr, 10);
        for (unsigned int i=0; i<nof_channels; i++)
        {
          device->GetDeviceConfig().channel[i].rx.gfir.enabled = true;
          device->GetDeviceConfig().channel[i].rx.gfir.bandwidth = gfir;
        }
      }
      else if (arg.first == "calibrate")
      {
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].rx.calibrate = true;
      }
    }
  } 

  // Set max packet size.
  // BENTODO: This might need to be 256?
  max_packet_size = (wire_format == lime::SDRDevice::StreamConfig::I12 ? 1360 : 1020)/nof_channels;

  state = states::SUCCESSFUL_INIT;
}

bool radio_lime_rx_stream::start(const uint64_t time_spec)
{
  if (state != states::SUCCESSFUL_INIT) {
    return true;
  }

  starting_timestamp = time_spec;

  if (!safe_execution([this]() {
        stream->Configure(device->GetDeviceConfig(), 0);
        stream->StreamSetup(device->GetStreamConfig(), 0);
        stream->StreamStart(0);
      })) {
    printf("Error: failed to start receive stream %d. %s.", id, get_error_message().c_str());
  }

  // Transition to streaming state.
  state = states::STREAMING;

  return true;
}

baseband_gateway_receiver::metadata radio_lime_rx_stream::receive(baseband_gateway_buffer& buffs)
{
  baseband_gateway_receiver::metadata ret = {};
  lime::SDRDevice::StreamMeta         md;
  unsigned                            nsamples            = buffs[0].size();
  unsigned                            rxd_samples_total   = 0;
  // unsigned                            timeout_trial_count = 0;

  // Receive stream in multiple blocks.
  while (rxd_samples_total < nsamples) {
    unsigned rxd_samples = 0;
    if (!receive_block(rxd_samples, buffs, rxd_samples_total, md)) {
      printf("Error: failed receiving packet. %s.\n", get_error_message().c_str());
      return {};
    }

    // Save timespec for first block.
    if (rxd_samples_total == 0) {
      ret.ts = md.timestamp;
    }

    // Increase the total amount of received samples.
    rxd_samples_total += rxd_samples;

    // Prepare notification event.
    radio_notification_handler::event_description event = {};
    event.stream_id                                     = id;
    event.channel_id                                    = radio_notification_handler::UNKNOWN_ID;
    event.source                                        = radio_notification_handler::event_source::RECEIVE;
    event.type                                          = radio_notification_handler::event_type::UNDEFINED;

    // TODO
    // Handle error.
    /*switch (md.error_code) {
      case lime::rx_metadata_t::ERROR_CODE_TIMEOUT:
        ++timeout_trial_count;
        if (timeout_trial_count >= 10) {
          printf("Error: exceeded maximum number of timed out transmissions.\n");
          return ret;
        }
        break;
      case lime::rx_metadata_t::ERROR_CODE_NONE:
        // Ignored.
        break;
      case lime::rx_metadata_t::ERROR_CODE_LATE_COMMAND:
        event.type = radio_notification_handler::event_type::LATE;
        break;
      case lime::rx_metadata_t::ERROR_CODE_OVERFLOW:
        event.type = radio_notification_handler::event_type::OVERFLOW;
        break;
      case lime::rx_metadata_t::ERROR_CODE_BROKEN_CHAIN:
      case lime::rx_metadata_t::ERROR_CODE_ALIGNMENT:
      case lime::rx_metadata_t::ERROR_CODE_BAD_PACKET:
        printf("Error: unhandled error in Rx metadata %s.", md.strerror().c_str());
        return ret;
    }

    // Notify if the event type was set.
    if (event.type != radio_notification_handler::event_type::UNDEFINED) {
      notifier.on_radio_rt_event(event);
    }*/
  }

  // If it reaches here, there is no error.
  return ret;
}

bool radio_lime_rx_stream::stop()
{
  // Protect concurrent call of stop and reception.
  std::unique_lock<std::mutex> lock(stream_mutex);

  // Transition state to stop before locking to prevent real time priority thread owning the lock constantly.
  state = states::STOP;

  // Try to stop the stream.
  stream->StreamStop(0);

  return true;
}

void radio_lime_rx_stream::wait_stop()
{
  // nothing to wait here
  return;
}

unsigned radio_lime_rx_stream::get_buffer_size() const
{
  return max_packet_size;
}