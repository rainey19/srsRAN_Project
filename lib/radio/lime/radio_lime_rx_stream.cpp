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

#include "radio_lime_rx_stream.h"

using namespace srsran;

bool radio_lime_rx_stream::receive_block(unsigned&                       nof_rxd_samples,
                                         baseband_gateway_buffer_writer& data,
                                         unsigned                        offset,
                                         lime::SDRDevice::StreamMeta&    md)
{
  // Extract number of samples.
  unsigned num_samples = data.get_nof_samples() - offset;

  // Ignore reception if it is not streaming.
  if (state != states::STREAMING) {
    nof_rxd_samples = num_samples;
    return true;
  }

  // Make sure the number of channels is equal.
  srsran_assert(data.get_nof_channels() == nof_channels, "Number of channels does not match.");

  // Flatten buffers.
  static_vector<void*, RADIO_MAX_NOF_CHANNELS> buffs_flat_ptr(nof_channels);
  for (unsigned channel = 0; channel != nof_channels; ++channel) {
    buffs_flat_ptr[channel] = (void*)data[channel].subspan(offset, num_samples).data();
  }

  lime::complex32f_t** buffer = (lime::complex32f_t**)buffs_flat_ptr.data();

  // Protect concurrent call of reception and stop.
  std::unique_lock<std::mutex> lock(stream_mutex);

  return safe_execution([this, buffer, num_samples, &md, &nof_rxd_samples]() {
    nof_rxd_samples = stream->StreamRx(chipIndex, buffer, num_samples, &md);
  });
}

radio_lime_rx_stream::radio_lime_rx_stream(std::shared_ptr<LimeHandle> device_,
                                         const stream_description&     description,
                                         radio_notification_handler&   notifier_) :
  id(description.id),
  srate_Hz(description.srate_Hz),
  notifier(notifier_),
  stream(device_->dev()),
  device(device_),
  nof_channels(description.ports.size()),
  logger(srslog::fetch_basic_logger("RF")),
  chipIndex(0)
{
  srsran_assert(std::isnormal(srate_Hz) && (srate_Hz > 0.0), "Invalid sampling rate {}.", srate_Hz);

  // int availableRxChannels = LMS_GetNumChannels(stream, lime::Dir::Rx);
  // if (availableRxChannels < nof_channels)
  // {
  //   logger.error("Device supports only {} Rx channels, required {}", availableRxChannels, nof_channels);
  //   return;
  // }

  // Build stream arguments.
  lime::SDRDevice::StreamConfig::DataFormat wire_format;
  switch (description.otw_format) {
    case radio_configuration::over_the_wire_format::DEFAULT:
    case radio_configuration::over_the_wire_format::SC16:
      wire_format = lime::SDRDevice::StreamConfig::DataFormat::I16;
      break;
    case radio_configuration::over_the_wire_format::SC12:
      wire_format = lime::SDRDevice::StreamConfig::DataFormat::I12;
      break;
    case radio_configuration::over_the_wire_format::SC8:
    default:
      logger.error("Failed to create receive stream {}. invalid OTW format!", id);
      return;
  }

  device->GetStreamConfig().rxCount = nof_channels;
  for (unsigned int i=0; i<nof_channels; i++)
  {
    device->GetStreamConfig().rxChannels[i] = i;
    device->GetDeviceConfig().channel[i].rx.enabled = true;
    device->GetDeviceConfig().channel[i].rx.sampleRate = srate_Hz;
    device->GetDeviceConfig().channel[i].rx.oversample = 2;
  }

  // Parse out optional arguments.
  if (!description.args.empty())
  {
    std::vector<std::pair<std::string, std::string>> args;
    if (!device->split_args(description.args, args))
    {
      logger.error("Failed to create receive stream {}. Could not parse args!", id);
      return;
    }

    for (auto& arg : args)
    {
      if (arg.first == "nrbandwidth")
      {
        unsigned long nr_bw = std::stoul(arg.second, nullptr, 10);
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].rx.lpf = (nr_bw*1e6) / 2;
      }
      else if (arg.first == "rxlpf")
      {
        unsigned long lpf = std::stoul(arg.second, nullptr, 10);
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].rx.lpf = lpf;
      }
      else if (arg.first == "rxoversample")
      {
        unsigned long oversample = std::stoul(arg.second, nullptr, 10);
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].rx.oversample = oversample;
      }
      else if (arg.first == "rxgfir")
      {
        unsigned long gfir = std::stoul(arg.second, nullptr, 10);
        for (unsigned int i=0; i<nof_channels; i++)
        {
          device->GetDeviceConfig().channel[i].rx.gfir.enabled = true;
          device->GetDeviceConfig().channel[i].rx.gfir.bandwidth = gfir;
        }
      }
      else if (arg.first == "rxcalibrate")
      {
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].rx.calibrate = true;
      }
      else if (arg.first == "rxtestSignal")
      {
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].rx.testSignal = true;
      }
      // 0=PATH_RFE_NONE, 1=PATH_RFE_LNAH, 2=PATH_RFE_LNAL, 3=PATH_RFE_LNAW
      else if (arg.first == "rxpathint")
      {
        unsigned long path = std::stoul(arg.second, nullptr, 10);
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].rx.path = path;
      }
      else if (arg.first == "rxpath")
      {
        bool match = false;
        auto paths = device->dev()->GetDescriptor().rfSOC[0].rxPathNames;
        for(uint j=0; j<paths.size(); ++j)
        {
          if (strcasecmp(paths[j].c_str(), arg.second.c_str()) == 0)
          {
            logger.debug("RX path: {} ({})", arg.second.c_str(), j);
            for (unsigned int i=0; i<nof_channels; i++)
              device->GetDeviceConfig().channel[i].rx.path = j;
            match = true;
            break;
          }
        }

        if (!match)
          logger.error("RX path {} not valid!", arg.second.c_str());
      }
      else if (arg.first == "usepoll")
      {
        if (!device->GetStreamConfig().extraConfig)
          device->GetStreamConfig().extraConfig = new lime::SDRDevice::StreamConfig::Extras();
        
        unsigned long mode = std::stoul(arg.second, nullptr, 10);
        device->GetStreamConfig().extraConfig->usePoll = (bool)mode;
      }
      else if (arg.first == "rxPacketsInBatch")
      {
        if (!device->GetStreamConfig().extraConfig)
          device->GetStreamConfig().extraConfig = new lime::SDRDevice::StreamConfig::Extras();
        
        unsigned long number = std::stoul(arg.second, nullptr, 10);
        device->GetStreamConfig().extraConfig->rxPacketsInBatch = number;
      }
      else if (arg.first == "rxSamplesInPacket")
      {
        if (!device->GetStreamConfig().extraConfig)
          device->GetStreamConfig().extraConfig = new lime::SDRDevice::StreamConfig::Extras();
        
        unsigned long number = std::stoul(arg.second, nullptr, 10);
        device->GetStreamConfig().extraConfig->rxSamplesInPacket = number;
      }
      else
        continue;
      
      logger.debug("Set {} to {}", arg.first, arg.second);
    }
  } 

  // Set max packet size.
  // TODO: This might need to be 256?
  max_packet_size = (wire_format == lime::SDRDevice::StreamConfig::DataFormat::I12 ? 1360 : 1020)/nof_channels;
  // max_packet_size = 256;

  state = states::SUCCESSFUL_INIT;
}

bool radio_lime_rx_stream::start(const uint64_t time_spec)
{
  if (state != states::SUCCESSFUL_INIT) {
    return true;
  }

  if (!safe_execution([this]() {
        stream->StreamSetup(device->GetStreamConfig(), chipIndex);
        stream->StreamStart(chipIndex);
      })) {
    logger.error("Failed to start receive stream {}. {}.", id, get_error_message().c_str());
  }

  // Transition to streaming state.
  state = states::STREAMING;

  return true;
}

baseband_gateway_receiver::metadata radio_lime_rx_stream::receive(baseband_gateway_buffer_writer& data)
{
  baseband_gateway_receiver::metadata ret = {};
  lime::SDRDevice::StreamMeta         md;
  unsigned                            nsamples            = data[0].size();
  unsigned                            rxd_samples_total   = 0;

  // Receive stream in multiple blocks.
  while (rxd_samples_total < nsamples) {
    unsigned rxd_samples = 0;
    if (!receive_block(rxd_samples, data, rxd_samples_total, md)) {
      logger.error("Failed receiving packet. {}.", get_error_message().c_str());
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
  }

  // If it reaches here, there is no error.
  return ret;
}

bool radio_lime_rx_stream::stop()
{
  // Transition state to stop before locking to prevent real time priority thread owning the lock constantly.
  state = states::STOP;

  // Protect concurrent call of stop and reception.
  std::unique_lock<std::mutex> lock(stream_mutex);

  // Try to stop the stream.
  stream->StreamStop(chipIndex);

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
