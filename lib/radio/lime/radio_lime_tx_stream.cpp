/*
 *
 * Copyright 2021-2025 Software Radio Systems Limited
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
#include "srsran/gateways/baseband/buffer/baseband_gateway_buffer_reader_view.h"
#include "srsran/srsvec/zero.h"

using namespace srsran;

bool recv_async_msg(bool is_tx, const lime::StreamStats *stream_stats, void* user_data)
{
  lime::callback_info_t* cbinf = (lime::callback_info_t*)user_data;
  radio_notification_handler* notifier = (radio_notification_handler*)cbinf->notifier;
  radio_lime_tx_stream_fsm* state_fsm = (radio_lime_tx_stream_fsm*)cbinf->state_fsm;
  unsigned stream_id = cbinf->stream_id;

  // Handle event.
  radio_notification_handler::event_description event_description = {};
  event_description.stream_id                                     = stream_id;
  event_description.channel_id                                    = 0;
  event_description.source                                        = (is_tx) ? (radio_notification_handler::event_source::TRANSMIT)
                                                                            : (radio_notification_handler::event_source::RECEIVE);
  event_description.type                                          = radio_notification_handler::event_type::UNDEFINED;

  if (stream_stats->underrun) {
    event_description.type = radio_notification_handler::event_type::UNDERFLOW;
    state_fsm->async_event_late_underflow(stream_stats->timestamp);
  }

  if (stream_stats->overrun || stream_stats->loss) {
    event_description.type = radio_notification_handler::event_type::OTHER;
  }

  if (stream_stats->late) {
    event_description.type = radio_notification_handler::event_type::LATE;
    state_fsm->async_event_late_underflow(stream_stats->timestamp);
  }

  notifier->on_radio_rt_event(event_description);

  return true;
}

bool radio_lime_tx_stream::transmit_block(unsigned&                            nof_txd_samples,
                                         const baseband_gateway_buffer_reader& buffs,
                                         unsigned                              buffer_offset,
                                         lime::StreamMeta&                     md)
{
  // Extract number of samples.
  unsigned num_samples = buffs[0].size() - buffer_offset;

  // Make sure the number of channels is equal.
  report_fatal_error_if_not(buffs.get_nof_channels() == nof_channels, "Number of channels does not match.");

  // Flatten buffers.
  static_vector<void*, RADIO_MAX_NOF_CHANNELS> buffs_flat_ptr(nof_channels);
  for (unsigned channel = 0; channel != nof_channels; ++channel) {
    buffs_flat_ptr[channel] = (void**)buffs[channel].subspan(buffer_offset, num_samples).data();
  }

  // Make lime buffers.
  const lime::complex32f_t** buffs_cpp = const_cast<const lime::complex32f_t **>((lime::complex32f_t**)buffs_flat_ptr.data());

  // Safe transmission.
  return safe_execution([this, &buffs_cpp, num_samples, &md, &nof_txd_samples]() {
    nof_txd_samples = stream->StreamTx(chipIndex, buffs_cpp, num_samples, &md, std::__1::chrono::microseconds{TRANSMIT_TIMEOUT_US});
  });
}

radio_lime_tx_stream::radio_lime_tx_stream(std::shared_ptr<LimeHandle> device_,
                                          const stream_description&    description,
                                          task_executor&               async_executor_,
                                          radio_notification_handler&  notifier_) :
  stream_id(description.id),
  async_executor(async_executor_),
  notifier(notifier_),
  stream(device_->dev()),
  device(device_),
  srate_hz(description.srate_hz),
  nof_channels(description.ports.size()),
  chipIndex(0)
{
  srsran_assert(std::isnormal(srate_hz) && (srate_hz > 0.0), "Invalid sampling rate {}.", srate_hz);

  // int availableTxChannels = LMS_GetNumChannels(stream, lime::Dir::Tx);
  // if (availableTxChannels < nof_channels)
  // {
  //   printf("Device supports only %d Tx channels, required %d.\n", availableTxChannels, nof_channels);
  //   return;
  // }

  // Build stream arguments.
  lime::DataFormat wire_format;
  switch (description.otw_format) {
    case radio_configuration::over_the_wire_format::DEFAULT:
    case radio_configuration::over_the_wire_format::SC16:
      wire_format = lime::DataFormat::I16;
      break;
    case radio_configuration::over_the_wire_format::SC12:
      wire_format = lime::DataFormat::I12;
      break;
    case radio_configuration::over_the_wire_format::SC8:
    default:
      printf("Failed to create transmit stream %d. Invalid OTW format!\n", stream_id);
      return;
  }

  device->GetStreamConfig().linkFormat    = wire_format;
  device->GetStreamConfig().format        = lime::DataFormat::F32;
  device->GetStreamConfig().alignPhase    = (nof_channels>1) ? true : false;
  device->GetStreamConfig().hintSampleRate = srate_hz;
   // NOT USING THIS FOR NOW
  // device->GetStreamConfig().userData = malloc(sizeof(lime::callback_info_t));
  for (unsigned int i=0; i<nof_channels; i++)
  {
    device->GetStreamConfig().channels.emplace
    device->GetStreamConfig().txChannels[i] = i;
    device->GetDeviceConfig().channel[i].tx.enabled = true;
    device->GetDeviceConfig().channel[i].tx.sampleRate = srate_hz;
    device->GetDeviceConfig().channel[i].tx.oversample = 2;
  }

  device->GetStreamConfig().channels.at(lime::TRXDir::Tx).clear();
  int max_ant = device->dev()->GetDescriptor().rfSOC[chipIndex].channelCount;
  if (nof_channels > max_ant) {
    printf("Failed to create transmit stream %d. Invalid number of antennas (max %d, asked for %d)\n", stream_id, max_ant, nof_channels);
    return;
  }

  for (int j = 0; j < deviceChannelCount; ++j) {
    if (tx_require > 0) {
      device->GetStreamConfig().channels.at(TRXDir::Tx).push_back(j);
      --tx_require;
    }
  }



  // Parse out optional arguments.
  if (!description.args.empty())
  {
    std::vector<std::pair<std::string, std::string>> args;
    if (!device->split_args(description.args, args))
    {
      printf("Failed to create transmit stream %d. Could not parse args!\n", stream_id);
      return;
    }

    for (auto& arg : args)
    {
      if (arg.first == "nrbandwidth")
      {
        unsigned long nr_bw = std::stoul(arg.second, nullptr, 10);
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].tx.lpf = (nr_bw*1e6) / 2;
      }
      else if (arg.first == "txlpf")
      {
        unsigned long lpf = std::stoul(arg.second, nullptr, 10);
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].tx.lpf = lpf;
      }
      else if (arg.first == "txoversample")
      {
        unsigned long oversample = std::stoul(arg.second, nullptr, 10);
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].tx.oversample = oversample;
      }
      else if (arg.first == "txgfir")
      {
        unsigned long gfir = std::stoul(arg.second, nullptr, 10);
        for (unsigned int i=0; i<nof_channels; i++)
        {
          device->GetDeviceConfig().channel[i].tx.gfir.enabled = true;
          device->GetDeviceConfig().channel[i].tx.gfir.bandwidth = gfir;
        }
      }
      else if (arg.first == "txcalibrate")
      {
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].tx.calibrate = true;
      }
      else if (arg.first == "txtestSignal")
      {
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].tx.testSignal = true;
      }
      // 0=NONE, 1=BAND1, 2=BAND2
      else if (arg.first == "txpathint")
      {
        unsigned long path = std::stoul(arg.second, nullptr, 10);
        for (unsigned int i=0; i<nof_channels; i++)
          device->GetDeviceConfig().channel[i].tx.path = path;
      }
      else if (arg.first == "txpath")
      {
        bool match = false;
        auto paths = device->dev()->GetDescriptor().rfSOC[0].txPathNames;
        for(uint j=0; j<paths.size(); ++j)
        {
          if (strcasecmp(paths[j].c_str(), arg.second.c_str()) == 0)
          {
            printf("TX path: %s (%d)\n", arg.second.c_str(), j);
            for (unsigned int i=0; i<nof_channels; i++)
              device->GetDeviceConfig().channel[i].tx.path = j;
            match = true;
            break;
          }
        }

        if (!match)
          printf("TX path %s not valid!\n", arg.second.c_str());
      }
      else if (arg.first == "txMaxPacketsInBatch")
      {
        if (!device->GetStreamConfig().extraConfig)
          device->GetStreamConfig().extraConfig = new lime::SDRDevice::StreamConfig::Extras();
        
        unsigned long number = std::stoul(arg.second, nullptr, 10);
        device->GetStreamConfig().extraConfig->txMaxPacketsInBatch = number;
      }
      else if (arg.first == "txSamplesInPacket")
      {
        if (!device->GetStreamConfig().extraConfig)
          device->GetStreamConfig().extraConfig = new lime::SDRDevice::StreamConfig::Extras();
        
        unsigned long number = std::stoul(arg.second, nullptr, 10);
        device->GetStreamConfig().extraConfig->txSamplesInPacket = number;
      }
      else if (arg.first == "lmsconfig")
      {
        device->GetLMSConfPath() = arg.second;
      }
      else
        continue;

      printf("Set %s to %s\n", arg.first.c_str(), arg.second.c_str());
    }
  } 

  // Set max packet size.
  // TODO: This might need to be 256?
  max_packet_size = (wire_format == lime::SDRDevice::StreamConfig::DataFormat::I12 ? 1360 : 1020)/nof_channels;
  // max_packet_size = 256;

  // Notify FSM that it was successfully initialized.
  state_fsm.init_successful();

  // Create asynchronous task.
  device->GetStreamConfig().statusCallback = recv_async_msg;
}

void radio_lime_tx_stream::transmit(const baseband_gateway_buffer_reader& data, const baseband_gateway_transmitter_metadata& tx_md)
{
  // Protect stream transmitter.
  std::unique_lock<std::mutex> lock(stream_transmit_mutex);

  lime::SDRDevice::StreamMeta meta;

  unsigned nsamples          = data.get_nof_samples();
  unsigned txd_samples_total = 0;

  // Run states.
  // if (!state_fsm.transmit_block(meta, time_spec)) {
  //   nof_txd_samples = num_samples;
  //   return true;
  // }
  // TODO: add burst flags back in!
  // meta.flush = (md->flags & TRX_WRITE_MD_FLAG_END_OF_BURST);
  meta.flush=false;
  meta.timestamp=tx_md.ts;
  meta.useTimestamp=true;

  if (state_fsm.is_stopping()) {
    return;
  }

  // Receive stream in multiple blocks.
  while (txd_samples_total < nsamples) {
    unsigned txd_samples = 0;
    if (!transmit_block(txd_samples, data, txd_samples_total, meta)) {
      printf("Error: failed transmitting packet. %s.\n", get_error_message().c_str());
      return;
    }

    // Save timespec for first block.
    meta.timestamp += txd_samples;

    // Increment the total amount of received samples.
    txd_samples_total += txd_samples;
  }
}

void radio_lime_tx_stream::stop()
{
  lime::StreamMeta md;
  state_fsm.stop(md);

  // Send end-of-burst if it is in the middle of a burst.
  if (md.flushPartialPacket) {
    std::unique_lock<std::mutex> transmit_lock(stream_transmit_mutex);

    // Notify end of burst.
    radio_notification_handler::event_description event_description = {};
    event_description.stream_id                                     = stream_id;
    event_description.channel_id                                    = 0;
    event_description.source = radio_notification_handler::event_source::TRANSMIT;
    event_description.type   = radio_notification_handler::event_type::END_OF_BURST;
    notifier.on_radio_rt_event(event_description);

    // Flatten buffers.
    std::array<cf_t, 4>                          buffer;
    static_vector<void*, RADIO_MAX_NOF_CHANNELS> buffs_flat_ptr(nof_channels);
    for (unsigned channel = 0; channel != nof_channels; ++channel) {
      buffs_flat_ptr[channel] = (void*)buffer.data();
    }

    // Make UHD buffers.
    uhd::tx_streamer::buffs_type buffs_cpp(buffs_flat_ptr.data(), nof_channels);

    // Safe transmission. Ignore return.
    safe_execution([this, &buffs_cpp, &md]() {
      // Actual transmission. Ignore number of transmitted samples.
      stream->send(buffs_cpp, 0, md, TRANSMIT_TIMEOUT_S);
    });
  }
}

void radio_lime_tx_stream::wait_stop()
{
  // TODO
  // state_fsm.wait_stop();
}

unsigned radio_lime_tx_stream::get_buffer_size() const
{
  return max_packet_size;
}
