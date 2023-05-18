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

#include "radio_lime_exception_handler.h"
#include "radio_lime_rx_stream.h"
#include "radio_lime_tx_stream.h"
#include "srsran/radio/radio_session.h"

#define clip(val,range) std::max(range.min, std::min(range.max, val))

/// \brief Determines whether a frequency is valid within a range.
///
/// A frequency is considered valid within a range if the range clips the frequency value within 1 Hz error.
static bool radio_lime_device_validate_freq_range(const lime::Range& range, double freq)
{
  double clipped_freq = clip(freq, range);
  return std::abs(clipped_freq - freq) < 1.0;
}

/// \brief Determines whether a gain is valid within a range.
///
/// A gain is considered valid within a range if the range clips the frequency value within 0.01 error.
static bool radio_lime_device_validate_gain_range(const lime::Range& range, double gain)
{
  int64_t clipped_gain = static_cast<uint64_t>(std::round(clip(gain, range) * 100));
  int64_t uint_gain    = static_cast<uint64_t>(gain * 100);

  return (clipped_gain == uint_gain);
}

static double to_MHz(double value_Hz)
{
  return value_Hz * 1e-6;
}

namespace srsran {

static void LogCallback(lime::SDRDevice::LogLevel lvl, const char* msg)
{
  printf("LIMELOG: %s", msg);
  return;

  // static srslog::basic_logger& logger = srslog::fetch_basic_logger("RF");

  // switch (lvl)
  // {
  //   case lime::SDRDevice::LogLevel::CRITICAL:
  //   case lime::SDRDevice::LogLevel::ERROR:
  //     logger.error(msg);
  //     break;
  //   case lime::SDRDevice::LogLevel::WARNING:
  //     logger.warning(msg);
  //     break;
  //   case lime::SDRDevice::LogLevel::INFO:
  //     logger.info(msg);
  //     break;
  //   case lime::SDRDevice::LogLevel::VERBOSE:
  //   case lime::SDRDevice::LogLevel::DEBUG:
  //   default:
  //     logger.debug(msg);
  //     break;
  // }
}

class radio_lime_device : public lime_exception_handler
{
public:
  radio_lime_device() : logger(srslog::fetch_basic_logger("RF")) {}

  bool is_valid() const { return device != nullptr; }

  bool lime_make(const std::string& device_address)
  {
    // Destroy any previous USRP instance
    device = nullptr;

    // Enumerate devices
    std::vector<lime::DeviceHandle> devHandles = lime::DeviceRegistry::enumerate();
    if (devHandles.size() == 0)
    {
      fprintf(stderr, "No LMS7002M boards found!\n");
      return false;
    }
    else
    {
      logger.debug("Available LMS7002M devices:");
      for (const auto &dev : devHandles) {
        logger.debug("\t\"{}\"", dev.serialize().c_str());
      }
    }

    // Connect and initialize
    lime::DeviceHandle first_device_ = devHandles.front();
    logger.debug("Selected: {}", first_device_.serialize().c_str());
    device = LimeHandle::get(first_device_);
    if (device == nullptr)
    {
      logger.error("Port[0] failed to connect: {}", first_device_.serialize().c_str());
      return false;
    }

    // Initialize devices to default settings
    device->dev()->SetMessageLogCallback(LogCallback);
    printf("\nSETUP DONE, CALLING INIT\n");
    device->dev()->Init();
    printf("\nINIT FINALLY RETURNED\n");

    // calibrations setup
    // device->dev()->EnableCache(false);

    return true;
  }

  // TODO
  bool get_mboard_sensor_names(std::vector<std::string>& sensors)
  {
    return true;
  }

  // TODO
  bool get_rx_sensor_names(std::vector<std::string>& sensors)
  {
    return true;
  }

  // TODO
  bool get_sensor(const std::string& sensor_name, double& sensor_value)
  {
    return true;
  }

  // TODO
  bool get_sensor(const std::string& sensor_name, bool& sensor_value)
  {
    return true;
  }

  // TODO
  bool get_rx_sensor(const std::string& sensor_name, bool& sensor_value)
  {
    return true;
  }

  // TODO
  bool set_time_unknown_pps(const uint64_t timespec)
  {
    return true;
  }

  bool get_time_now(uint64_t& timespec)
  {
    return safe_execution([this, &timespec]()
    {
      lime::SDRDevice::StreamStats rx, tx;
      device->dev()->StreamStatus(0, &rx, &tx);
      timespec = rx.timestamp;
    });
  }

  // TODO
  bool set_sync_source(const radio_configuration::clock_sources& config)
  {
    // Convert clock source to string.
    std::string clock_src;
    switch (config.clock) {
      case radio_configuration::clock_sources::source::DEFAULT:
      case radio_configuration::clock_sources::source::INTERNAL:
        clock_src = "internal";
        break;
      case radio_configuration::clock_sources::source::EXTERNAL:
        clock_src = "external";
        break;
      case radio_configuration::clock_sources::source::GPSDO:
        clock_src = "gpsdo";
        break;
    }

    // Convert sync source to string.
    std::string sync_src;
    switch (config.sync) {
      case radio_configuration::clock_sources::source::DEFAULT:
      case radio_configuration::clock_sources::source::INTERNAL:
        sync_src = "internal";
        break;
      case radio_configuration::clock_sources::source::EXTERNAL:
      case radio_configuration::clock_sources::source::GPSDO:
        sync_src = "external";
        break;
    }

    logger.debug("Setting PPS source to '{}' and clock source to '{}'.", sync_src, clock_src);
    return safe_execution([this, &sync_src, &clock_src]()
    {
      if (clock_src == "external")
      {
        bool is_xtrx = true;
        double ref_freq = 10e6;
        if (is_xtrx)
        {
          ref_freq = 31.22e6;
        }
        device->dev()->SetClockFreq(lime::SDRDevice::ClockID::CLK_REFERENCE, ref_freq, 0);
      }
    });
  }

  bool set_rx_rate(double rate)
  {
    logger.debug("Setting Rx Rate to {} MHz.", to_MHz(rate));

    return safe_execution([this, rate]() {
      lime::Range range(0, 120e6, 1);

      // TODO: not implemented in limesuite yet
      // LMS_GetSampleRateRange(device->dev(), false, &range);

      if (!radio_lime_device_validate_freq_range(range, rate)) {
        on_error("Rx Rate {} MHz is invalid. The nearest valid value is {}.", to_MHz(rate), to_MHz(clip(rate, range)));
        return;
      }

      // TODO: not implemented in limesuite yet
      // size_t oversample = 2;
      // LMS_SetSampleRateDir(device->dev(), false, rate, oversample);
      lime::SDRDevice::SDRConfig conf;
      conf.skipDefaults = true;
      conf.referenceClockFreq = 0;
      conf.channel[0].rx.sampleRate = rate;
      conf.channel[1].rx.sampleRate = rate;
    });
  }

  bool set_tx_rate(double rate)
  {
    logger.debug("Setting Tx Rate to {} MHz.", to_MHz(rate));

    return safe_execution([this, rate]() {
      lime::Range range(0, 120e6, 1);

      // TODO: not implemented in limesuite yet
      // LMS_GetSampleRateRange(device->dev(), true, &range);

      if (!radio_lime_device_validate_freq_range(range, rate)) {
        on_error("Tx Rate {} MHz is invalid. The nearest valid value is {}.", to_MHz(rate), to_MHz(clip(rate, range)));
        return;
      }

      // TODO: not implemented in limesuite yet
      // size_t oversample = 2;
      // LMS_SetSampleRateDir(device->dev(), false, rate, oversample);
      lime::SDRDevice::SDRConfig& conf = device->GetDeviceConfig();
      conf.channel[0].tx.sampleRate = rate;
      conf.channel[1].tx.sampleRate = rate;
    });
  }

  bool set_command_time(const uint64_t timespec)
  {
    return true;
  }

  std::unique_ptr<radio_lime_tx_stream> create_tx_stream(task_executor&                                 async_executor,
                                                        radio_notification_handler&                     notifier,
                                                        const radio_lime_tx_stream::stream_description& description)
  {
    std::unique_ptr<radio_lime_tx_stream> stream =
        std::make_unique<radio_lime_tx_stream>(device, description, async_executor, notifier);

    if (stream->is_successful()) {
      return stream;
    }

    return nullptr;
  }

  std::unique_ptr<radio_lime_rx_stream> create_rx_stream(radio_notification_handler&                    notifier,
                                                        const radio_lime_rx_stream::stream_description& description)
  {
    std::unique_ptr<radio_lime_rx_stream> stream = std::make_unique<radio_lime_rx_stream>(device, description, notifier);

    if (stream->is_successful()) {
      return stream;
    }

    printf("Error: failed to create receive stream %d. %s.", description.id, stream->get_error_message().c_str());
    return nullptr;
  }

  bool set_tx_gain(unsigned ch, double gain)
  {
    logger.debug("Setting channel {} Tx gain to {:.2f} dB.", ch, gain);

    return safe_execution([this, ch, gain]() {
      lime::Range range(0, 70, 0.1);

      if (!radio_lime_device_validate_gain_range(range, gain)) {
        on_error("Tx gain (i.e., {} dB) is out-of-range. Range is [{}, {}] dB in steps of {} dB.",
                 gain,
                 range.min,
                 range.max,
                 range.step);
        return;
      }

      // TODO: not implemented in limesuite yet!
      // LMS_SetGaindB(device->dev(), true, ch, gain);

      lime::SDRDevice::SDRConfig& conf = device->GetDeviceConfig();
      conf.channel[ch].tx.gain = gain;
    });
  }

  bool set_rx_gain(size_t ch, double gain)
  {
    logger.debug("Setting channel {} Rx gain to {:.2f} dB.", ch, gain);

    return safe_execution([this, ch, gain]() {
      lime::Range range(0, 70, 0.1);

      if (!radio_lime_device_validate_gain_range(range, gain)) {
        on_error("Rx gain (i.e., {} dB) is out-of-range. Range is [{}, {}] dB in steps of {} dB.",
                 gain,
                 range.min,
                 range.max,
                 range.step);
        return;
      }

      // TODO: not implemented in limesuite yet!
      // LMS_SetGaindB(device->dev(), false, ch, gain);

      lime::SDRDevice::SDRConfig& conf = device->GetDeviceConfig();
      conf.channel[ch].rx.gain = gain;
    });
  }

  bool set_tx_freq(uint32_t ch, const radio_configuration::lo_frequency& config)
  {
    logger.debug("Setting channel {} Tx frequency to {} MHz.", ch, to_MHz(config.center_frequency_hz));

    return safe_execution([this, ch, &config]() {
      lime::Range range(0, 3.7e9, 1);

      // TODO: not implemented in limesuite yet
      // LMS_GetSampleRateRange(device->dev(), false, &range);

      if (!radio_lime_device_validate_freq_range(range, config.center_frequency_hz)) {
        on_error("Tx RF frequency {} MHz is out-of-range. Range is {} - {}.",
                 to_MHz(config.center_frequency_hz),
                 to_MHz(range.min),
                 to_MHz(range.max));
        return;
      }

      lime::SDRDevice::SDRConfig& conf = device->GetDeviceConfig();
      conf.channel[ch].tx.centerFrequency = config.center_frequency_hz;
    });
  }

  bool set_rx_freq(uint32_t ch, const radio_configuration::lo_frequency& config)
  {
    logger.debug("Setting channel {} Rx frequency to {} MHz.", ch, to_MHz(config.center_frequency_hz));

    return safe_execution([this, ch, &config]() {
      lime::Range range(0, 3.7e9, 1);

      // TODO: not implemented in limesuite yet
      // LMS_GetSampleRateRange(device->dev(), false, &range);

      if (!radio_lime_device_validate_freq_range(range, config.center_frequency_hz)) {
        on_error("Rx RF frequency {} MHz is out-of-range. Range is {} - {}.",
                 to_MHz(config.center_frequency_hz),
                 to_MHz(range.min),
                 to_MHz(range.max));
        return;
      }

      lime::SDRDevice::SDRConfig& conf = device->GetDeviceConfig();
      conf.channel[ch].rx.centerFrequency = config.center_frequency_hz;
    });
  }

private:
  std::shared_ptr<LimeHandle> device = nullptr;
  srslog::basic_logger&       logger;
};

} // namespace srsran
