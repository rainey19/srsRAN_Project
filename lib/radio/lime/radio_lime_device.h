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
#include "radio_lime_rx_stream.h"
#include "radio_lime_tx_stream.h"
#include "srsran/radio/radio_session.h"
#include "srsran/srslog/srslog.h"

#define clip(val,range) std::max(range.min, std::min(range.max, val))

/// \brief Determines whether a frequency is valid within a range.
///
/// A frequency is considered valid within a range if the range clips the frequency value within 1 Hz error.
static bool radio_lime_device_validate_freq_range(const lime::Range<double>& range, double freq)
{
  double clipped_freq = clip(freq, range);
  return std::abs(clipped_freq - freq) < 1.0;
}

/// \brief Determines whether a gain is valid within a range.
///
/// A gain is considered valid within a range if the range clips the frequency value within 0.01 error.
static bool radio_lime_device_validate_gain_range(const lime::Range<double>& range, double gain)
{
  int64_t clipped_gain = static_cast<uint64_t>(std::round(clip(gain, range) * 100));
  int64_t uint_gain    = static_cast<uint64_t>(gain * 100);

  return (clipped_gain == uint_gain);
}

static double toMHz(double value_Hz)
{
  return value_Hz * 1e-6;
}

namespace srsran {

// TODO
static void LogCallback(lime::LogLevel lvl, const std::string msg)
{
  static srslog::basic_logger& logger = srslog::fetch_basic_logger("RF");

  switch (lvl)
  {
    case lime::LogLevel::Critical:
    case lime::LogLevel::Error:
      logger.error(msg);
      break;
    case lime::LogLevel::Warning:
      logger.warning(msg);
      break;
    case lime::LogLevel::Info:
      logger.info(msg);
      break;
    case lime::LogLevel::Verbose:
    case lime::LogLevel::Debug:
    default:
      logger.debug(msg);
      break;
  }
}

class radio_lime_device : public lime_exception_handler
{
public:
  radio_lime_device() : logger(srslog::fetch_basic_logger("RF")) {}

  bool is_valid() const { return device != nullptr; }

  bool lime_make(const std::string& device_args)
  {
    // Destroy any previous USRP instance
    device = nullptr;

    // Enumerate devices
    std::vector<lime::DeviceHandle> devHandles = lime::DeviceRegistry::enumerate();
    if (devHandles.size() == 0)
    {
      logger.error("No LMS7002M boards found!\n");
      fprintf(stderr, "No LMS7002M boards found!\n");
      return false;
    }
    else
    {
      logger.debug("Available LMS7002M devices:");
      for (const auto &dev : devHandles) {
        logger.debug("\t\"{}\"", dev.Serialize().c_str());
      }
    }

    // Connect and initialize
    lime::DeviceHandle first_device_ = devHandles.front();
    logger.debug("Selected: {}", first_device_.Serialize().c_str());
    device = LimeHandle::get(first_device_);
    if (device == nullptr)
    {
      logger.error("Port[0] failed to connect: {}", first_device_.Serialize().c_str());
      return false;
    }

    // Initialize devices to default settings
    device->dev()->SetMessageLogCallback(LogCallback);
    device->dev()->Init();

    return true;
  }

  // TODO
  bool get_mboard_sensor_names(std::vector<std::string>& sensors)
  {
    sensors.push_back("temp");
    return true;
  }

  // TODO
  bool get_rx_sensor_names(std::vector<std::string>& sensors)
  {
    return true;
  }

  bool get_sensor(const std::string& sensor_name, double& sensor_value)
  {
    if (sensor_name == "temp")
    {
      // TODO replace 0 with chipIndex
      lime::LMS7002M* chip = static_cast<lime::LMS7002M*>(device->dev()->GetInternalChip(0));
      sensor_value = chip->GetTemperature();
      return true;
    }

    return false;
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

  // TODO
  bool get_time_now(uint64_t& timespec)
  {
    timespec = 0;
    return true;
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
        device->dev()->SetClockFreq((uint8_t)lime::LMS7002M::ClockID::CLK_REFERENCE, ref_freq, 0);
      }
    });
  }

  bool set_rx_rate(double rate)
  {
    logger.debug("Setting Rx Rate to {} MSPS.", toMHz(rate));

    return safe_execution([this, rate]() {
      lime::Range range(0, 120e6, 1);

      // TODO: not implemented in limesuite yet
      // LMS_GetSampleRateRange(device->dev(), false, &range);
      if (!radio_lime_device_validate_freq_range(range, rate)) {
        on_error("Rx Rate {} MHz is invalid. The nearest valid value is {}.", toMHz(rate), toMHz(clip(rate, range)));
        return;
      }

      // device->GetDeviceConfig().referenceClockFreq = 0;
      device->GetDeviceConfig().channel[0].rx.sampleRate = rate;
      device->GetDeviceConfig().channel[1].rx.sampleRate = rate;
    });
  }

  bool set_tx_rate(double rate)
  {
    logger.debug("Setting Tx Rate to {} MSPS.", toMHz(rate));

    return safe_execution([this, rate]() {
      lime::Range range(0, 120e6, 1);

      // TODO: not implemented in limesuite yet
      // LMS_GetSampleRateRange(device->dev(), true, &range);

      if (!radio_lime_device_validate_freq_range(range, rate)) {
        on_error("Tx Rate {} MHz is invalid. The nearest valid value is {}.", toMHz(rate), toMHz(clip(rate, range)));
        return;
      }

      device->GetDeviceConfig().channel[0].tx.sampleRate = rate;
      device->GetDeviceConfig().channel[1].tx.sampleRate = rate;
    });
  }

  bool set_command_time(const uint64_t timespec)
  {
    // TODO: ?
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

    logger.error("Failed to create receive stream {}. {}.", description.id, stream->get_error_message().c_str());
    return nullptr;
  }

  void execute_config(std::string dev_args)
  {
    logger.debug("Configuring radio...");
    auto t1 = std::chrono::high_resolution_clock::now();

    // device->GetDeviceConfig().skipDefaults = true; // defaults are already initialized once at the startup
    device->dev()->Configure(device->GetDeviceConfig(), 0);
    
    if (device->GetLMSConfPath() != "")
    {
      lime::LMS7002M* chip = static_cast<lime::LMS7002M*>(device->dev()->GetInternalChip(0));
      chip->LoadConfig(device->GetLMSConfPath().c_str());
    }

    lime::LMS7002M* chip = static_cast<lime::LMS7002M*>(device->dev()->GetInternalChip(0));
    logger.info("Actual tx freq: {:.3f} MHz", chip->GetFrequencySX(lime::TRXDir::Tx)/1e6);
    logger.info("Actual rx freq: {:.3f} MHz", chip->GetFrequencySX(lime::TRXDir::Rx)/1e6);
    logger.info("Temp?: {:.1f}", chip->GetTemperature());
    logger.info("TX rate: {:.3f} Msps", chip->GetSampleRate(lime::TRXDir::Tx, lime::LMS7002M::Channel::ChA)/1e6);
    logger.info("RX rate: {:.3f} Msps", chip->GetSampleRate(lime::TRXDir::Rx, lime::LMS7002M::Channel::ChA)/1e6);

    auto t2 = std::chrono::high_resolution_clock::now();
    logger.debug("Radio configured in {}ms.", std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count());
  }

  bool set_tx_gain(unsigned ch, double gain)
  {
    logger.debug("Setting channel {} Tx gain to {:.2f} dB.", ch, gain);

    return safe_execution([this, ch, gain]() {
      // WITH NEW_GAIN_BEHAVIOUR MAX GAIN VALUE IS 62
      lime::Range range(0, 74, 0.1);

      if (!radio_lime_device_validate_gain_range(range, gain)) {
        on_error("Tx gain (i.e., {} dB) is out-of-range. Range is [{}, {}] dB in steps of {} dB.",
                 gain,
                 range.min,
                 range.max,
                 range.step);
        return;
      }

      // if (device->dev()->SetGain(0, lime::TRXDir::Tx, ch, lime::eGainTypes::UNKNOWN, gain)) {
      //   on_error("Error setting TX gain!");
      //   return;
      // }

      lime::SDRDevice::SDRConfig& conf = device->GetDeviceConfig();
      conf.channel[ch].tx.gain.emplace(std::pair<lime::eGainTypes, double>(lime::eGainTypes::UNKNOWN, gain));
    });
  }

  bool set_rx_gain(size_t ch, double gain)
  {
    logger.debug("Setting channel {} Rx gain to {:.2f} dB.", ch, gain);

    return safe_execution([this, ch, gain]() {
      lime::Range range(0, 74, 0.1);

      if (!radio_lime_device_validate_gain_range(range, gain)) {
        on_error("Rx gain (i.e., {} dB) is out-of-range. Range is [{}, {}] dB in steps of {} dB.",
                 gain,
                 range.min,
                 range.max,
                 range.step);
        return;
      }

      // if (device->dev()->SetGain(0, lime::TRXDir::Rx, ch, lime::eGainTypes::UNKNOWN, gain)) {
      //   on_error("Error setting TX gain!");
      //   return;
      // }

      lime::SDRDevice::SDRConfig& conf = device->GetDeviceConfig();
      conf.channel[ch].rx.gain.emplace(std::pair<lime::eGainTypes, double>(lime::eGainTypes::UNKNOWN, gain));
    });
  }

  bool set_tx_freq(uint32_t ch, const radio_configuration::lo_frequency& config)
  {
    logger.debug("Setting channel {} Tx frequency to {} MHz.", ch, toMHz(config.center_frequency_hz));

    return safe_execution([this, ch, &config]() {
      lime::Range range(0, 3.7e9, 1);

      if (!radio_lime_device_validate_freq_range(range, config.center_frequency_hz)) {
        on_error("Tx RF frequency {} MHz is out-of-range. Range is {} - {}.",
                 toMHz(config.center_frequency_hz),
                 toMHz(range.min),
                 toMHz(range.max));
        return;
      }

      device->GetDeviceConfig().channel[ch].tx.centerFrequency = config.center_frequency_hz;
    });
  }

  bool set_rx_freq(uint32_t ch, const radio_configuration::lo_frequency& config)
  {
    logger.debug("Setting channel {} Rx frequency to {} MHz.", ch, toMHz(config.center_frequency_hz));

    return safe_execution([this, ch, &config]() {
      lime::Range range(0, 3.7e9, 1);

      if (!radio_lime_device_validate_freq_range(range, config.center_frequency_hz)) {
        on_error("Rx RF frequency {} MHz is out-of-range. Range is {} - {}.",
                 toMHz(config.center_frequency_hz),
                 toMHz(range.min),
                 toMHz(range.max));
        return;
      }

      device->GetDeviceConfig().channel[ch].rx.centerFrequency = config.center_frequency_hz;
    });
  }

private:
  std::shared_ptr<LimeHandle> device = nullptr;
  srslog::basic_logger&       logger;
};

} // namespace srsran
