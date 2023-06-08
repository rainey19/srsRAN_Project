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

static bool reused_bool;
#define error_catch(thing, function)                                   \
reused_bool = function;                                                \
if (reused_bool) {                                                     \
  logger.error("Error setting {}! (returned {})", thing, reused_bool); \
}


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

static double toMHz(double value_Hz)
{
  return value_Hz * 1e-6;
}

namespace srsran {

static void LogCallback(lime::SDRDevice::LogLevel lvl, const char* msg)
{
  static srslog::basic_logger& logger = srslog::fetch_basic_logger("RF");

  logger.debug(msg);
  /*switch (lvl)
  {
    case lime::SDRDevice::LogLevel::CRITICAL:
    case lime::SDRDevice::LogLevel::ERROR:
      logger.error(msg);
      break;
    case lime::SDRDevice::LogLevel::WARNING:
      logger.warning(msg);
      break;
    case lime::SDRDevice::LogLevel::INFO:
      logger.info(msg);
      break;
    case lime::SDRDevice::LogLevel::VERBOSE:
    case lime::SDRDevice::LogLevel::DEBUG:
    default:
      logger.debug(msg);
      break;
  }*/
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
    device->dev()->Init();

    return true;
  }

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
        device->dev()->SetClockFreq(lime::SDRDevice::ClockID::CLK_REFERENCE, ref_freq, 0);
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

      // TODO: not implemented in limesuite yet
      // LMS_SetSampleRateDir(device->dev(), false, rate, oversample);
      device->GetDeviceConfig().referenceClockFreq = 0;
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

      // TODO: not implemented in limesuite yet
      // LMS_SetSampleRateDir(device->dev(), false, rate, oversample);
      device->GetDeviceConfig().channel[1].tx.sampleRate = rate;
      device->GetDeviceConfig().channel[0].tx.sampleRate = rate;
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

    logger.error("Failed to create receive stream {}. {}.", description.id, stream->get_error_message().c_str());
    return nullptr;
  }

  void execute_config(std::string dev_args)
  {
    logger.debug("Configuring radio...");
    auto t1 = std::chrono::high_resolution_clock::now();

    device->GetDeviceConfig().skipDefaults = true; // defaults are already initialized once at the startup
    device->dev()->Configure(device->GetDeviceConfig(), 0);

    /*// Temporary gain setting hack
    set_gain_hack(dev_args);

    if (device->GetDeviceConfig().channel[0].rx.gain == 69)
    {
      logger.debug("Calibration style 1");
      device->GetDeviceConfig().channel[0].rx.calibrate = true;
      device->GetDeviceConfig().channel[0].tx.calibrate = true;
      device->GetDeviceConfig().channel[1].rx.calibrate = true;
      device->GetDeviceConfig().channel[1].tx.calibrate = true;
      logger.debug("Attempting calibration");
      device->dev()->Configure(device->GetDeviceConfig(), 0);
      logger.debug("Finished");
    }
    else if (device->GetDeviceConfig().channel[0].rx.gain == 55)
    {
      logger.debug("Calibration style 2");
      device->GetDeviceConfig().channel[0].rx.calibrate = true;
      device->GetDeviceConfig().channel[0].tx.calibrate = true;
      device->GetDeviceConfig().channel[1].rx.calibrate = true;
      device->GetDeviceConfig().channel[1].tx.calibrate = true;
      logger.debug("Setting paths to loopback");
      uint8_t rx_path = device->GetDeviceConfig().channel[0].rx.path;
      uint8_t tx_path = device->GetDeviceConfig().channel[0].tx.path;
      logger.debug("Saved old paths (rx={}, tx={})", rx_path, tx_path);
      device->GetDeviceConfig().channel[0].rx.path = 0;
      device->GetDeviceConfig().channel[0].tx.path = 0;
      device->GetDeviceConfig().channel[1].rx.path = 0;
      device->GetDeviceConfig().channel[1].tx.path = 0;
      logger.debug("Attempting calibration");
      device->dev()->Configure(device->GetDeviceConfig(), 0);
      logger.debug("Finished");
      device->GetDeviceConfig().channel[0].rx.path = rx_path;
      device->GetDeviceConfig().channel[0].tx.path = tx_path;
      device->GetDeviceConfig().channel[1].rx.path = rx_path;
      device->GetDeviceConfig().channel[1].tx.path = tx_path;
      logger.debug("Returned paths to previous (rx={}, tx={})", rx_path, tx_path);
    }*/

    auto t2 = std::chrono::high_resolution_clock::now();
    logger.debug("Radio configured in {}ms.", std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count());
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
    logger.debug("Setting channel {} Tx frequency to {} MHz.", ch, toMHz(config.center_frequency_hz));

    return safe_execution([this, ch, &config]() {
      lime::Range range(0, 3.7e9, 1);

      // TODO: not implemented in limesuite yet
      // LMS_GetSampleRateRange(device->dev(), false, &range);

      if (!radio_lime_device_validate_freq_range(range, config.center_frequency_hz)) {
        on_error("Tx RF frequency {} MHz is out-of-range. Range is {} - {}.",
                 toMHz(config.center_frequency_hz),
                 toMHz(range.min),
                 toMHz(range.max));
        return;
      }

      lime::SDRDevice::SDRConfig& conf = device->GetDeviceConfig();
      conf.channel[ch].tx.centerFrequency = config.center_frequency_hz;
    });
  }

  bool set_rx_freq(uint32_t ch, const radio_configuration::lo_frequency& config)
  {
    logger.debug("Setting channel {} Rx frequency to {} MHz.", ch, toMHz(config.center_frequency_hz));

    return safe_execution([this, ch, &config]() {
      lime::Range range(0, 3.7e9, 1);

      // TODO: not implemented in limesuite yet
      // LMS_GetSampleRateRange(device->dev(), false, &range);

      if (!radio_lime_device_validate_freq_range(range, config.center_frequency_hz)) {
        on_error("Rx RF frequency {} MHz is out-of-range. Range is {} - {}.",
                 toMHz(config.center_frequency_hz),
                 toMHz(range.min),
                 toMHz(range.max));
        return;
      }

      lime::SDRDevice::SDRConfig& conf = device->GetDeviceConfig();
      conf.channel[ch].rx.centerFrequency = config.center_frequency_hz;
    });
  }

private:
  std::shared_ptr<LimeHandle> device = nullptr;
  srslog::basic_logger&       logger;

  void set_gain_hack(std::string dev_args)
  {
    // Parse out optional arguments.
    if (!dev_args.empty())
    {
      std::vector<std::pair<std::string, std::string>> args;
      device->split_args(dev_args, args);

      // 0-min, 15-max
      int lna = -1;
      // 0-min, 31-max
      int pga = -1;
      // 0-min, 64-max
      int iamp = -1;
      // 0-max, 31-min
      int txpad = -1;

      logger.debug("Configuring gains...");
      for (auto& arg : args)
      {
        if (arg.first == "lna")
          lna = std::stoul(arg.second, nullptr, 10);
        else if (arg.first == "pga")
          pga = std::stoul(arg.second, nullptr, 10);
        else if (arg.first == "iamp")
          iamp = std::stoul(arg.second, nullptr, 10);
        else if (arg.first == "txpad")
          txpad = std::stoul(arg.second, nullptr, 10);
        else
          continue;
        logger.debug("Setting {} to {}", arg.first, arg.second);
      }

      // TODO replace 0 with chipIndex
      lime::LMS7002M* chip = static_cast<lime::LMS7002M*>(device->dev()->GetInternalChip(0));
      for(int mac=1; mac<=2; ++mac)
      {
        error_catch("LMS7_MAC",
                    chip->Modify_SPI_Reg_bits(LMS7_MAC, mac));
        if (lna != -1)
        {
          error_catch("LMS7_MAC",
                    chip->Modify_SPI_Reg_bits(LMS7_G_LNA_RFE, lna));
        }
        if (pga != -1)
        {
          error_catch("LMS7_MAC",
                    chip->Modify_SPI_Reg_bits(LMS7_G_PGA_RBB, pga));
        }
        if (iamp != -1)
        {
          error_catch("LMS7_MAC",
                    chip->Modify_SPI_Reg_bits(LMS7_CG_IAMP_TBB, iamp));
        }
        if (txpad != -1)
        {
          error_catch("LMS7_MAC",
                    chip->Modify_SPI_Reg_bits(LMS7_LOSS_MAIN_TXPAD_TRF, txpad));
        }
      }
      chip->Modify_SPI_Reg_bits(LMS7_MAC, 1);
    }
  }
};

} // namespace srsran
