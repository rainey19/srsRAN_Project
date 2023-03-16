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
#include "radio_xtrx_rx_stream.h"
#include "radio_xtrx_tx_stream.h"
#include "srsran/radio/radio_session.h"
#include "xtrxll_api.h"

/// \brief Determines whether a frequency is valid within a range.
///
/// A frequency is considered valid within a range if the range clips the frequency value within 1 Hz error.
static bool radio_xtrx_device_validate_freq_range(double freq)
{
  double min = 30e6;
  double max = 3.8e9;

  return (freq <= max) && (freq >= min);
}

/// \brief Determines whether a gain is valid within a range.
///
/// A gain is considered valid within a range if the range clips the frequency value within 0.01 error.
static bool radio_xtrx_device_validate_gain_range(bool TX, double gain)
{
  if (TX)
    return (gain >= -52) && (gain <= 0);
  else // RX
    return (gain >= 0) && (gain <= 30);
}

namespace srsran {

class radio_xtrx_device : public xtrx_exception_handler
{
public:
  radio_xtrx_device() : logger(srslog::fetch_basic_logger("RF")) {}

  bool is_valid() const { return xtrx_handle != nullptr; }

  bool __attribute__((optimize("O0"))) xtrx_dev_make(const std::string& device_address)
  {
    std::string dev_addr;
    uint64_t val;
    int res;
    
    // Destroy any previous xtrx_handle instance
    xtrx_handle = nullptr;

    if (device_address.length()) {
      dev_addr = device_address;
    }
    else {
      dev_addr = "/dev/xtrx0";
    }

    fmt::print("Making xtrx_handle object with args '{}'\n", dev_addr);

    xtrx_handle = XTRXHandle::get(dev_addr);

    res = xtrx_val_get(xtrx_handle->dev(), XTRX_TRX, XTRX_CH_ALL, XTRX_BOARD_TEMP, &val);
    printf("XTRX current board temp %.2f C\n", ((double)val / 256.0));

    return res;
  }

  bool get_mboard_sensor_names(std::vector<std::string>& sensors)
  {
    sensors.push_back("ref_locked");
    sensors.push_back("gps_locked"); // TODO: is this handled by FPGA or does it need a thread? See: test_xtrxll.c
    sensors.push_back("board_temp");
    sensors.push_back("tx_time");
    // sensors.push_back("lms7_temp");

    return 0;
  }

  bool get_rx_sensor_names(std::vector<std::string>& sensors)
  {
    // sensors.push_back("lo_locked");
    return 0;
  }

  bool get_sensor(const std::string& sensor_name, double& sensor_value)
  {
    int res;
    uint64_t val;

    // XTRX_IC_TEMP // no high level implementation!
    // XTRX_LMS7_TEMP // no high level implementation!
    // XTRX_REF_REFCLK // seems to be a set operation???

    if (sensor_name == "board_temp") {
      res = xtrx_val_get(xtrx_handle->dev(), XTRX_TRX, XTRX_CH_ALL, XTRX_BOARD_TEMP, &val);
      sensor_value = (double)val / 256.0;
    }
    else if (sensor_name == "tx_time") {
      res = xtrx_val_get(xtrx_handle->dev(), XTRX_TRX, XTRX_CH_ALL, XTRX_TX_TIME, &val);
      sensor_value = val;
    }
    else {
      return 1;
    }

    return res;
  }

  bool get_sensor(const std::string& sensor_name, bool& sensor_value)
  {
    int res;
    uint64_t val;

    if (sensor_name == "ref_locked") {
      res = xtrx_val_get(xtrx_handle->dev(), XTRX_TRX, XTRX_CH_ALL, XTRX_OSC_LATCH_1PPS, &val);
      sensor_value = (val) ? true : false;
    }
    else {
      return 1;
    }

    return res;
  }

  bool get_rx_sensor(const std::string& sensor_name, bool& sensor_value)
  {
    return 1;
  }

// TODO (or not?)
  bool set_time_unknown_pps(const baseband_gateway_timestamp& timespec)
  {
    return safe_execution([this, &timespec]() {
      // xtrx_handle->set_time_unknown_pps(timespec);
    });
  }

// TODO (or not?)
  bool set_automatic_master_clock_rate(double srate_Hz)
  {
    return safe_execution([this, &srate_Hz]() {
      // Get range of valid master clock rates.
      // uhd::meta_range_t range = xtrx_handle->get_master_clock_rate_range();

      // Select the nearest valid master clock rate.
      // double mcr_Hz = range.clip(srate_Hz);

      // xtrx_handle->set_master_clock_rate(mcr_Hz);
    });
  }

  bool __attribute__((optimize("O0"))) get_time_now(baseband_gateway_timestamp& timespec)
  {
    uint32_t reg_val[2];
    unsigned int llreg;
    struct xtrxll_dev* lldev = {}; // TODO make nullptr to avoid mem leak?

    // TODO: maybe I should be tracking gtime in an object like the uhd timespec?
		// llreg = XTRXLL_OSC_LATCHED;     // OSCLATCH
		// llreg = XTRXLL_RX_TIME;         // RXTIME
		// llreg = XTRXLL_TX_TIME;         // TXTIME
		llreg = XTRXLL_GTIME_SECFRAC;   // GTIME
		// llreg = XTRXLL_GTIME_OFF;       // GT_OFF

    if (xtrx_handle->get_ll(&lldev)) {
      return 1;
    }
    if (xtrxll_get_sensor(lldev, llreg, (int*)&reg_val[0])) {
      return 1;
    }

		if (llreg == XTRXLL_GTIME_SECFRAC) {
      // sec = reg_val[0];
      // ns in ticks = reg_val[1];
			timespec = (uint64_t)reg_val[1];
		} else {
			timespec = reg_val[0];
		}

		return 0;
  }

  bool __attribute__((optimize("O0"))) set_sync_source(const radio_configuration::clock_sources& config)
  {
    std::string clk;
    std::string pps;

    // Convert clock source to string.
    xtrx_clock_source_t timing_src = {};
    switch (config.clock) {
      case radio_configuration::clock_sources::source::DEFAULT:
      case radio_configuration::clock_sources::source::INTERNAL:
        clk = "internal";
      case radio_configuration::clock_sources::source::GPSDO:
        clk = "gpsdo";
        timing_src = XTRX_CLKSRC_INT;
        break;
      case radio_configuration::clock_sources::source::EXTERNAL:
        clk = "external";
        timing_src = XTRX_CLKSRC_EXT;
        break;
    }

    // Convert sync source to string.
    std::string sync_src;
    switch (config.sync) {
      case radio_configuration::clock_sources::source::EXTERNAL:
        pps = "external";
        timing_src = XTRX_CLKSRC_EXT_W1PPS_SYNC;
        break;
      case radio_configuration::clock_sources::source::DEFAULT:
      case radio_configuration::clock_sources::source::INTERNAL:
        pps = "internal";
      case radio_configuration::clock_sources::source::GPSDO:
        pps = "gpsdo";
        break;
    }

    logger.debug("Setting PPS source to '{}' and clock source to '{}'.", pps, clk);

    auto devvv = xtrx_handle->dev();
    int retval = xtrx_set_ref_clk(devvv, 0, timing_src);
    return retval;
  }

  bool set_rx_rate(double rate)
  {
    int ret;
    logger.debug("Setting Rx Rate to {} MHz.", rate*1e-6);

    return safe_execution([this, rate, &ret]() {
      double cgen_freq = 0;
      double cgen_actual, _actual_rx_rate, _actual_tx_rate;
      double _rate = XTRXHandle::clip_range(XTRX_RX, rate);

      if (rate != _rate) {
        on_error("Rx Rate {:.2f} MHz is invalid. The nearest valid value is {:.2f}.",
                 rate*1e-6,
                 _rate*1e-6);
        return;
      }

      ret = xtrx_set_samplerate(xtrx_handle->dev(), cgen_freq, _rate, _rate,
								  0, //XTRX_SAMPLERATE_FORCE_UPDATE,
                  // (soft_filter) ? XTRX_SAMPLERATE_FORCE_TX_INTR | XTRX_SAMPLERATE_FORCE_RX_DECIM : 0,
								  &cgen_actual, &_actual_rx_rate, &_actual_tx_rate);

      fprintf(stderr, "trx_xtrx_get_sample_rate set=%.0f act=%.0f master=%.3f MHz\n",
			        _rate, _actual_rx_rate, cgen_actual / 1e6);
    });

    return ret;
  }

  bool set_tx_rate(double rate)
  {
    int ret;
    logger.debug("Setting Tx Rate to {} MHz.", rate*1e-6);

    return safe_execution([this, rate, &ret]() {
      double cgen_freq = 0;
      double cgen_actual, _actual_rx_rate, _actual_tx_rate;
      double _rate = XTRXHandle::clip_range(XTRX_TX, rate);

      if (rate != _rate) {
        on_error("Tx Rate {:.2f} MHz is invalid. The nearest valid value is {:.2f}.",
                 rate*1e-6,
                 _rate*1e-6);
        return;
      }

      ret = xtrx_set_samplerate(xtrx_handle->dev(), cgen_freq, _rate, _rate,
								  0, //XTRX_SAMPLERATE_FORCE_UPDATE,
                  // (soft_filter) ? XTRX_SAMPLERATE_FORCE_TX_INTR | XTRX_SAMPLERATE_FORCE_TX_DECIM : 0,
								  &cgen_actual, &_actual_rx_rate, &_actual_tx_rate);

      fprintf(stderr, "trx_xtrx_get_sample_rate set=%.0f act=%.0f master=%.3f MHz\n",
			        _rate, _actual_tx_rate, cgen_actual / 1e6);
    });

    return ret;
  }

// TODO (or not?)
  bool set_command_time(const baseband_gateway_timestamp& timespec)
  {
    // return safe_execution([this, &timespec]() { xtrx_handle->set_command_time(timespec); });
    return 1;
  }

  std::unique_ptr<radio_xtrx_tx_stream> create_tx_stream(task_executor&                                 async_executor,
                                                        radio_notification_handler&                     notifier,
                                                        const radio_xtrx_tx_stream::stream_description& description)
  {
    std::unique_ptr<radio_xtrx_tx_stream> stream =
        std::make_unique<radio_xtrx_tx_stream>(xtrx_handle, description, async_executor, notifier);

    if (stream->is_successful()) {
      return stream;
    }

    return nullptr;
  }

  std::unique_ptr<radio_xtrx_rx_stream> create_rx_stream(radio_notification_handler&                    notifier,
                                                        const radio_xtrx_rx_stream::stream_description& description)
  {
    std::unique_ptr<radio_xtrx_rx_stream> stream = std::make_unique<radio_xtrx_rx_stream>(xtrx_handle, description, notifier);

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
      if (!radio_xtrx_device_validate_gain_range(1, gain)) {
        on_error("Tx gain (i.e., {} dB) is out-of-range. Range is [{}, {}] dB in steps of {} dB.",
                 gain,
                 -52,
                 0,
                 0.1);
        return;
      }

      int res = xtrx_set_gain(xtrx_handle->dev(), XTRXHandle::xtrx_channel(ch), XTRX_TX_PAD_GAIN, gain, NULL);
      if (res) {
        on_error("XTRX failed to set TX gain (returned {})",
                 res);
      }
    });

    return 0;
  }

  bool set_rx_gain(size_t ch, double gain)
  {
    logger.debug("Setting channel {} Rx gain to {:.2f} dB.", ch, gain);

    return safe_execution([this, ch, gain]() {
      if (!radio_xtrx_device_validate_gain_range(0, gain)) {
        on_error("Rx gain (i.e., {} dB) is out-of-range. Range is [{}, {}] dB in steps of {} dB.",
                 gain,
                 0,
                 30,
                 0.1);
        return;
      }

      int res = xtrx_set_gain(xtrx_handle->dev(), XTRXHandle::xtrx_channel(ch), XTRX_RX_LNA_GAIN, gain, NULL);
      if (res) {
        on_error("XTRX failed to set RX gain (returned {})",
                 res);
      }
    });
  }

  bool set_tx_freq(uint32_t ch, const radio_configuration::lo_frequency& config)
  {
    logger.debug("Setting channel {} Tx frequency to {} MHz.", ch, config.center_frequency_hz*1e-6);

    return safe_execution([this, ch, &config]() {
      if (!radio_xtrx_device_validate_freq_range(config.center_frequency_hz)) {
        on_error("Tx RF frequency {} MHz is out-of-range. Range is {} - {}.",
                 config.center_frequency_hz*1e-6,
                 30,
                 3.8e3);
        return;
      }

      double _actual_rf_tx = 0;
      int res = xtrx_tune(xtrx_handle->dev(), XTRX_TUNE_TX_FDD, config.center_frequency_hz, &_actual_rf_tx);
      if (res) {
        on_error("XTRX failed to tune TX (returned {})",
                 res);
      }
      logger.debug("Result: channel {} Tx frequency actual = {} MHz.", ch, _actual_rf_tx*1e-6);
    });
  }

  bool set_rx_freq(uint32_t ch, const radio_configuration::lo_frequency& config)
  {
    logger.debug("Setting channel {} Rx frequency to {} MHz.", ch, config.center_frequency_hz*1e-6);

    return safe_execution([this, ch, &config]() {
      if (!radio_xtrx_device_validate_freq_range(config.center_frequency_hz)) {
        on_error("Rx RF frequency {} MHz is out-of-range. Range is {} - {}.",
                 config.center_frequency_hz,
                 30,
                 3.8e3);
        return;
      }

      double _actual_rf_rx = 0;
      int res = xtrx_tune(xtrx_handle->dev(), XTRX_TUNE_RX_FDD, config.center_frequency_hz, &_actual_rf_rx);
      if (res) {
        on_error("XTRX failed to tune RX (returned {})",
                 res);
      }
      logger.debug("Result: channel {} Rx frequency actual = {} MHz.", ch, _actual_rf_rx*1e-6);
    });
  }

private:
  std::shared_ptr<XTRXHandle> xtrx_handle = nullptr;
  srslog::basic_logger&       logger;
};

} // namespace srsran
