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

#include "radio_lime_impl.h"
#include "radio_lime_device.h"
#include <thread>
#include <uhd/utils/thread_priority.h>

using namespace srsran;

// TODO
bool radio_session_lime_impl::set_time_to_gps_time()
{
  // const std::string sensor_name = "gps_time";

  // std::vector<std::string> sensors;
  // if (device.get_mboard_sensor_names(sensors) != LIME_ERROR_NONE) {
  //   fmt::print("Error: failed to read sensors. {}\n", device.get_error_message());
  //   return false;
  // }

  // // Find sensor name. Error if it is not available.
  // if (std::find(sensors.begin(), sensors.end(), sensor_name) == sensors.end()) {
  //   fmt::print("Error: sensor {} not found.\n", sensor_name);
  //   return false;
  // }

  // // Get actual sensor value
  // double frac_secs = 0.0;
  // if (!device.get_sensor(sensor_name, frac_secs)) {
  //   fmt::print("Error: not possible to read sensor {}. {}\n", sensor_name, device.get_error_message());
  //   return false;
  // }

  // // Get time and set
  // fmt::print("Setting {} time to {}s\n", LMS_GetDeviceInfo(&device.get_type()), frac_secs);
  // if (device.set_time_unknown_pps(lime::time_spec_t(frac_secs)) != LIME_ERROR_NONE) {
  //   fmt::print("Error: failed to set time. {}\n", device.get_error_message());
  //   return false;
  // }

  return true;
}

bool radio_session_lime_impl::wait_sensor_locked(const std::string& sensor_name, bool is_mboard, int timeout)
{
  auto end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout);

  // Get sensor list
  std::vector<std::string> sensors;
  if (is_mboard) {
    // motherboard sensor
    if (!device.get_mboard_sensor_names(sensors)) {
      fmt::print("Error: getting mboard sensor names. {}", device.get_error_message());
      return false;
    }
  } else {
    // daughterboard sensor
    if (!device.get_rx_sensor_names(sensors)) {
      fmt::print("Error: getting Rx sensor names. {}", device.get_error_message());
      return false;
    }
  }

  // Find sensor name. Error if it is not available.
  if (std::find(sensors.begin(), sensors.end(), sensor_name) == sensors.end()) {
    fmt::print("Error: sensor {} not found.\n", sensor_name);
    return false;
  }

  do {
    // Get actual sensor value
    bool is_locked = false;
    if (is_mboard) {
      if (!device.get_sensor(sensor_name, is_locked)) {
        fmt::print("Error: reading mboard sensor {}. {}.\n", sensor_name, device.get_error_message());
        return false;
      }
    } else {
      if (!device.get_rx_sensor(sensor_name, is_locked)) {
        fmt::print("Error: reading rx sensor {}. {}.\n", sensor_name, device.get_error_message());
        return false;
      }
    }

    // Return true if the sensor is locked.
    if (is_locked) {
      return true;
    }

    // Sleep some time before trying it again.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  } while (std::chrono::steady_clock::now() < end_time);

  return false;
}

bool radio_session_lime_impl::set_tx_gain_unprotected(unsigned port_idx, double gain_dB)
{
  if (port_idx >= tx_port_map.size()) {
    fmt::print(
        "Error: transmit port index ({}) exceeds the number of ports ({}).\n", port_idx, (int)tx_port_map.size());
    return false;
  }

  // Setup gain.
  if (!device.set_tx_gain(port_idx, gain_dB)) {
    fmt::print("Error: setting gain for transmitter {}. {}\n", port_idx, device.get_error_message());
  }

  return true;
}

bool radio_session_lime_impl::set_rx_gain_unprotected(unsigned port_idx, double gain_dB)
{
  if (port_idx >= rx_port_map.size()) {
    fmt::print("Error: receive port index ({}) exceeds the number of ports ({}).\n", port_idx, (int)rx_port_map.size());
    return false;
  }

  // Setup gain.
  if (!device.set_rx_gain(port_idx, gain_dB)) {
    fmt::print("Error: setting gain for receiver {}. {}\n", port_idx, device.get_error_message());
    return false;
  }

  return true;
}

bool radio_session_lime_impl::set_tx_freq(unsigned port_idx, radio_configuration::lo_frequency frequency)
{
  if (port_idx >= tx_port_map.size()) {
    fmt::print(
        "Error: transmit port index ({}) exceeds the number of ports ({}).\n", port_idx, (int)tx_port_map.size());
    return false;
  }

  // Setup frequency.
  if (!device.set_tx_freq(port_idx, frequency)) {
    fmt::print("Error: setting frequency for transmitter {}. {}\n", port_idx, device.get_error_message());
    return false;
  }

  return true;
}

bool radio_session_lime_impl::set_rx_freq(unsigned port_idx, radio_configuration::lo_frequency frequency)
{
  if (port_idx >= rx_port_map.size()) {
    fmt::print("Error: receive port index ({}) exceeds the number of ports ({}).\n", port_idx, (int)tx_port_map.size());
    return false;
  }

  // Setup frequency.
  if (!device.set_rx_freq(port_idx, frequency)) {
    fmt::print("Error: setting frequency for receiver {}. {}.\n", port_idx, device.get_error_message());
    return false;
  }

  return true;
}

bool radio_session_lime_impl::start_rx_stream(baseband_gateway_timestamp init_time)
{
  // Prevent multiple threads from starting streams simultaneously.
  std::unique_lock<std::mutex> lock(stream_start_mutex);

  if (!stream_start_required) {
    return true;
  }

  // Flag stream start is no longer required.
  stream_start_required = false;

  // Immediate start of the stream.
  uint64_t time_spec = 0;

  // Get current radio time as timestamp.
  // TODO: use init_time?
  if (!device.get_time_now(time_spec)) {
    fmt::print("Error: getting time to start stream. {}.\n", device.get_error_message());
    return false;
  }
  // Add delay to current time.
  time_spec += START_STREAM_DELAY_S * sampling_rate_hz;

  // Issue all streams to start.
  for (auto& bb_gateway : bb_gateways) {
    if (!bb_gateway->get_rx_stream().start(time_spec)) {
      return false;
    }
  }

  return true;
}

radio_session_lime_impl::radio_session_lime_impl(const radio_configuration::radio& radio_config,
                                               task_executor&                    async_executor_,
                                               radio_notification_handler&       notifier_) :
  sampling_rate_hz(radio_config.sampling_rate_hz), async_executor(async_executor_), notifier(notifier_)
{
  // Set the logging level.
  #ifdef LIME_LOG_INFO
  // TODO: add logging!
  lime::SDRDevice::LogLevel severity_level = lime::SDRDevice::INFO;
  if (!radio_config.log_level.empty()) {
    std::string log_level = radio_config.log_level;

    for (auto& e : log_level) {
      e = std::toupper(e);
    }

    if (log_level == "WARNING") {
      severity_level = lime::SDRDevice::WARNING;
    } else if (log_level == "INFO") {
      severity_level = lime::SDRDevice::INFO;
    } else if (log_level == "DEBUG") {
      severity_level = lime::SDRDevice::DEBUG;
    } else if (log_level == "TRACE") {
      severity_level = lime::SDRDevice::VERBOSE;
    } else {
      severity_level = lime::SDRDevice::ERROR;
    }
  }
  lime::log::set_console_level(severity_level);
  #endif

  unsigned total_rx_channel_count = 0;
  for (const radio_configuration::stream& stream_config : radio_config.rx_streams) {
    total_rx_channel_count += stream_config.channels.size();
  }

  unsigned total_tx_channel_count = 0;
  for (const radio_configuration::stream& stream_config : radio_config.rx_streams) {
    total_tx_channel_count += stream_config.channels.size();
  }

  // Open device.
  if (!device.lime_make(radio_config.args)) {
    fmt::print("Failed to open device with address '{}': {}\n", radio_config.args, device.get_error_message());
    return;
  }

  bool        is_locked = false;
  std::string sensor_name;

  // Set sync source.
  if (!device.set_sync_source(radio_config.clock)) {
    fmt::print("Error: couldn't set sync source: {}\n", device.get_error_message());
    return;
  }

  // Set GPS time if GPSDO is selected.
  if (radio_config.clock.sync == radio_configuration::clock_sources::source::GPSDO) {
    set_time_to_gps_time();
  }

  // Select oscillator sensor name.
  if (radio_config.clock.sync == radio_configuration::clock_sources::source::GPSDO) {
    sensor_name = "gps_locked";
  } else {
    sensor_name = "ref_locked";
  }

  // Wait until external reference / GPS is locked.
  if (radio_config.clock.sync == radio_configuration::clock_sources::source::GPSDO ||
      radio_config.clock.sync == radio_configuration::clock_sources::source::EXTERNAL) {
    // blocks until clock source is locked
    if (not wait_sensor_locked(sensor_name, true, CLOCK_TIMEOUT_MS)) {
      fmt::print("Could not lock reference clock source. Sensor: {}={}.\n", sensor_name, is_locked ? "true" : "false");
      return;
    }
  }

  // Set Tx rate.
  if (!device.set_tx_rate(radio_config.sampling_rate_hz)) {
    fmt::print("Error: setting Tx sampling rate. {}\n", device.get_error_message());
    return;
  }

  // Set Rx rate.
  if (!device.set_rx_rate(radio_config.sampling_rate_hz)) {
    fmt::print("Error: setting Rx sampling rate. {}\n", device.get_error_message());
    return;
  }

  // Reset timestamps.
  if ((total_rx_channel_count > 1 || total_tx_channel_count > 1) &&
      radio_config.clock.sync != radio_configuration::clock_sources::source::GPSDO) {
    device.set_time_unknown_pps(0);
  }

  // Lists of stream descriptions.
  std::vector<radio_lime_tx_stream::stream_description> tx_stream_description_list;
  std::vector<radio_lime_rx_stream::stream_description> rx_stream_description_list;

  // For each transmit stream, create stream and configure RF ports.
  for (unsigned stream_idx = 0; stream_idx != radio_config.tx_streams.size(); ++stream_idx) {
    // Select stream.
    const radio_configuration::stream& stream = radio_config.tx_streams[stream_idx];

    // Prepare stream description.
    radio_lime_tx_stream::stream_description stream_description = {};
    stream_description.id                                      = stream_idx;
    stream_description.otw_format                              = radio_config.otw_format;
    stream_description.srate_hz                                = radio_config.sampling_rate_hz;
    stream_description.args                                    = radio_config.args;

    // Setup ports.
    for (unsigned channel_idx = 0; channel_idx != stream.channels.size(); ++channel_idx) {
      // Select the port index.
      unsigned port_idx = tx_port_map.size();

      // Set channel port.
      stream_description.ports.emplace_back(port_idx);

      // Save the stream and channel indexes for the port.
      tx_port_map.emplace_back(port_to_stream_channel(stream_idx, channel_idx));
    }

    // Setup port.
    for (unsigned channel_idx = 0; channel_idx != stream.channels.size(); ++channel_idx) {
      // Get the port index.
      unsigned port_idx = stream_description.ports[channel_idx];

      // Extract port configuration.
      const radio_configuration::channel& channel = stream.channels[channel_idx];

      // Setup gain.
      set_tx_gain_unprotected(port_idx, channel.gain_dB);

      // Setup frequency.
      if (!set_tx_freq(port_idx, channel.freq)) {
        return;
      }

      // Inform about ignored argument.
      if (!channel.args.empty()) {
        fmt::print("Warning: transmitter {} unused args.\n", port_idx);
      }
    }

    // Add stream description to the list.
    tx_stream_description_list.emplace_back(stream_description);
  }

  // For each receive stream, create stream and configure RF ports.
  for (unsigned stream_idx = 0; stream_idx != radio_config.rx_streams.size(); ++stream_idx) {
    // Select stream.
    const radio_configuration::stream& stream = radio_config.rx_streams[stream_idx];

    // Prepare stream description.
    radio_lime_rx_stream::stream_description stream_description = {};
    stream_description.id                                      = stream_idx;
    stream_description.otw_format                              = radio_config.otw_format;
    stream_description.srate_Hz                                = radio_config.sampling_rate_hz;
    stream_description.args                                    = radio_config.args;

    // Setup ports.
    for (unsigned channel_idx = 0; channel_idx != stream.channels.size(); ++channel_idx) {
      // Select the port index.
      unsigned port_idx = rx_port_map.size();

      // Set channel port.
      stream_description.ports.emplace_back(port_idx);

      // Save the stream and channel indexes for the port.
      rx_port_map.emplace_back(port_to_stream_channel(stream_idx, channel_idx));
    }

    // Setup port.
    for (unsigned channel_idx = 0; channel_idx != stream.channels.size(); ++channel_idx) {
      // Get the port index.
      unsigned port_idx = stream_description.ports[channel_idx];

      // Extract port configuration.
      const radio_configuration::channel& channel = stream.channels[channel_idx];

      // Setup gain.
      if (!set_rx_gain_unprotected(port_idx, channel.gain_dB)) {
        return;
      }

      // Setup frequency.
      if (!set_rx_freq(port_idx, channel.freq)) {
        return;
      }

      // Inform about ignored argument.
      if (!channel.args.empty()) {
        fmt::print("Warning: transmitter {} unused args.\n", port_idx);
      }
    }

    // Add stream description to the list.
    rx_stream_description_list.emplace_back(stream_description);
  }

  // Create baseband gateways.
  for (unsigned i_stream = 0; i_stream != radio_config.tx_streams.size(); ++i_stream) {
    bb_gateways.emplace_back(std::make_unique<radio_lime_baseband_gateway>(
        device, async_executor, notifier, tx_stream_description_list[i_stream], rx_stream_description_list[i_stream]));

    // Early return if the gateway was not successfully created.
    if (!bb_gateways.back()->is_successful()) {
      return;
    }
  }

  // Set the radio configuration.
  device.execute_config(radio_config.args);

  // Transition to successfully initialized.
  state = states::SUCCESSFUL_INIT;
}

void radio_session_lime_impl::stop()
{
  // Transition state to stop.
  state = states::STOP;

  // Signal stop for each transmit stream.
  for (auto& gateway : bb_gateways) {
    gateway->get_tx_stream().stop();
  }

  // Signal stop for each receive stream.
  for (auto& gateway : bb_gateways) {
    gateway->get_rx_stream().stop();
  }

  // Wait for streams to join.
  for (auto& gateway : bb_gateways) {
    gateway->get_tx_stream().wait_stop();
    gateway->get_rx_stream().wait_stop();
  }
}

void radio_session_lime_impl::start(baseband_gateway_timestamp init_time)
{
  if (!start_rx_stream(init_time)) {
    fmt::print("Failed to start Rx streams.\n");
  }
}

baseband_gateway_timestamp radio_session_lime_impl::read_current_time()
{
  // TODO!
  // uhd::time_spec_t time;
  // if (!device.get_time_now(time)) {
  //   fmt::print("Error retrieving time.\n");
  // }
  // return time.to_ticks(actual_sampling_rate_Hz);
  return 1;
}

std::unique_ptr<radio_session> radio_factory_lime_impl::create(const radio_configuration::radio& config,
                                                              task_executor&                    async_task_executor,
                                                              radio_notification_handler&       notifier)
{
  std::unique_ptr<radio_session_lime_impl> session =
      std::make_unique<radio_session_lime_impl>(config, async_task_executor, notifier);
  if (!session->is_successful()) {
    return nullptr;
  }

  return std::move(session);
}

radio_config_lime_config_validator radio_factory_lime_impl::config_validator;
