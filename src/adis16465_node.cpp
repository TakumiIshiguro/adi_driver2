// Copyright (c) 2017, Analog Devices Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#include "adi_driver2/adis16465_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unistd.h>

using namespace std::chrono_literals;

namespace adi_driver2
{

ImuNode::ImuNode()
: Node("adis16465_node"), imu_(std::make_shared<Adis16470>()),
  system_clock_(RCL_SYSTEM_TIME)
{
  // Read parameters
  declare_parameter("device", "/dev/ttyACM0");
  declare_parameter("frame_id", "imu");
  declare_parameter("burst_mode", true);
  declare_parameter("publish_temperature", true);
  declare_parameter("rate", 200.0);
  declare_parameter("io_timeout_ms", 4);

  device_ = get_parameter("device").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  burst_mode_ = get_parameter("burst_mode").as_bool();
  publish_temperature_ = get_parameter("publish_temperature").as_bool();
  rate_ = get_parameter("rate").as_double();
  io_timeout_ms_ = get_parameter("io_timeout_ms").as_int();

  constexpr double kInternalSampleRateHz = 2000.0;
  constexpr int64_t kMaxDecimationFactor = 2000;
  if (!std::isfinite(rate_) || rate_ <= 0.0 || rate_ > kInternalSampleRateHz) {
    throw std::invalid_argument("rate must be finite and in the range (0, 2000] Hz");
  }

  const double requested_decimation_factor = kInternalSampleRateHz / rate_;
  const int64_t decimation_factor = std::llround(requested_decimation_factor);
  if (decimation_factor < 1 || decimation_factor > kMaxDecimationFactor ||
    std::abs(requested_decimation_factor - static_cast<double>(decimation_factor)) > 1e-9)
  {
    throw std::invalid_argument("rate must be an integer divisor of the ADIS16465 2000 Hz rate");
  }

  decimation_rate_ = static_cast<int16_t>(decimation_factor - 1);
  loop_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / rate_));
  if (io_timeout_ms_ < 1 || io_timeout_ms_ > 100) {
    throw std::invalid_argument("io_timeout_ms must be in the range [1, 100]");
  }
  imu_->set_io_timeout(std::chrono::milliseconds(io_timeout_ms_));

  RCLCPP_INFO(this->get_logger(), "device: %s", device_.c_str());
  RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "rate: %.3f Hz (DEC_RATE: 0x%04x)",
    rate_, static_cast<uint16_t>(decimation_rate_));
  RCLCPP_INFO(
    this->get_logger(), "burst_mode: %s",
    (burst_mode_ ? "true" : "false"));
  RCLCPP_INFO(
    this->get_logger(), "publish_temperature: %s",
    (publish_temperature_ ? "true" : "false"));
  RCLCPP_INFO(this->get_logger(), "io_timeout_ms: %d", io_timeout_ms_);

  // Data publisher
  imu_data_pub_ = create_publisher<sensor_msgs::msg::Imu>("data_raw", 100);
  if (publish_temperature_) {
    temp_data_pub_ =
      create_publisher<sensor_msgs::msg::Temperature>("temperature", 100);
  }
  // Bias estimate service

  bias_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "bias_estimate",
    std::bind(&ImuNode::bias_estimate, this, std::placeholders::_1, std::placeholders::_2));

  while (!is_opened()) {
    RCLCPP_WARN(this->get_logger(), "Keep trying to open the device in 1 second period...");
    sleep(1);
    open();
  }

  start_acquisition();
}

ImuNode::~ImuNode()
{
  acquisition_running_ = false;
  if (acquisition_thread_.joinable()) {
    acquisition_thread_.join();
  }
  RCLCPP_INFO(
    this->get_logger(),
    "IMU acquisition summary: io_errors=%llu deadline_misses=%llu gaps=%llu max_gap=%.3f ms",
    static_cast<unsigned long long>(io_error_count_),
    static_cast<unsigned long long>(deadline_miss_count_),
    static_cast<unsigned long long>(gap_count_), max_gap_ms_);
  imu_->closePort();
}

bool ImuNode::bias_estimate(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  RCLCPP_INFO(this->get_logger(), "bias_estimate");

  const std::lock_guard<std::mutex> lock(imu_mutex_);
  if (imu_->bias_correction_update() < 0) {
    response->success = false;
    response->message = "Bias correction update failed";

    return false;
  }
  response->success = true;
  response->message = "Success";

  return true;
}

/**
 * @brief Check if the device is opened
 */
bool ImuNode::is_opened(void) {return imu_->fd_ >= 0;}

/**
 * @brief Open IMU device file
 */
void ImuNode::open(void)
{
  // Open device file
  if (imu_->openPort(device_) < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to open device %s",
      device_.c_str());
  }
  // Wait 10ms for SPI ready
  usleep(10000);
  int16_t pid = 0;
  if (imu_->get_product_id(pid) == 0) {
    RCLCPP_INFO(this->get_logger(), "Product ID: %x", pid);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to read product ID");
    imu_->closePort();
    return;
  }
  if (imu_->configure_gyro_scale() < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to determine gyroscope range from RANG_MDL");
    imu_->closePort();
    return;
  }
  RCLCPP_INFO(
    this->get_logger(), "Gyroscope range: +/- %d deg/s (%.6f deg/s/LSB)",
    imu_->gyro_range_dps(), 1.0 / imu_->gyro_lsb_per_dps());
  if (imu_->set_bias_estimation_time(0x070a) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set bias estimation time");
  }
  if (imu_->set_decimation_rate(decimation_rate_) < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to set DEC_RATE to 0x%04x",
      static_cast<uint16_t>(decimation_rate_));
  }
}

void ImuNode::publish_imu_data(const rclcpp::Time & stamp)
{
  sensor_msgs::msg::Imu data;
  data.header.frame_id = frame_id_;
  data.header.stamp = stamp;

  // Linear acceleration
  data.linear_acceleration.x = imu_->accl[0];
  data.linear_acceleration.y = imu_->accl[1];
  data.linear_acceleration.z = imu_->accl[2];

  // Angular velocity
  data.angular_velocity.x = imu_->gyro[0];
  data.angular_velocity.y = imu_->gyro[1];
  data.angular_velocity.z = imu_->gyro[2];

  // Orientation (not provided)
  data.orientation.x = 0;
  data.orientation.y = 0;
  data.orientation.z = 0;
  data.orientation.w = 1;

  imu_data_pub_->publish(data);
}

void ImuNode::publish_temp_data(const rclcpp::Time & stamp)
{
  sensor_msgs::msg::Temperature data;
  data.header.frame_id = frame_id_;
  data.header.stamp = stamp;

  // imu Temperature
  data.temperature = imu_->temp;
  data.variance = 0;

  temp_data_pub_->publish(data);
}

void ImuNode::start_acquisition()
{
  acquisition_running_ = true;
  acquisition_thread_ = std::thread(&ImuNode::acquisition_loop, this);
}

void ImuNode::acquisition_loop()
{
  using SteadyClock = std::chrono::steady_clock;
  auto next_sample_time = SteadyClock::now();
  auto last_publish_time = SteadyClock::time_point{};

  while (acquisition_running_ && rclcpp::ok()) {
    std::this_thread::sleep_until(next_sample_time);
    if (!acquisition_running_ || !rclcpp::ok()) {
      break;
    }

    int update_result = -1;
    {
      const std::lock_guard<std::mutex> lock(imu_mutex_);
      update_result = burst_mode_ ? imu_->update_burst() : imu_->update();
    }

    const auto publish_time = SteadyClock::now();
    if (update_result == 0) {
      const rclcpp::Time stamp = system_clock_.now();
      publish_imu_data(stamp);
      if (!burst_mode_ && publish_temperature_) {
        publish_temp_data(stamp);
      }

      if (last_publish_time != SteadyClock::time_point{}) {
        const double gap_ms = std::chrono::duration<double, std::milli>(
          publish_time - last_publish_time).count();
        if (gap_ms > 1.5 * std::chrono::duration<double, std::milli>(loop_period_).count()) {
          ++gap_count_;
          max_gap_ms_ = std::max(max_gap_ms_, gap_ms);
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), system_clock_, 5000,
            "IMU publish gap %.3f ms (gaps=%llu, io_errors=%llu, deadline_misses=%llu)",
            gap_ms, static_cast<unsigned long long>(gap_count_),
            static_cast<unsigned long long>(io_error_count_),
            static_cast<unsigned long long>(deadline_miss_count_));
        }
      }
      last_publish_time = publish_time;
    } else {
      ++io_error_count_;
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), system_clock_, 5000,
        "Cannot read IMU (io_errors=%llu)",
        static_cast<unsigned long long>(io_error_count_));
    }

    next_sample_time += loop_period_;
    const auto now = SteadyClock::now();
    if (now >= next_sample_time) {
      const auto overdue = now - next_sample_time;
      const uint64_t missed =
        static_cast<uint64_t>(overdue / loop_period_) + 1;
      deadline_miss_count_ += missed;
      next_sample_time += loop_period_ * missed;
    }
  }
}
} // namespace adi_driver2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  spin(std::make_shared<adi_driver2::ImuNode>());
  rclcpp::shutdown();
  return 0;
}
