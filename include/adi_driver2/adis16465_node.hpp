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

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include <chrono>
#include <cstdint>
#include <string>

#include "adi_driver2/adis16465.h"

namespace adi_driver2
{
class ImuNode : public rclcpp::Node
{
public:
  ImuNode();
  ~ImuNode();

  bool bias_estimate(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response
  );

  bool is_opened(void);
  void open(void);
  void publish_imu_data(void);
  void publish_temp_data(void);

  void loop(void);

  std::shared_ptr<Adis16470> imu_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_data_pub_;
  //   ros::ServiceServer bias_srv_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr bias_srv_;

  rclcpp::TimerBase::SharedPtr loop_timer_;

  rclcpp::Clock system_clock_;

  std::string device_;
  std::string frame_id_;
  bool burst_mode_;
  bool publish_temperature_;
  double rate_;
  int16_t decimation_rate_;
  std::chrono::nanoseconds loop_period_;

};
} // namespace adi_driver2
