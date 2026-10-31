// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std::placeholders::_1;
using std_msgs::msg::Float64;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options)
{
  // declare parameters
  declare_parameter("speed_to_erpm_gain", 0.0);
  declare_parameter("speed_to_erpm_offset", 0.0);
  declare_parameter("steering_angle_to_servo_gain", 0.0);
  declare_parameter("steering_angle_to_servo_offset", 0.0);

  // Servo travel limits (the same params vesc_driver clips the command to) and
  // an optional explicit steering limit.
  declare_parameter("servo_min", 0.0);
  declare_parameter("servo_max", 0.0);
  const double steering_angle_max_param = declare_parameter("steering_angle_max", 0.0);

  // get conversion parameters
  speed_to_erpm_gain_ = get_parameter("speed_to_erpm_gain").get_value<double>();
  speed_to_erpm_offset_ = get_parameter("speed_to_erpm_offset").get_value<double>();
  steering_to_servo_gain_ = get_parameter("steering_angle_to_servo_gain").get_value<double>();
  steering_to_servo_offset_ = get_parameter("steering_angle_to_servo_offset").get_value<double>();

  // The steering rack is not symmetric about the straight-ahead servo value:
  // with offset=0.55 and servo_min/max=0.15/0.85 the rack has 0.40 servo units
  // of travel to the left but only 0.30 to the right, so a command of +0.33 rad
  // is reachable while -0.33 rad is not. vesc_driver clips the servo silently,
  // which means the car understeers to one side and the controller keeps asking
  // for an angle it will never get. Clamp the requested angle to the smaller of
  // the two reaches so both directions behave identically.
  steering_angle_max_ = steering_angle_max_param;
  if (steering_angle_max_ <= 0.0 && std::fabs(steering_to_servo_gain_) > 1e-6) {
    const double servo_min = get_parameter("servo_min").get_value<double>();
    const double servo_max = get_parameter("servo_max").get_value<double>();

    if (servo_max > servo_min) {
      // angle = (servo - offset) / gain; the sign of the gain decides which
      // end is left, so take the magnitude of each end rather than assume.
      const double reach_a = std::fabs(
        (servo_min - steering_to_servo_offset_) / steering_to_servo_gain_);
      const double reach_b = std::fabs(
        (servo_max - steering_to_servo_offset_) / steering_to_servo_gain_);
      steering_angle_max_ = std::min(reach_a, reach_b);

      RCLCPP_INFO(
        get_logger(),
        "Steering clamped to +/-%.4f rad (%.2f deg): servo %.3f..%.3f around "
        "offset %.3f gives %.4f / %.4f rad of travel",
        steering_angle_max_, steering_angle_max_ * 180.0 / 3.14159265358979323846,
        servo_min, servo_max, steering_to_servo_offset_, reach_a, reach_b);
      if (std::fabs(reach_a - reach_b) > 0.01) {
        RCLCPP_WARN(
          get_logger(),
          "Steering rack is asymmetric (%.4f vs %.4f rad); the larger side is "
          "restricted to match. Widen servo_min/servo_max if the linkage allows "
          "more travel and this limit expands automatically.",
          reach_a, reach_b);
      }
    }
  } else if (steering_angle_max_ > 0.0) {
    RCLCPP_INFO(
      get_logger(), "Steering clamped to +/-%.4f rad (steering_angle_max)",
      steering_angle_max_);
  }

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));
}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  // calc vesc electric RPM (speed)
  Float64 erpm_msg;
  erpm_msg.data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

  // calc steering angle (servo), clamped to the symmetric rack limit
  double steering_angle = cmd->drive.steering_angle;
  if (steering_angle_max_ > 0.0) {
    steering_angle = std::clamp(steering_angle, -steering_angle_max_, steering_angle_max_);
  }

  Float64 servo_msg;
  servo_msg.data = steering_to_servo_gain_ * steering_angle + steering_to_servo_offset_;

  // publish
  if (rclcpp::ok()) {
    erpm_pub_->publish(erpm_msg);
    servo_pub_->publish(servo_msg);
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)
