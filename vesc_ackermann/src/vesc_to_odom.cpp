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

#include "vesc_ackermann/vesc_to_odom.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

VescToOdom::VescToOdom(const rclcpp::NodeOptions & options)
: Node("vesc_to_odom_node", options),
  odom_frame_("odom"),
  base_frame_("base_link"),
  use_servo_cmd_(true),
  publish_tf_(false),
  twist_cov_vx_(0.01),
  twist_cov_vy_(0.0025),
  twist_cov_yaw_base_(0.03),
  twist_cov_yaw_scale_(0.25),
  servo_lag_s_(0.10),
  have_servo_cmd_(false),
  last_servo_angle_(0.0),
  servo_rate_rad_s_(0.0),
  x_(0.0),
  y_(0.0),
  yaw_(0.0)
{
  // get ROS parameters
  odom_frame_ = declare_parameter("odom_frame", odom_frame_);
  base_frame_ = declare_parameter("base_frame", base_frame_);
  use_servo_cmd_ = declare_parameter("use_servo_cmd_to_calc_angular_velocity", use_servo_cmd_);
  
  declare_parameter<double>("speed_to_erpm_gain", 0.0);
  declare_parameter<double>("speed_to_erpm_offset", 0.0);

  speed_to_erpm_gain_ = get_parameter("speed_to_erpm_gain").get_value<double>();
  speed_to_erpm_offset_ = get_parameter("speed_to_erpm_offset").get_value<double>();

  if (use_servo_cmd_) {
    declare_parameter<double>("steering_angle_to_servo_gain", 0.0);
    declare_parameter<double>("steering_angle_to_servo_offset", 0.0);
    declare_parameter<double>("wheelbase", 0.0);
    
    steering_to_servo_gain_ = get_parameter("steering_angle_to_servo_gain").get_value<double>();
    steering_to_servo_offset_ = get_parameter("steering_angle_to_servo_offset").get_value<double>();
    wheelbase_ = get_parameter("wheelbase").get_value<double>();
  }

  publish_tf_ = declare_parameter("publish_tf", publish_tf_);

  // Twist measurement noise. robot_localization has no per-input covariance
  // scaling -- it uses the covariance in the message verbatim, and clamps any
  // variance below 1e-9 up to 1e-9 (i.e. "this measurement is exact"). A
  // driver that leaves the twist covariance at zero therefore tells the EKF
  // that both the speed and the steering echo are perfect, and consecutive
  // updates from different sources simply overwrite each other.
  twist_cov_vx_ = declare_parameter("twist_cov_vx", twist_cov_vx_);
  twist_cov_vy_ = declare_parameter("twist_cov_vy", twist_cov_vy_);
  twist_cov_yaw_base_ = declare_parameter("twist_cov_yaw_base", twist_cov_yaw_base_);
  twist_cov_yaw_scale_ = declare_parameter("twist_cov_yaw_scale", twist_cov_yaw_scale_);
  servo_lag_s_ = declare_parameter("servo_lag_s", servo_lag_s_);
  servo_lag_s_ = std::max(servo_lag_s_, 1e-3);

  // create odom publisher
  odom_pub_ = create_publisher<Odometry>("odom", 10);

  // create tf broadcaster
  if (publish_tf_) {
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }

  // subscribe to vesc state and. optionally, servo command
  vesc_state_sub_ = create_subscription<VescStateStamped>(
    "sensors/core", 10, std::bind(&VescToOdom::vescStateCallback, this, _1));

  if (use_servo_cmd_) {
    servo_sub_ = create_subscription<Float64>(
      "sensors/servo_position_command", 10, std::bind(&VescToOdom::servoCmdCallback, this, _1));
  }
}

void VescToOdom::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  // check that we have a last servo command if we are depending on it for angular velocity
  if (use_servo_cmd_ && !last_servo_cmd_) {
    return;
  }

  // convert to engineering units
  double current_speed = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  if (std::fabs(current_speed) < 0.05) {
    current_speed = 0.0;
  }
  double current_steering_angle(0.0), current_angular_velocity(0.0);
  if (use_servo_cmd_) {
    current_steering_angle =
      (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
    current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
  }

  // use current state as last state if this is our first time here
  if (!last_state_) {
    last_state_ = state;
  }

  // calc elapsed time
  auto dt = rclcpp::Time(state->header.stamp) - rclcpp::Time(last_state_->header.stamp);

  /** @todo could probably do better propigating odometry, e.g. trapezoidal integration */

  // propigate odometry
  double x_dot = current_speed * cos(yaw_);
  double y_dot = current_speed * sin(yaw_);
  x_ += x_dot * dt.seconds();
  y_ += y_dot * dt.seconds();
  if (use_servo_cmd_) {
    yaw_ += current_angular_velocity * dt.seconds();
  }

  // save state for next time
  last_state_ = state;

  // publish odometry message
  Odometry odom;
  odom.header.frame_id = odom_frame_;
  odom.header.stamp = state->header.stamp;
  odom.child_frame_id = base_frame_;

  // Position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sin(yaw_ / 2.0);
  odom.pose.pose.orientation.w = cos(yaw_ / 2.0);

  // Position uncertainty
  /** @todo Think about position uncertainty, perhaps get from parameters? */
  odom.pose.covariance[0] = 0.2;   ///< x
  odom.pose.covariance[7] = 0.2;   ///< y
  odom.pose.covariance[35] = 0.4;  ///< yaw

  // Velocity ("in the coordinate frame given by the child_frame_id")
  odom.twist.twist.linear.x = current_speed;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = current_angular_velocity;

  // Velocity uncertainty.
  //
  // The yaw rate is derived from the *commanded* servo position rather than a
  // measured wheel angle, so it carries two kinds of error:
  //
  //  1. A scale error. wheelbase in vesc.yaml is not the measured wheelbase,
  //     and vyaw = v * tan(steer) / L is linear in 1/L, so the estimate is off
  //     by a constant fraction of itself. Modelled as a term proportional to
  //     |vyaw| (twist_cov_yaw_scale). Set it near zero once the wheelbase is
  //     verified.
  //
  //  2. A lag error. While the servo is still slewing the command leads the
  //     wheels, so the estimate is wrong by roughly dyaw/dt * lag, which for a
  //     full lock-to-lock sweep at speed is several rad/s. It collapses to the
  //     sensor noise floor once the servo settles, hence the decay with time
  //     since the last servo command arrived.
  //
  // Reporting this honestly lets the EKF use the wheel yaw rate when it is
  // trustworthy (it is a gyro-bias-free kinematic constraint) and fall back to
  // the IMU while the steering is moving, instead of the all-or-nothing choice
  // between fusing the echo as exact and dropping it entirely.
  double steering_slew_rad_s = 0.0;
  if (have_servo_cmd_) {
    const double servo_age = std::max(0.0, (now() - last_servo_time_).seconds());
    steering_slew_rad_s = servo_rate_rad_s_ * std::exp(-servo_age / servo_lag_s_);
  }
  const double lag_sigma = (wheelbase_ > 1e-6)
    ? current_speed * steering_slew_rad_s * servo_lag_s_ / wheelbase_
    : 0.0;
  const double yaw_rate_sigma = std::hypot(
    twist_cov_yaw_base_ + twist_cov_yaw_scale_ * std::fabs(current_angular_velocity),
    lag_sigma);

  odom.twist.covariance[0] = twist_cov_vx_;              ///< vx
  odom.twist.covariance[7] = twist_cov_vy_;              ///< vy
  odom.twist.covariance[35] = yaw_rate_sigma * yaw_rate_sigma;  ///< vyaw

  if (publish_tf_) {
    TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.header.stamp = now();
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;

    if (rclcpp::ok()) {
      tf_pub_->sendTransform(tf);
    }
  }

  if (rclcpp::ok()) {
    odom_pub_->publish(odom);
  }
}

void VescToOdom::servoCmdCallback(const Float64::SharedPtr servo)
{
  last_servo_cmd_ = servo;

  const double angle =
    (servo->data - steering_to_servo_offset_) / steering_to_servo_gain_;
  const rclcpp::Time stamp = now();

  if (have_servo_cmd_) {
    const double dt = (stamp - last_servo_time_).seconds();
    if (dt > 1e-3) {
      const double rate = std::fabs(angle - last_servo_angle_) / dt;
      // Smooth over roughly one servo time constant so a single sample does
      // not dominate the covariance estimate.
      const double alpha = std::min(1.0, dt / servo_lag_s_);
      servo_rate_rad_s_ += alpha * (rate - servo_rate_rad_s_);
    }
  }

  last_servo_angle_ = angle;
  last_servo_time_ = stamp;
  have_servo_cmd_ = true;
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)
