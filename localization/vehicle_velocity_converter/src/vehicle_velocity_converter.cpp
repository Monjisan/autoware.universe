// Copyright 2021 TierIV
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "vehicle_velocity_converter/vehicle_velocity_converter.hpp"

VehicleVelocityConverter::VehicleVelocityConverter() : TildeNode("vehicle_velocity_converter")
{
  // set covariance value for twist with covariance msg
  std::vector<double> covariance = declare_parameter<std::vector<double>>("twist_covariance");
  for (std::size_t i = 0; i < covariance.size(); ++i) {
    twist_covariance_[i] = covariance[i];
  }
  frame_id_ = declare_parameter("frame_id", "base_link");

  vehicle_report_sub_ = create_tilde_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "velocity_status", rclcpp::QoS{100},
    std::bind(&VehicleVelocityConverter::callbackVelocityReport, this, std::placeholders::_1));

  twist_with_covariance_pub_ = create_tilde_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance", rclcpp::QoS{10});
}

void VehicleVelocityConverter::callbackVelocityReport(
  const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
{
  if (msg->header.frame_id != frame_id_) {
    RCLCPP_WARN(get_logger(), "frame_id is not base_link.");
  }

  // set twist with covariance msg from vehicle report msg
  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance_msg;
  twist_with_covariance_msg.header = msg->header;
  twist_with_covariance_msg.twist.twist.linear.x = msg->longitudinal_velocity;
  twist_with_covariance_msg.twist.twist.linear.y = msg->lateral_velocity;
  twist_with_covariance_msg.twist.twist.angular.z = msg->heading_rate;
  twist_with_covariance_msg.twist.covariance = twist_covariance_;

  twist_with_covariance_pub_->publish(twist_with_covariance_msg);
}
