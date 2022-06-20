// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef TRAJECTORY_FOLLOWER_NODES__CONTROLLER_NODE_HPP_
#define TRAJECTORY_FOLLOWER_NODES__CONTROLLER_NODE_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "motion_common/motion_common.hpp"
#include "motion_common/trajectory_common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_follower/debug_values.hpp"
#include "trajectory_follower/lateral_controller_base.hpp"
#include "trajectory_follower/longitudinal_controller_base.hpp"
#include "trajectory_follower/longitudinal_controller_utils.hpp"
#include "trajectory_follower/lowpass_filter.hpp"
#include "trajectory_follower/pid.hpp"
#include "trajectory_follower/smooth_stop.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_control_msgs/msg/longitudinal_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_system_msgs/msg/float32_multi_array_diagnostic.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower_nodes
{
using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;
namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;
namespace motion_common = ::autoware::motion::motion_common;

/// \classController
/// \brief The node class used for generating longitudinal control commands (velocity/acceleration)
class TRAJECTORY_FOLLOWER_PUBLIC Controller : public rclcpp::Node
{
public:
  explicit Controller(const rclcpp::NodeOptions & node_options);

private:
  // ros variables
  rclcpp::TimerBase::SharedPtr timer_control_;

  trajectory_follower::InputData input_data_;

  std::shared_ptr<trajectory_follower::LongitudinalControllerBase> longitudinal_controller_;
  std::shared_ptr<trajectory_follower::LateralControllerBase> lateral_controller_;

  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr sub_ref_path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr sub_steering_;
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_cmd_pub_;

  /**
   * @brief compute control command, and publish periodically
   */
  void callbackTimerControl();
  void onTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr);
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onSteering(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg);
};
}  // namespace trajectory_follower_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER_NODES__CONTROLLER_NODE_HPP_