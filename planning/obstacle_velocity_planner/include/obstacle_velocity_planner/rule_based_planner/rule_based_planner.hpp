// Copyright 2022 Tier IV, Inc.
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

#ifndef OBSTACLE_VELOCITY_PLANNER__RULE_BASED_PLANNER__RULE_BASED_PLANNER_HPP_
#define OBSTACLE_VELOCITY_PLANNER__RULE_BASED_PLANNER__RULE_BASED_PLANNER_HPP_

#include "obstacle_velocity_planner/planner_interface.hpp"
#include "obstacle_velocity_planner/rule_based_planner/debug_values.hpp"
#include "obstacle_velocity_planner/rule_based_planner/pid_controller.hpp"

#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "tier4_planning_msgs/msg/stop_reason_array.hpp"
#include "tier4_planning_msgs/msg/stop_speed_exceeded.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <boost/optional.hpp>

#include <memory>
#include <vector>

using tier4_debug_msgs::msg::Float32MultiArrayStamped;
using tier4_planning_msgs::msg::StopSpeedExceeded;

class RuleBasedPlanner : public PlannerInterface
{
public:
  RuleBasedPlanner(
    rclcpp::Node & node, const LongitudinalInfo & longitudinal_info,
    const vehicle_info_util::VehicleInfo & vehicle_info);

  Trajectory generateTrajectory(
    const ObstacleVelocityPlannerData & planner_data, boost::optional<VelocityLimit> & vel_limit,
    DebugData & debug_data) override;

  void updateParam(const std::vector<rclcpp::Parameter> & parameters) override;

private:
  struct SlowDownObstacleInfo
  {
    SlowDownObstacleInfo(
      const TargetObstacle & obstacle_arg, const double dist_to_slow_down_arg,
      const double normalized_dist_to_slow_down_arg)
    : obstacle(obstacle_arg),
      dist_to_slow_down(dist_to_slow_down_arg),
      normalized_dist_to_slow_down(normalized_dist_to_slow_down_arg)
    {
    }
    TargetObstacle obstacle;
    double dist_to_slow_down;
    double normalized_dist_to_slow_down;
  };

  struct StopObstacleInfo
  {
    StopObstacleInfo(const TargetObstacle & obstacle_arg, const double dist_to_stop_arg)
    : obstacle(obstacle_arg), dist_to_stop(dist_to_stop_arg)
    {
    }
    TargetObstacle obstacle;
    double dist_to_stop;
  };

  // ROS param
  double vel_to_acc_weight_;
  double min_slow_down_target_vel_;
  double max_slow_down_obstacle_velocity_to_stop_;
  double safe_distance_margin_;
  double min_strong_stop_accel_;

  // pid controller
  std::unique_ptr<PIDController> pid_controller_;
  double output_ratio_during_accel_;

  // Publisher
  rclcpp::Publisher<tier4_planning_msgs::msg::StopReasonArray>::SharedPtr stop_reasons_pub_;
  rclcpp::Publisher<StopSpeedExceeded>::SharedPtr stop_speed_exceeded_pub_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_values_pub_;

  boost::optional<double> prev_target_vel_;

  boost::optional<size_t> doStop(
    const ObstacleVelocityPlannerData & planner_data, const StopObstacleInfo & stop_obstacle_info,
    std::vector<TargetObstacle> & debug_obstacles_to_stop,
    visualization_msgs::msg::MarkerArray & debug_walls_marker) const;

  VelocityLimit doSlowDown(
    const ObstacleVelocityPlannerData & planner_data,
    const SlowDownObstacleInfo & slow_down_obstacle_info,
    std::vector<TargetObstacle> & debug_obstacles_to_slow_down,
    visualization_msgs::msg::MarkerArray & debug_walls_marker);

  DebugValues debug_values_;
};

#endif  // OBSTACLE_VELOCITY_PLANNER__RULE_BASED_PLANNER__RULE_BASED_PLANNER_HPP_
