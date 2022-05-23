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

#ifndef OBSTACLE_VELOCITY_PLANNER__COMMON_STRUCTS_HPP_
#define OBSTACLE_VELOCITY_PLANNER__COMMON_STRUCTS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/optional.hpp>

#include <vector>

struct TargetObstacle
{
  TargetObstacle(
    const rclcpp::Time & arg_time_stamp,
    const autoware_auto_perception_msgs::msg::PredictedObject & object,
    const geometry_msgs::msg::Point & arg_collision_point)
  {
    time_stamp = arg_time_stamp;
    orientation_reliable = true;
    pose = object.kinematics.initial_pose_with_covariance.pose;
    velocity_reliable = true;
    velocity = object.kinematics.initial_twist_with_covariance.twist.linear.x;
    is_classified = true;
    classification = object.classification.at(0);
    shape = object.shape;

    predicted_paths.clear();
    for (const auto & path : object.kinematics.predicted_paths) {
      predicted_paths.push_back(path);
    }

    collision_point = arg_collision_point;
  }

  rclcpp::Time time_stamp;
  bool orientation_reliable;
  geometry_msgs::msg::Pose pose;
  bool velocity_reliable;
  float velocity;
  bool is_classified;
  autoware_auto_perception_msgs::msg::ObjectClassification classification;
  autoware_auto_perception_msgs::msg::Shape shape;
  std::vector<autoware_auto_perception_msgs::msg::PredictedPath> predicted_paths;
  geometry_msgs::msg::Point collision_point;
};

struct ObstacleVelocityPlannerData
{
  rclcpp::Time current_time;
  autoware_auto_planning_msgs::msg::Trajectory traj;
  geometry_msgs::msg::Pose current_pose;
  double current_vel;
  double current_acc;
  std::vector<TargetObstacle> target_obstacles;
};

struct LongitudinalInfo
{
  LongitudinalInfo(
    const double arg_max_accel, const double arg_min_accel, const double arg_max_jerk,
    const double arg_min_jerk, const double arg_min_object_accel, const double arg_idling_time)
  : max_accel(arg_max_accel),
    min_accel(arg_min_accel),
    max_jerk(arg_max_jerk),
    min_jerk(arg_min_jerk),
    min_object_accel(arg_min_object_accel),
    idling_time(arg_idling_time)
  {
  }
  double max_accel;
  double min_accel;
  double max_jerk;
  double min_jerk;
  double min_object_accel;
  double idling_time;
};

struct LongitudinalMotion
{
  void reset()
  {
    vel = {};
    max_acc = {};
    min_acc = {};
    max_jerk = {};
    min_jerk = {};
  }

  boost::optional<double> vel;
  boost::optional<double> max_acc;
  boost::optional<double> min_acc;
  boost::optional<double> max_jerk;
  boost::optional<double> min_jerk;
};

struct DebugData
{
  std::vector<TargetObstacle> intentionally_ignored_obstacles;
  std::vector<TargetObstacle> obstacles_to_stop;
  std::vector<TargetObstacle> obstacles_to_slow_down;
  visualization_msgs::msg::MarkerArray stop_wall_marker;
  visualization_msgs::msg::MarkerArray slow_down_wall_marker;
  std::vector<tier4_autoware_utils::Polygon2d> detection_polygons;
  geometry_msgs::msg::Point collision_point;
};

#endif  // OBSTACLE_VELOCITY_PLANNER__COMMON_STRUCTS_HPP_
