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

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <boost/optional.hpp>

#include <vector>

using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_perception_msgs::msg::Shape;

struct TargetObstacle
{
  TargetObstacle(
    const rclcpp::Time & arg_time_stamp, const PredictedObject & object,
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
  ObjectClassification classification;
  Shape shape;
  std::vector<PredictedPath> predicted_paths;
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
    const double arg_min_jerk, const double arg_min_strong_accel, const double arg_idling_time,
    const double arg_min_object_accel, const double arg_safe_distance_margin)
  : max_accel(arg_max_accel),
    min_accel(arg_min_accel),
    max_jerk(arg_max_jerk),
    min_jerk(arg_min_jerk),
    min_strong_accel(arg_min_strong_accel),
    idling_time(arg_idling_time),
    min_object_accel(arg_min_object_accel),
    safe_distance_margin(arg_safe_distance_margin)
  {
  }
  double max_accel;
  double min_accel;
  double max_jerk;
  double min_jerk;
  double min_strong_accel;
  double idling_time;
  double min_object_accel;
  double safe_distance_margin;
};

struct DebugData
{
  std::vector<PredictedObject> intentionally_ignored_obstacles;
  std::vector<TargetObstacle> obstacles_to_stop;
  std::vector<TargetObstacle> obstacles_to_cruise;
  visualization_msgs::msg::MarkerArray stop_wall_marker;
  visualization_msgs::msg::MarkerArray cruise_wall_marker;
  std::vector<tier4_autoware_utils::Polygon2d> detection_polygons;
  std::vector<geometry_msgs::msg::Point> collision_points;
};

#endif  // OBSTACLE_VELOCITY_PLANNER__COMMON_STRUCTS_HPP_