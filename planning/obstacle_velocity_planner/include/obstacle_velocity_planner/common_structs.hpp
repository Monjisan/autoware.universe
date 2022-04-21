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

#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

#include <boost/optional.hpp>

#include <vector>

struct TargetObstacle
{
  static std::vector<TargetObstacle> convertToTargetObstacles(
    const autoware_auto_perception_msgs::msg::PredictedObjects & objects)
  {
    const auto time_stamp = rclcpp::Time(objects.header.stamp);

    std::vector<TargetObstacle> target_obstacles;
    for (const auto object : objects.objects) {
      target_obstacles.push_back(TargetObstacle(time_stamp, object));
    }
    return target_obstacles;
  }

  TargetObstacle(
    const rclcpp::Time & arg_time_stamp,
    const autoware_auto_perception_msgs::msg::PredictedObject & object)
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

struct LongitudinalParam
{
  LongitudinalParam(
    const double arg_max_accel, const double arg_min_accel, const double arg_max_jerk, const double arg_min_jerk,
    const double arg_min_limit_accel, const double arg_min_stop_accel,
    const double arg_safe_distance_margin)
  : max_accel(arg_max_accel),
    min_accel(arg_min_accel),
    max_jerk(arg_max_jerk),
    min_jerk(arg_min_jerk),
    min_limit_accel(arg_min_limit_accel),
    min_stop_accel(arg_min_stop_accel),
    safe_distance_margin(arg_safe_distance_margin)
  {
  }

  double max_accel;
  double min_accel;
  double max_jerk;
  double min_jerk;
  double min_limit_accel;
  double min_stop_accel;
  double safe_distance_margin;
};

struct RSSParam
{
  RSSParam(const double arg_min_ego_accel,
           const double arg_min_obstacle_accel,
           const double arg_idling_time)
    : min_ego_accel(arg_min_ego_accel),
      min_obstacle_accel(arg_min_obstacle_accel),
      idling_time(arg_idling_time)
  {}

  double min_ego_accel;
  double min_obstacle_accel;
  double idling_time;
};

#endif  // OBSTACLE_VELOCITY_PLANNER__COMMON_STRUCTS_HPP_
