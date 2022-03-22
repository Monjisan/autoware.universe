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

#ifndef OBSTACLE_VELOCITY_PLANNER__POLYGON_UTILS_HPP_
#define OBSTACLE_VELOCITY_PLANNER__POLYGON_UTILS_HPP_

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry.hpp>
#include <boost/optional.hpp>

#include <vector>

namespace polygon_utils
{
namespace bg = boost::geometry;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;

Polygon2d convertObstacleToPolygon(
  const geometry_msgs::msg::Pose & pose, const autoware_auto_perception_msgs::msg::Shape & shape);

void appendPointToPolygon(Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point);
void appendPointToPolygon(Polygon2d & polygon, const geometry_msgs::msg::Point32 & geom_point);

boost::optional<size_t> getFirstCollisionIndex(
  const std::vector<Polygon2d> & base_polygons, const Polygon2d & target_polygon,
  const double margin = 0.0);

boost::optional<size_t> getFirstNonCollisionIndex(
  const std::vector<Polygon2d> & base_polygons,
  const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const autoware_auto_perception_msgs::msg::Shape & shape, const size_t start_idx,
  const double margin);

bool willCollideWithSurroundObstacle(
  const autoware_auto_planning_msgs::msg::Trajectory & traj,
  const std::vector<Polygon2d> & traj_polygons,
  const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const autoware_auto_perception_msgs::msg::Shape & shape, const double dist_margin,
  const double max_dist, const double max_ego_obj_overlap_time,
  const double max_prediction_time_for_collision_check);

std::vector<Polygon2d> createOneStepPolygons(
  const autoware_auto_planning_msgs::msg::Trajectory & traj,
  const vehicle_info_util::VehicleInfo & vehicle_info);

Polygon2d createOneStepPolygon(
  const geometry_msgs::msg::Pose & base_step_pose, const geometry_msgs::msg::Pose & next_step_pose,
  const vehicle_info_util::VehicleInfo & vehicle_info);
}  // namespace polygon_utils

#endif  // OBSTACLE_VELOCITY_PLANNER__POLYGON_UTILS_HPP_
