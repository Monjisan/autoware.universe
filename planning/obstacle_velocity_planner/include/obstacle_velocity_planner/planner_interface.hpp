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

#ifndef OBSTACLE_VELOCITY_PLANNER__PLANNER_INTERFACE_HPP_
#define OBSTACLE_VELOCITY_PLANNER__PLANNER_INTERFACE_HPP_

#include "obstacle_velocity_planner/common_structs.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"
#include "obstacle_velocity_planner/utils.hpp"

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "tier4_planning_msgs/msg/velocity_limit.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <boost/optional.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <vector>

using autoware_auto_planning_msgs::msg::Trajectory;
using tier4_planning_msgs::msg::VelocityLimit;

class PlannerInterface
{
public:
  PlannerInterface(
                   rclcpp::Node & node,
    const LongitudinalParam & longitudinal_param, const RSSParam & rss_param, const vehicle_info_util::VehicleInfo & vehicle_info)
  : longitudinal_param_(longitudinal_param), rss_param_(rss_param), vehicle_info_(vehicle_info)
  {
    debug_obstacles_marker_pub_ = node.create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/obstacles_marker", 1);
    debug_walls_marker_pub_ = node.create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/walls_marker", 1);
  }

  PlannerInterface() = default;

  virtual Trajectory generateTrajectory(
    const ObstacleVelocityPlannerData & planner_data,
    boost::optional<VelocityLimit> & vel_limit) = 0;

  void updateCommonParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    auto & li = longitudinal_param_;

    /*
    tier4_autoware_utils::updateParam<double>(parameters, "common.max_accel", li.max_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "common.min_accel", li.min_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "common.max_jerk", li.max_jerk);
    tier4_autoware_utils::updateParam<double>(parameters, "common.min_jerk", li.min_jerk);
    tier4_autoware_utils::updateParam<double>(
      parameters, "common.min_obstacle_accel", li.min_obstacle_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "common.idling_time", li.idling_time);
    */

    auto & ri = rss_param_;
  }

  virtual void updateParam([[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters) {}

  // TODO(shimizu) remove this function
  void setMaps(
    const std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr,
    const std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr,
    const std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr)
  {
    lanelet_map_ptr_ = lanelet_map_ptr;
    traffic_rules_ptr_ = traffic_rules_ptr;
    routing_graph_ptr_ = routing_graph_ptr;
  }

  // TODO(shimizu) remove this function
  void setSmoothedTrajectory(const Trajectory::SharedPtr traj) { smoothed_trajectory_ptr_ = traj; }

protected:
  // Parameters
  LongitudinalParam longitudinal_param_;
  RSSParam rss_param_;

  // Vehicle Parameters
  vehicle_info_util::VehicleInfo vehicle_info_;

  // TODO(shimizu) remove these parameters
  // Lanelet Map Pointers
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;
  Trajectory::SharedPtr smoothed_trajectory_ptr_;

  double calcRSSDistance(
    const double ego_vel, const double obj_vel, const double margin = 0.0) const
  {
    const auto & i = rss_param_;
    const double rss_dist_with_margin =
      ego_vel * i.idling_time + std::pow(ego_vel, 2) * 0.5 / std::abs(i.min_ego_accel) -
      std::pow(obj_vel, 2) * 0.5 / std::abs(i.min_obstacle_accel) + margin;
    return rss_dist_with_margin;
  }

  void publishTargetObstacles(const std::vector<TargetObstacle> & obstacles_to_stop,
                             const std::vector<TargetObstacle> & obstacles_to_slow_down) {
    // TODO(murooka) change shape of markers based on its shape
    visualization_msgs::msg::MarkerArray obstacles_marker;

    // obstacles to slow down
    for (size_t i = 0; i < obstacles_to_slow_down.size(); ++i) {
      const auto marker = obstacle_velocity_utils::getObjectMarkerArray(
                                                                        obstacles_to_slow_down.at(i).pose, i, "obstacles_to_slow_down", 0.7, 0.7, 0.0);
      tier4_autoware_utils::appendMarkerArray(marker, &obstacles_marker);
    }

    // obstacles to stop
    for (size_t i = 0; i < obstacles_to_stop.size(); ++i) {
      const auto marker = obstacle_velocity_utils::getObjectMarkerArray(
                                                                        obstacles_to_stop.at(i).pose, i, "obstacles_to_stop", 1.0, 0.0, 0.0);
      tier4_autoware_utils::appendMarkerArray(marker, &obstacles_marker);
    }

    debug_obstacles_marker_pub_->publish(obstacles_marker);
  }

  void publishWalls(const visualization_msgs::msg::MarkerArray & debug_walls_marker) {
    debug_walls_marker_pub_->publish(debug_walls_marker);
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_obstacles_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_walls_marker_pub_;
};

#endif  // OBSTACLE_VELOCITY_PLANNER__PLANNER_INTERFACE_HPP_
