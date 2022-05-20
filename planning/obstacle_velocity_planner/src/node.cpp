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

#include "obstacle_velocity_planner/node.hpp"

#include "obstacle_velocity_planner/polygon_utils.hpp"
#include "obstacle_velocity_planner/utils.hpp"

#include <tier4_autoware_utils/ros/update_param.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <tier4_autoware_utils/trajectory/tmp_conversion.hpp>

#include <boost/format.hpp>

#include <algorithm>
#include <chrono>

/*
geometry_msgs::msg::Point operator+(
  const geometry_msgs::msg::Point & front_pos, const geometry_msgs::msg::Point & back_pos)
{
  geometry_msgs::msg::Point added_pos;
  added_pos.x = front_pos.x + back_pos.x;
  added_pos.y = front_pos.y + back_pos.y;
  added_pos.z = front_pos.z + back_pos.z;
  return added_pos;
}

geometry_msgs::msg::Point operator*(
  const geometry_msgs::msg::Point & pos, const double val)
{
  geometry_msgs::msg::Point added_pos;
  added_pos.x = pos.x * val;
  added_pos.y = pos.y * val;
  added_pos.z = pos.z * val;
  return added_pos;
}

geometry_msgs::msg::Point lerpPoint(const geometry_msgs::msg::Point & front_pos, const
geometry_msgs::msg::Point & back_pos, const double ratio)
{
  return front_pos * ratio + back_pos * (1 - ratio);
}

template <class T>
geometry_msgs::msg::Point calcInterpolatedPointWithLongitudinalOffset(const std::vector<T> & points,
const double longitudinal_offset, const boost::optional<size_t> start_idx)
{
  if (points.empty()) {
    throw std::logic_error("points is empty.");
  }

  if (start_idx) {
    if (start_idx.get() < 0 || points.size() <= start_idx.get()) {
      throw std::out_of_range("start_idx is out of range.");
    }
  } else {
    if (longitudinal_offset > 0) {
      start_idx = 0;
    } else {
      start_idx = points.size() - 1;
    }
  }

  double sum_length = 0.0;
  if (longitudinal_offset > 0) {
    for (size_t i = start_idx; i < points.size() - 1; ++i) {
      const double segment_length = tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i +
1)); sum_length += segment_length; if (sum_length >= longitudinal_offset) { const double
front_length = segment_length - (sum_length - longitudinal_offset); return
lerpPoint(tier4_autoware_utils::getPoint(points.at(i)),tier4_autoware_utils::getPoint(points.at(i +
1)), front_length / segment_length);
      }
    }
    return tier4_autoware_utils::getPoint(points.back());
  }

  for (size_t i = start_idx; i > 0; --i) {
    const double segment_length = tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i +
1)); sum_length += segment_length; if (sum_length >= -longitudinal_offset) { const double
front_length = segment_length - (sum_length + longitudinal_offset); return
lerpPoint(tier4_autoware_utils::getPoint(points.at(i)),tier4_autoware_utils::getPoint(points.at(i +
1)), front_length / segment_length);
    }
  }
  return tier4_autoware_utils::getPoint(points.front());
}
*/

namespace
{
VelocityLimitClearCommand createVelocityLimitClearCommandMsg(const rclcpp::Time & current_time)
{
  VelocityLimitClearCommand msg;
  msg.stamp = current_time;
  msg.sender = "obstacle_velocity_planner";
  msg.command = true;
  return msg;
}

Trajectory trimTrajectoryFrom(const Trajectory & input, const geometry_msgs::msg::Point & position)
{
  Trajectory output{};

  double min_distance = 0.0;
  size_t min_distance_index = 0;
  bool is_init = false;
  for (size_t i = 0; i < input.points.size(); ++i) {
    const double x = input.points.at(i).pose.position.x - position.x;
    const double y = input.points.at(i).pose.position.y - position.y;
    const double squared_distance = x * x + y * y;
    if (!is_init || squared_distance < min_distance * min_distance) {
      is_init = true;
      min_distance = std::sqrt(squared_distance);
      min_distance_index = i;
    }
  }
  for (size_t i = min_distance_index; i < input.points.size(); ++i) {
    output.points.push_back(input.points.at(i));
  }

  return output;
}

TrajectoryPoint getBackwardPointFromBasePoint(
  const TrajectoryPoint & p_from, const TrajectoryPoint & p_to, const TrajectoryPoint & p_base,
  const double backward_length)
{
  TrajectoryPoint output;
  const double dx = p_to.pose.position.x - p_from.pose.position.x;
  const double dy = p_to.pose.position.y - p_from.pose.position.y;
  const double norm = std::hypot(dx, dy);

  output = p_base;
  output.pose.position.x += backward_length * dx / norm;
  output.pose.position.y += backward_length * dy / norm;

  return output;
}

// TODO(murooka) replace with spline interpolation
Trajectory decimateTrajectory(const Trajectory & input, const double step_length)
{
  Trajectory output{};

  if (input.points.empty()) {
    return output;
  }

  double trajectory_length_sum = 0.0;
  double next_length = 0.0;

  for (int i = 0; i < static_cast<int>(input.points.size()) - 1; ++i) {
    const auto & p_front = input.points.at(i);
    const auto & p_back = input.points.at(i + 1);
    constexpr double epsilon = 1e-3;

    if (next_length <= trajectory_length_sum + epsilon) {
      const auto p_interpolate =
        getBackwardPointFromBasePoint(p_front, p_back, p_back, next_length - trajectory_length_sum);
      output.points.push_back(p_interpolate);
      next_length += step_length;
      continue;
    }

    trajectory_length_sum += tier4_autoware_utils::calcDistance2d(p_front, p_back);
  }

  output.points.push_back(input.points.back());

  return output;
}

PredictedPath getHighestConfidencePredictedPath(const std::vector<PredictedPath> & predicted_paths)
{
  const auto reliable_path = std::max_element(
    predicted_paths.begin(), predicted_paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });
  return *reliable_path;
}

bool isAngleAlignedWithTrajectory(
  const Trajectory & traj, const geometry_msgs::msg::Pose & pose, const double threshold_angle)
{
  if (traj.points.empty()) {
    return false;
  }

  const double obj_yaw = tf2::getYaw(pose.orientation);

  const size_t nearest_idx = tier4_autoware_utils::findNearestIndex(traj.points, pose.position);
  const double traj_yaw = tf2::getYaw(traj.points.at(nearest_idx).pose.orientation);

  const double diff_yaw = tier4_autoware_utils::normalizeRadian(obj_yaw - traj_yaw);

  const bool is_aligned = std::abs(diff_yaw) <= threshold_angle;
  return is_aligned;
}
}  // namespace

namespace motion_planning
{
ObstacleVelocityPlannerNode::ObstacleVelocityPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("obstacle_velocity_planner", node_options), self_pose_listener_(this)
{
  using std::placeholders::_1;

  // Subscriber
  trajectory_sub_ = create_subscription<Trajectory>(
    "~/input/trajectory", rclcpp::QoS{1},
    std::bind(&ObstacleVelocityPlannerNode::trajectoryCallback, this, _1));
  smoothed_trajectory_sub_ = create_subscription<Trajectory>(
    "/planning/scenario_planning/trajectory", rclcpp::QoS{1},
    std::bind(&ObstacleVelocityPlannerNode::smoothedTrajectoryCallback, this, _1));
  objects_sub_ = create_subscription<PredictedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&ObstacleVelocityPlannerNode::objectsCallback, this, _1));
  odom_sub_ = create_subscription<Odometry>(
    "~/input/odometry", rclcpp::QoS{1},
    std::bind(&ObstacleVelocityPlannerNode::odomCallback, this, std::placeholders::_1));
  sub_map_ = this->create_subscription<HADMapBin>(
    "~/input/map", rclcpp::QoS{1}.transient_local(),
    std::bind(&ObstacleVelocityPlannerNode::mapCallback, this, std::placeholders::_1));

  // Publisher
  trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  vel_limit_pub_ = create_publisher<VelocityLimit>("~/output/velocity_limit", 1);
  clear_vel_limit_pub_ =
    create_publisher<VelocityLimitClearCommand>("~/output/clear_velocity_limit", 1);
  // debug_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/marker",
  // 1);
  debug_calculation_time_pub_ = create_publisher<Float32Stamped>("~/debug/calculation_time", 1);
  debug_obstacles_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/obstacles_marker", 1);
  debug_slow_down_wall_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/slow_down_wall_marker", 1);
  debug_stop_wall_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/stop_wall_marker", 1);

  // Obstacle
  in_objects_ptr_ = std::make_unique<PredictedObjects>();

  // Vehicle Parameters
  vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();

  // longitudinal_info
  const double max_accel = declare_parameter<double>("common.max_accel");
  const double min_accel = declare_parameter<double>("common.min_accel");
  const double max_jerk = declare_parameter<double>("common.max_jerk");
  const double min_jerk = declare_parameter<double>("common.min_jerk");
  const double min_object_accel = declare_parameter<double>("common.min_object_accel");
  const double idling_time = declare_parameter<double>("common.idling_time");
  const auto longitudinal_info =
    LongitudinalInfo(max_accel, min_accel, max_jerk, min_jerk, min_object_accel, idling_time);

  // Obstacle filtering parameters
  obstacle_filtering_param_.rough_detection_area_expand_width =
    declare_parameter<double>("obstacle_filtering.rough_detection_area_expand_width");
  obstacle_filtering_param_.detection_area_expand_width =
    declare_parameter<double>("obstacle_filtering.detection_area_expand_width");
  obstacle_filtering_param_.decimate_step_length =
    declare_parameter<double>("obstacle_filtering.decimate_step_length");
  obstacle_filtering_param_.min_obstacle_crossing_velocity =
    declare_parameter<double>("obstacle_filtering.min_obstacle_crossing_velocity");
  obstacle_filtering_param_.margin_for_collision_time =
    declare_parameter<double>("obstacle_filtering.margin_for_collision_time");
  obstacle_filtering_param_.max_ego_obj_overlap_time =
    declare_parameter<double>("obstacle_filtering.max_ego_obj_overlap_time");
  obstacle_filtering_param_.max_prediction_time_for_collision_check =
    declare_parameter<double>("obstacle_filtering.max_prediction_time_for_collision_check");
  obstacle_filtering_param_.crossing_obstacle_traj_angle_threshold =
    declare_parameter<double>("obstacle_filtering.crossing_obstacle_traj_angle_threshold");

  // Wait for first self pose
  self_pose_listener_.waitForFirstPose();

  // planning method
  const std::string planning_method_param =
    declare_parameter<std::string>("common.planning_method");
  planning_method_ = getPlanningMethodType(planning_method_param);

  if (planning_method_ == PlanningMethod::OPTIMIZATION_BASE) {
    planner_ptr_ =
      std::make_unique<OptimizationBasedPlanner>(*this, longitudinal_info, vehicle_info_);
  } else if (planning_method_ == PlanningMethod::RULE_BASE) {
    planner_ptr_ = std::make_unique<RuleBasedPlanner>(*this, longitudinal_info, vehicle_info_);
  } else {
    std::logic_error("Designated method is not supported.");
  }

  // set parameter callback
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ObstacleVelocityPlannerNode::paramCallback, this, std::placeholders::_1));

  lpf_acc_ = std::make_shared<LowpassFilter1d>(0.0, 0.2);
}

ObstacleVelocityPlannerNode::PlanningMethod ObstacleVelocityPlannerNode::getPlanningMethodType(
  const std::string & param) const
{
  if (param == "rule_base") {
    return PlanningMethod::RULE_BASE;
  } else if (param == "optimization_base") {
    return PlanningMethod::OPTIMIZATION_BASE;
  }
  return PlanningMethod::INVALID;
}

rcl_interfaces::msg::SetParametersResult ObstacleVelocityPlannerNode::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  planner_ptr_->updateCommonParam(parameters);
  planner_ptr_->updateParam(parameters);

  // obstacle_filtering
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.rough_detection_area_expand_width",
    obstacle_filtering_param_.rough_detection_area_expand_width);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.detection_area_expand_width",
    obstacle_filtering_param_.detection_area_expand_width);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.decimate_step_length",
    obstacle_filtering_param_.decimate_step_length);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.min_obstacle_crossing_velocity",
    obstacle_filtering_param_.min_obstacle_crossing_velocity);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.margin_for_collision_time",
    obstacle_filtering_param_.margin_for_collision_time);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.max_ego_obj_overlap_time",
    obstacle_filtering_param_.max_ego_obj_overlap_time);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.max_prediction_time_for_collision_check",
    obstacle_filtering_param_.max_prediction_time_for_collision_check);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.crossing_obstacle_traj_angle_threshold",
    obstacle_filtering_param_.crossing_obstacle_traj_angle_threshold);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void ObstacleVelocityPlannerNode::mapCallback(const HADMapBin::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "[Obstacle Velocity Planner]: Start loading lanelet");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);

  auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr;

  RCLCPP_INFO(get_logger(), "[Obstacle Velocity Planner]: Start loading lanelet");
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr, &traffic_rules_ptr, &routing_graph_ptr);

  if (planner_ptr_) {
    planner_ptr_->setMaps(lanelet_map_ptr, traffic_rules_ptr, routing_graph_ptr);
    RCLCPP_INFO(get_logger(), "[Obstacle Velocity Planner]: Map is loaded");
  }
}

void ObstacleVelocityPlannerNode::objectsCallback(const PredictedObjects::SharedPtr msg)
{
  in_objects_ptr_ = msg;
}

void ObstacleVelocityPlannerNode::odomCallback(const Odometry::SharedPtr msg)
{
  if (current_twist_ptr_) {
    prev_twist_ptr_ = current_twist_ptr_;
  }

  current_twist_ptr_ = std::make_unique<geometry_msgs::msg::TwistStamped>();
  current_twist_ptr_->header = msg->header;
  current_twist_ptr_->twist = msg->twist.twist;
}

void ObstacleVelocityPlannerNode::smoothedTrajectoryCallback(const Trajectory::SharedPtr msg)
{
  planner_ptr_->setSmoothedTrajectory(msg);
}

void ObstacleVelocityPlannerNode::trajectoryCallback(const Trajectory::SharedPtr msg)
{
  tier4_autoware_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch;
  stop_watch.tic();

  std::lock_guard<std::mutex> lock(mutex_);

  // Subscribed variables are not ready
  if (msg->points.empty() || !current_twist_ptr_ || !prev_twist_ptr_ || !in_objects_ptr_) {
    return;
  }

  // convert obstacle type
  const auto obstacles = TargetObstacle::convertToTargetObstacles(*in_objects_ptr_);

  // Prepare algorithmic data
  ObstacleVelocityPlannerData planner_data;
  planner_data.current_time = get_clock()->now();
  planner_data.traj = *msg;
  planner_data.current_pose = self_pose_listener_.getCurrentPose()->pose;
  planner_data.current_vel = current_twist_ptr_->twist.linear.x;
  planner_data.current_acc = calcCurrentAccel();
  planner_data.target_obstacles = filterObstacles(
    obstacles, planner_data.traj, planner_data.current_pose, planner_data.current_vel);

  DebugData debug_data;

  // generate Trajectory
  boost::optional<VelocityLimit> vel_limit;
  const auto output_traj = planner_ptr_->generateTrajectory(planner_data, vel_limit, debug_data);

  // publisher external velocity limit if required
  publishVelocityLimit(vel_limit);

  // Publish trajectory
  trajectory_pub_->publish(output_traj);

  {  // publish calculation_time
    const double calculation_time = stop_watch.toc();
    Float32Stamped calculation_time_msg;
    calculation_time_msg.stamp = planner_data.current_time;
    calculation_time_msg.data = calculation_time;
    debug_calculation_time_pub_->publish(calculation_time_msg);
  }

  // publish debug data
  publishDebugData(debug_data);

  // RCLCPP_WARN_STREAM(get_logger(), elapsed_time);
}

void ObstacleVelocityPlannerNode::publishVelocityLimit(
  const boost::optional<VelocityLimit> & vel_limit)
{
  if (vel_limit) {
    vel_limit_pub_->publish(vel_limit.get());
    need_to_clear_vel_limit_ = true;
  } else {
    if (need_to_clear_vel_limit_) {
      const auto clear_vel_limit_msg = createVelocityLimitClearCommandMsg(get_clock()->now());
      clear_vel_limit_pub_->publish(clear_vel_limit_msg);
      need_to_clear_vel_limit_ = false;
    }
  }
}

double ObstacleVelocityPlannerNode::calcCurrentAccel() const
{
  const double diff_vel = current_twist_ptr_->twist.linear.x - prev_twist_ptr_->twist.linear.x;
  const double diff_time = std::max(
    (rclcpp::Time(current_twist_ptr_->header.stamp) - rclcpp::Time(prev_twist_ptr_->header.stamp))
      .seconds(),
    1e-03);

  const double accel = diff_vel / diff_time;

  return lpf_acc_->filter(accel);
}

std::vector<TargetObstacle> ObstacleVelocityPlannerNode::filterObstacles(
  const std::vector<TargetObstacle> & obstacles, const Trajectory & traj,
  const geometry_msgs::msg::Pose & current_pose, const double current_vel)
{
  const auto trimmed_traj = trimTrajectoryFrom(traj, current_pose.position);
  const auto decimated_traj =
    decimateTrajectory(trimmed_traj, obstacle_filtering_param_.decimate_step_length);
  if (decimated_traj.points.size() < 2) {
    return {};
  }

  const auto decimated_traj_polygons =
    polygon_utils::createOneStepPolygons(decimated_traj, vehicle_info_);

  const auto surrounding_lanelets = getSurroundingLanelets(current_pose);

  std::vector<TargetObstacle> target_obstacles;
  for (const auto & obstacle : obstacles) {
    const auto predicted_path_with_highest_confidence =
      getHighestConfidencePredictedPath(obstacle.predicted_paths);
    if (!checkOnMapObject(obstacle.pose, surrounding_lanelets)) {
      continue;
    }

    // rough area filtering
    const double dist_from_obstacle_to_traj = [&]() {
      return tier4_autoware_utils::calcLateralOffset(decimated_traj.points, obstacle.pose.position);
    }();
    if (dist_from_obstacle_to_traj > obstacle_filtering_param_.rough_detection_area_expand_width) {
      // obstacle is farm from the trajectory
      RCLCPP_INFO_EXPRESSION(
        get_logger(), true, "Ignore obstacles since it is far from the trajectory.");
      continue;
    }

    // precise area filtering
    const auto first_within_idx = polygon_utils::getFirstCollisionIndex(
      decimated_traj_polygons,
      polygon_utils::convertObstacleToPolygon(obstacle.pose, obstacle.shape),
      obstacle_filtering_param_.detection_area_expand_width);

    if (first_within_idx) {  // obsacles inside the trajectory
      const bool is_target_obstacle =
        planner_ptr_->isStopObstacle(obstacle.classification.label) ||
        planner_ptr_->isSlowDownObstacle(obstacle.classification.label);
      const bool is_angle_aligned = isAngleAlignedWithTrajectory(
        decimated_traj, obstacle.pose,
        obstacle_filtering_param_.crossing_obstacle_traj_angle_threshold);
      const double has_high_speed =
        std::abs(obstacle.velocity) > obstacle_filtering_param_.min_obstacle_crossing_velocity;
      // running vehicle crossing the ego trajectory
      if (is_target_obstacle && !is_angle_aligned && has_high_speed) {
        const double time_to_collision = [&]() {
          const double dist_from_ego_to_obstacle =
            tier4_autoware_utils::calcSignedArcLength(
              decimated_traj.points, current_pose.position, first_within_idx.get()) -
            vehicle_info_.max_longitudinal_offset_m;
          return dist_from_ego_to_obstacle / std::max(1e-6, current_vel);
        }();

        const double time_to_obstacle_getting_out = [&]() {
          const auto obstacle_getting_out_idx = polygon_utils::getFirstNonCollisionIndex(
            decimated_traj_polygons, predicted_path_with_highest_confidence, obstacle.shape,
            first_within_idx.get(), obstacle_filtering_param_.detection_area_expand_width);
          if (!obstacle_getting_out_idx) {
            return std::numeric_limits<double>::max();
          }

          const double dist_to_obstacle_getting_out = tier4_autoware_utils::calcSignedArcLength(
            decimated_traj.points, obstacle.pose.position, obstacle_getting_out_idx.get());

          return dist_to_obstacle_getting_out / obstacle.velocity;
        }();

        if (
          time_to_collision >
          time_to_obstacle_getting_out + obstacle_filtering_param_.margin_for_collision_time) {
          // False Condition 1. Ignore vehicle obstacles inside the trajectory, which is running
          // and does not collide with ego in a certain time.
          RCLCPP_INFO_EXPRESSION(
            get_logger(), true, "Ignore inside obstacles since it will not collide with the ego.");
          continue;
        }
      }
    } else {  // obstacles outside the trajectory
      const double max_dist =
        3.0;  // std::max(vehicle_info_.max_longitudinal_offset_m, vehicle_info_.rear_overhang) +
              // std::max(shape.dimensions.x, shape.dimensions.y) / 2.0;
      const bool will_collide = polygon_utils::willCollideWithSurroundObstacle(
        decimated_traj, decimated_traj_polygons, predicted_path_with_highest_confidence,
        obstacle.shape, obstacle_filtering_param_.detection_area_expand_width, max_dist,
        obstacle_filtering_param_.max_ego_obj_overlap_time,
        obstacle_filtering_param_.max_prediction_time_for_collision_check);
      if (!will_collide) {
        // False Condition 2. Ignore vehicle obstacles outside the trajectory, whose predicted path
        // overlaps the ego trajectory in a certain time.
        RCLCPP_INFO_EXPRESSION(
          get_logger(), true, "Ignore outside obstacles since it will not collide with the ego.");
        continue;
      }
    }

    target_obstacles.push_back(obstacle);
  }

  return target_obstacles;
}

// following code is map-based obstacle filtering
bool ObstacleVelocityPlannerNode::checkOnMapObject(
  const geometry_msgs::msg::Pose & obstacle_pose, const lanelet::ConstLanelets & valid_lanelets)
{
  // If we do not have a map, return true
  if (!lanelet_map_ptr_) {
    return true;
  }

  // obstacle point
  lanelet::BasicPoint2d search_point(obstacle_pose.position.x, obstacle_pose.position.y);

  // nearest lanelet
  std::vector<std::pair<double, lanelet::Lanelet>> surrounding_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 10);

  // No Closest Lanelets
  if (surrounding_lanelets.empty()) {
    return false;
  }

  // Check if the vehicle is inside the lanelet
  bool has_find_close_lanelet = false;
  for (const auto & lanelet : surrounding_lanelets) {
    if (lanelet::geometry::inside(lanelet.second, search_point)) {
      has_find_close_lanelet = true;
      // return true;
      break;
    }
  }

  if (!has_find_close_lanelet) {
    // This object is out of the any lanelets in this map
    return false;
  }

  for (const auto & valid_lanelet : valid_lanelets) {
    if (lanelet::geometry::inside(valid_lanelet, search_point)) {
      return true;
    }
  }

  return false;
}

lanelet::ConstLanelets ObstacleVelocityPlannerNode::getSurroundingLanelets(
  const geometry_msgs::msg::Pose & current_pose)
{
  // If we do not have a map, return true
  if (!lanelet_map_ptr_) {
    return {};
  }

  const lanelet::BasicPoint2d search_point(current_pose.position.x, current_pose.position.y);

  // nearest lanelet
  const std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 10);

  // Get Current Lanelets
  lanelet::Lanelets current_lanelets;
  for (const auto & lanelet : nearest_lanelets) {
    if (lanelet::geometry::inside(lanelet.second, search_point)) {
      current_lanelets.push_back(lanelet.second);
    }
  }

  lanelet::ConstLanelets surrounding_lanelets;

  const double initial_search_dist = 200.0;
  const double delta_search_dist = 50.0;
  for (const auto & current_lanelet : current_lanelets) {
    // Step1.1 Get the left lanelet
    lanelet::routing::LaneletPaths left_paths;
    auto opt_left = routing_graph_ptr_->left(current_lanelet);
    if (!!opt_left) {
      for (double search_dist = initial_search_dist; search_dist > 0;
           search_dist -= delta_search_dist) {
        const auto tmp_paths = routing_graph_ptr_->possiblePaths(*opt_left, search_dist, 0, false);
        addValidLanelet(tmp_paths, surrounding_lanelets);
      }
    }

    // Step1.2 Get the right lanelet
    lanelet::routing::LaneletPaths right_paths;
    auto opt_right = routing_graph_ptr_->right(current_lanelet);
    if (!!opt_right) {
      for (double search_dist = initial_search_dist; search_dist > 0;
           search_dist -= delta_search_dist) {
        const auto tmp_paths = routing_graph_ptr_->possiblePaths(*opt_right, search_dist, 0, false);
        addValidLanelet(tmp_paths, surrounding_lanelets);
      }
    }

    // Step1.3 Get the centerline
    lanelet::routing::LaneletPaths center_paths;
    for (double search_dist = initial_search_dist; search_dist > 0;
         search_dist -= delta_search_dist) {
      const auto tmp_paths =
        routing_graph_ptr_->possiblePaths(current_lanelet, search_dist, 0, false);
      addValidLanelet(tmp_paths, surrounding_lanelets);
    }
  }

  return surrounding_lanelets;
}

void ObstacleVelocityPlannerNode::addValidLanelet(
  const lanelet::routing::LaneletPaths & candidate_paths, lanelet::ConstLanelets & valid_lanelets)
{
  // Check if candidate paths are already in the valid paths
  for (const auto & candidate_path : candidate_paths) {
    for (const auto & candidate_lanelet : candidate_path) {
      const bool is_new_lanelet =
        std::find(valid_lanelets.begin(), valid_lanelets.end(), candidate_lanelet) ==
        valid_lanelets.end();
      if (is_new_lanelet) {
        valid_lanelets.push_back(candidate_lanelet);
      }
    }
  }
}
}  // namespace motion_planning
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motion_planning::ObstacleVelocityPlannerNode)
