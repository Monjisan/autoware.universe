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
#include "tier4_autoware_utils/ros/update_param.hpp"
#include "tier4_autoware_utils/trajectory/tmp_conversion.hpp"

#include <boost/format.hpp>

#include <algorithm>
#include <chrono>

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

TrajectoryPoint calcLinearPoint(
  const TrajectoryPoint & p_from, const TrajectoryPoint & p_to, const double length)
{
  TrajectoryPoint output;
  const double dx = p_to.pose.position.x - p_from.pose.position.x;
  const double dy = p_to.pose.position.y - p_from.pose.position.y;
  const double norm = std::hypot(dx, dy);

  output = p_to;
  output.pose.position.x += length * dx / norm;
  output.pose.position.y += length * dy / norm;

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
        calcLinearPoint(p_front, p_back, next_length - trajectory_length_sum);
      output.points.push_back(p_interpolate);
      next_length += step_length;
      continue;
    }

    trajectory_length_sum += tier4_autoware_utils::calcDistance2d(p_front, p_back);
  }

  output.points.push_back(input.points.back());

  return output;
}

PredictedPath getHighestConfidencePredictedPath(const PredictedObject & predicted_object)
{
  const auto reliable_path = std::max_element(
    predicted_object.kinematics.predicted_paths.begin(),
    predicted_object.kinematics.predicted_paths.end(),
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
: Node("obstacle_velocity_planner", node_options),
  self_pose_listener_(this),
  in_objects_ptr_(nullptr),
  lpf_acc_ptr_(nullptr),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
  using std::placeholders::_1;

  // subscriber
  trajectory_sub_ = create_subscription<Trajectory>(
    "~/input/trajectory", rclcpp::QoS{1},
    std::bind(&ObstacleVelocityPlannerNode::onTrajectory, this, _1));
  smoothed_trajectory_sub_ = create_subscription<Trajectory>(
    "/planning/scenario_planning/trajectory", rclcpp::QoS{1},
    std::bind(&ObstacleVelocityPlannerNode::onSmoothedTrajectory, this, _1));
  objects_sub_ = create_subscription<PredictedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&ObstacleVelocityPlannerNode::onObjects, this, _1));
  odom_sub_ = create_subscription<Odometry>(
    "~/input/odometry", rclcpp::QoS{1},
    std::bind(&ObstacleVelocityPlannerNode::onOdometry, this, std::placeholders::_1));

  // publisher
  trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  vel_limit_pub_ = create_publisher<VelocityLimit>("~/output/velocity_limit", 1);
  clear_vel_limit_pub_ =
    create_publisher<VelocityLimitClearCommand>("~/output/clear_velocity_limit", 1);
  debug_calculation_time_pub_ = create_publisher<Float32Stamped>("~/debug/calculation_time", 1);
  debug_cruise_wall_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/cruise_wall_marker", 1);
  debug_stop_wall_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/stop_wall_marker", 1);
  debug_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/marker", 1);

  // longitudinal_info
  const auto longitudinal_info = [&]() {
    const double max_accel = declare_parameter<double>("normal.max_acc");
    const double min_accel = declare_parameter<double>("normal.min_acc");
    const double max_jerk = declare_parameter<double>("normal.max_jerk");
    const double min_jerk = declare_parameter<double>("normal.min_jerk");

    const double min_strong_accel = declare_parameter<double>("common.min_strong_accel");
    const double min_object_accel = declare_parameter<double>("common.min_object_accel");
    const double idling_time = declare_parameter<double>("common.idling_time");
    const double safe_distance_margin = declare_parameter<double>("common.safe_distance_margin");

    return LongitudinalInfo(
      max_accel, min_accel, max_jerk, min_jerk, min_strong_accel, idling_time, min_object_accel,
      safe_distance_margin);
  }();

  const bool is_showing_debug_info_ = declare_parameter<bool>("common.is_showing_debug_info");

  // low pass filter for ego acceleration
  const double lpf_gain_for_accel = declare_parameter<double>("common.lpf_gain_for_accel");
  lpf_acc_ptr_ = std::make_shared<LowpassFilter1d>(0.0, lpf_gain_for_accel);

  {  // Obstacle filtering parameters
    obstacle_filtering_param_.rough_detection_area_expand_width =
      declare_parameter<double>("obstacle_filtering.rough_detection_area_expand_width");
    obstacle_filtering_param_.detection_area_expand_width =
      declare_parameter<double>("obstacle_filtering.detection_area_expand_width");
    obstacle_filtering_param_.decimate_trajectory_step_length =
      declare_parameter<double>("obstacle_filtering.decimate_trajectory_step_length");
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
  }

  {  // planning algorithm
    const std::string planning_algorithm_param =
      declare_parameter<std::string>("common.planning_algorithm");
    planning_algorithm_ = getPlanningAlgorithmType(planning_algorithm_param);

    if (planning_algorithm_ == PlanningAlgorithm::OPTIMIZATION_BASE) {
      planner_ptr_ =
        std::make_unique<OptimizationBasedPlanner>(*this, longitudinal_info, vehicle_info_);
    } else if (planning_algorithm_ == PlanningAlgorithm::RULE_BASE) {
      planner_ptr_ = std::make_unique<RuleBasedPlanner>(*this, longitudinal_info, vehicle_info_);
    } else {
      std::logic_error("Designated algorithm is not supported.");
    }

    min_behavior_stop_margin_ = declare_parameter<double>("common.min_behavior_stop_margin");
    max_nearest_dist_deviation_ = declare_parameter<double>("common.max_nearest_dist_deviation");
    max_nearest_yaw_deviation_ = declare_parameter<double>("common.max_nearest_yaw_deviation");
    planner_ptr_->setParams(
      is_showing_debug_info_, min_behavior_stop_margin_, max_nearest_dist_deviation_,
      max_nearest_yaw_deviation_);
  }

  // wait for first self pose
  self_pose_listener_.waitForFirstPose();

  // set parameter callback
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ObstacleVelocityPlannerNode::onParam, this, std::placeholders::_1));
}

ObstacleVelocityPlannerNode::PlanningAlgorithm
ObstacleVelocityPlannerNode::getPlanningAlgorithmType(const std::string & param) const
{
  if (param == "rule_base") {
    return PlanningAlgorithm::RULE_BASE;
  } else if (param == "optimization_base") {
    return PlanningAlgorithm::OPTIMIZATION_BASE;
  }
  return PlanningAlgorithm::INVALID;
}

rcl_interfaces::msg::SetParametersResult ObstacleVelocityPlannerNode::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  planner_ptr_->updateCommonParam(parameters);
  planner_ptr_->updateParam(parameters);

  tier4_autoware_utils::updateParam<bool>(
    parameters, "common.is_showing_debug_info", is_showing_debug_info_);
  planner_ptr_->setParams(
    is_showing_debug_info_, min_behavior_stop_margin_, max_nearest_dist_deviation_,
    max_nearest_yaw_deviation_);

  // obstacle_filtering
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.rough_detection_area_expand_width",
    obstacle_filtering_param_.rough_detection_area_expand_width);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.detection_area_expand_width",
    obstacle_filtering_param_.detection_area_expand_width);
  tier4_autoware_utils::updateParam<double>(
    parameters, "obstacle_filtering.decimate_trajectory_step_length",
    obstacle_filtering_param_.decimate_trajectory_step_length);
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

void ObstacleVelocityPlannerNode::onObjects(const PredictedObjects::SharedPtr msg)
{
  in_objects_ptr_ = msg;
}

void ObstacleVelocityPlannerNode::onOdometry(const Odometry::SharedPtr msg)
{
  if (current_twist_ptr_) {
    prev_twist_ptr_ = current_twist_ptr_;
  }

  current_twist_ptr_ = std::make_unique<geometry_msgs::msg::TwistStamped>();
  current_twist_ptr_->header = msg->header;
  current_twist_ptr_->twist = msg->twist.twist;
}

void ObstacleVelocityPlannerNode::onSmoothedTrajectory(const Trajectory::SharedPtr msg)
{
  planner_ptr_->setSmoothedTrajectory(msg);
}

void ObstacleVelocityPlannerNode::onTrajectory(const Trajectory::SharedPtr msg)
{
  // check if subscribed variables are ready
  if (msg->points.empty() || !current_twist_ptr_ || !prev_twist_ptr_ || !in_objects_ptr_) {
    return;
  }

  stop_watch_.tic(__func__);

  // create algorithmic data
  DebugData debug_data;
  const auto planner_data = createPlannerData(*msg, debug_data);

  // generate Trajectory
  boost::optional<VelocityLimit> vel_limit;
  const auto output_traj = planner_ptr_->generateTrajectory(planner_data, vel_limit, debug_data);

  // publisher external velocity limit if required
  publishVelocityLimit(vel_limit);

  // Publish trajectory
  trajectory_pub_->publish(output_traj);

  // publish debug data
  publishDebugData(debug_data);

  // publish and print calcualtion time
  const double calculation_time = stop_watch_.toc(__func__);
  publishCalculationTime(calculation_time);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleVelocityPlanner"), is_showing_debug_info_, "%s := %f [ms]",
    __func__, calculation_time);
}

ObstacleVelocityPlannerData ObstacleVelocityPlannerNode::createPlannerData(
  const Trajectory & trajectory, DebugData & debug_data)
{
  stop_watch_.tic(__func__);

  const auto & current_pose = self_pose_listener_.getCurrentPose()->pose;
  const double current_vel = current_twist_ptr_->twist.linear.x;
  const double current_accel = calcCurrentAccel();

  // create planner_data
  ObstacleVelocityPlannerData planner_data;
  planner_data.current_time = now();
  planner_data.traj = trajectory;
  planner_data.current_pose = current_pose;
  planner_data.current_vel = current_vel;
  planner_data.current_acc = current_accel;
  planner_data.target_obstacles =
    filterObstacles(*in_objects_ptr_, trajectory, current_pose, current_vel, debug_data);

  // print calculation time
  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleVelocityPlanner"), is_showing_debug_info_, "  %s := %f [ms]",
    __func__, calculation_time);

  return planner_data;
}

double ObstacleVelocityPlannerNode::calcCurrentAccel() const
{
  const double diff_vel = current_twist_ptr_->twist.linear.x - prev_twist_ptr_->twist.linear.x;
  const double diff_time = std::max(
    (rclcpp::Time(current_twist_ptr_->header.stamp) - rclcpp::Time(prev_twist_ptr_->header.stamp))
      .seconds(),
    1e-03);

  const double accel = diff_vel / diff_time;

  return lpf_acc_ptr_->filter(accel);
}

std::vector<TargetObstacle> ObstacleVelocityPlannerNode::filterObstacles(
  const PredictedObjects & predicted_objects, const Trajectory & traj,
  const geometry_msgs::msg::Pose & current_pose, const double current_vel, DebugData & debug_data)
{
  const auto time_stamp = rclcpp::Time(predicted_objects.header.stamp);

  // calcualte decimated trajectory
  const auto trimmed_traj = trimTrajectoryFrom(traj, current_pose.position);
  const auto decimated_traj =
    decimateTrajectory(trimmed_traj, obstacle_filtering_param_.decimate_trajectory_step_length);
  if (decimated_traj.points.size() < 2) {
    return {};
  }

  // calcualte decimated trajectory polygons
  const auto decimated_traj_polygons = polygon_utils::createOneStepPolygons(
    decimated_traj, vehicle_info_, obstacle_filtering_param_.detection_area_expand_width);
  debug_data.detection_polygons = decimated_traj_polygons;

  std::vector<TargetObstacle> target_obstacles;
  for (const auto & predicted_object : predicted_objects.objects) {
    // filter object whose label is not cruised or stopped
    const bool is_target_obstacle =
      planner_ptr_->isStopObstacle(predicted_object.classification.front().label) ||
      planner_ptr_->isCruiseObstacle(predicted_object.classification.front().label);
    if (!is_target_obstacle) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), is_showing_debug_info_, "Ignore obstacles since its label is not target.");
      continue;
    }

    const auto & object_pose = predicted_object.kinematics.initial_pose_with_covariance.pose;
    const auto & object_velocity =
      predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x;

    // rough detection area filtering without polygons
    const double dist_from_obstacle_to_traj = [&]() {
      return tier4_autoware_utils::calcLateralOffset(decimated_traj.points, object_pose.position);
    }();
    if (dist_from_obstacle_to_traj > obstacle_filtering_param_.rough_detection_area_expand_width) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), is_showing_debug_info_,
        "Ignore obstacles since it is far from the trajectory.");
      continue;
    }

    // calculate collision points
    const auto obstacle_polygon =
      polygon_utils::convertObstacleToPolygon(object_pose, predicted_object.shape);
    std::vector<geometry_msgs::msg::Point> collision_points;
    const auto first_within_idx = polygon_utils::getFirstCollisionIndex(
      decimated_traj_polygons, obstacle_polygon, collision_points);

    // precide detection area filtering with polygons
    geometry_msgs::msg::Point nearest_collision_point;
    if (first_within_idx) {  // obsacles inside the trajectory
      // calculate nearest collision point
      nearest_collision_point =
        calcNearestCollisionPoint(first_within_idx.get(), collision_points, decimated_traj);
      debug_data.collision_points.push_back(nearest_collision_point);

      const bool is_angle_aligned = isAngleAlignedWithTrajectory(
        decimated_traj, object_pose,
        obstacle_filtering_param_.crossing_obstacle_traj_angle_threshold);
      const double has_high_speed =
        std::abs(object_velocity) > obstacle_filtering_param_.min_obstacle_crossing_velocity;

      // ignore running vehicle crossing the ego trajectory with high speed with some condition
      if (!is_angle_aligned && has_high_speed) {
        const double collision_time_margin = calcCollisionTimeMargin(
          current_pose, current_vel, nearest_collision_point, predicted_object,
          first_within_idx.get(), decimated_traj, decimated_traj_polygons);
        if (collision_time_margin > obstacle_filtering_param_.margin_for_collision_time) {
          // Ignore condition 1
          // Ignore vehicle obstacles inside the trajectory, which is crossing the trajectory with
          // high speed and does not collide with ego in a certain time.
          RCLCPP_INFO_EXPRESSION(
            get_logger(), is_showing_debug_info_,
            "Ignore inside obstacles since it will not collide with the ego.");

          debug_data.intentionally_ignored_obstacles.push_back(predicted_object);
          continue;
        }
      }
    } else {  // obstacles outside the trajectory
      const double max_dist =
        3.0;  // std::max(vehicle_info_.max_longitudinal_offset_m, vehicle_info_.rear_overhang) +
      // std::max(shape.dimensions.x, shape.dimensions.y) / 2.0;
      const auto predicted_path_with_highest_confidence =
        getHighestConfidencePredictedPath(predicted_object);

      const bool will_collide = polygon_utils::willCollideWithSurroundObstacle(
        decimated_traj, decimated_traj_polygons, predicted_path_with_highest_confidence,
        predicted_object.shape, max_dist, obstacle_filtering_param_.max_ego_obj_overlap_time,
        obstacle_filtering_param_.max_prediction_time_for_collision_check);

      // TODO(murooka) think later
      nearest_collision_point = object_pose.position;

      if (!will_collide) {
        // Ignore condition 2
        // Ignore vehicle obstacles outside the trajectory, whose predicted path
        // overlaps the ego trajectory in a certain time.
        RCLCPP_INFO_EXPRESSION(
          get_logger(), is_showing_debug_info_,
          "Ignore outside obstacles since it will not collide with the ego.");

        debug_data.intentionally_ignored_obstacles.push_back(predicted_object);
        continue;
      }
    }

    // convert to obstacle type
    const auto target_obstacle =
      TargetObstacle(time_stamp, predicted_object, nearest_collision_point);
    target_obstacles.push_back(target_obstacle);
  }

  return target_obstacles;
}

geometry_msgs::msg::Point ObstacleVelocityPlannerNode::calcNearestCollisionPoint(
  const size_t & first_within_idx, const std::vector<geometry_msgs::msg::Point> & collision_points,
  const Trajectory & decimated_traj)
{
  std::array<geometry_msgs::msg::Point, 2> segment_points;
  if (first_within_idx == 0) {
    const auto & traj_front_pose = decimated_traj.points.at(0).pose;
    segment_points.at(0) = traj_front_pose.position;

    const auto front_pos = tier4_autoware_utils::calcOffsetPose(
                             traj_front_pose, vehicle_info_.max_longitudinal_offset_m, 0.0, 0.0)
                             .position;
    segment_points.at(1) = front_pos;
  } else {
    const size_t seg_idx = first_within_idx - 1;
    segment_points.at(0) = decimated_traj.points.at(seg_idx).pose.position;
    segment_points.at(1) = decimated_traj.points.at(seg_idx + 1).pose.position;
  }

  size_t min_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t cp_idx = 0; cp_idx < collision_points.size(); ++cp_idx) {
    const auto & collision_point = collision_points.at(cp_idx);
    const double dist =
      tier4_autoware_utils::calcLongitudinalOffsetToSegment(segment_points, 0, collision_point);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = cp_idx;
    }
  }

  return collision_points.at(min_idx);
}

double ObstacleVelocityPlannerNode::calcCollisionTimeMargin(
  const geometry_msgs::msg::Pose & current_pose, const double current_vel,
  const geometry_msgs::msg::Point & nearest_collision_point,
  const PredictedObject & predicted_object, const size_t first_within_idx,
  const Trajectory & decimated_traj,
  const std::vector<tier4_autoware_utils::Polygon2d> & decimated_traj_polygons)
{
  const auto & object_pose = predicted_object.kinematics.initial_pose_with_covariance.pose;
  const auto & object_velocity =
    predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x;
  const auto predicted_path_with_highest_confidence =
    getHighestConfidencePredictedPath(predicted_object);

  const double time_to_collision = [&]() {
    const double dist_from_ego_to_obstacle =
      tier4_autoware_utils::calcSignedArcLength(
        decimated_traj.points, current_pose.position, nearest_collision_point) -
      vehicle_info_.max_longitudinal_offset_m;
    return dist_from_ego_to_obstacle / std::max(1e-6, current_vel);
  }();

  const double time_to_obstacle_getting_out = [&]() {
    const auto obstacle_getting_out_idx = polygon_utils::getFirstNonCollisionIndex(
      decimated_traj_polygons, predicted_path_with_highest_confidence, predicted_object.shape,
      first_within_idx);
    if (!obstacle_getting_out_idx) {
      return std::numeric_limits<double>::max();
    }

    const double dist_to_obstacle_getting_out = tier4_autoware_utils::calcSignedArcLength(
      decimated_traj.points, object_pose.position, obstacle_getting_out_idx.get());

    return dist_to_obstacle_getting_out / object_velocity;
  }();

  return time_to_collision - time_to_obstacle_getting_out;
}

void ObstacleVelocityPlannerNode::publishVelocityLimit(
  const boost::optional<VelocityLimit> & vel_limit)
{
  if (vel_limit) {
    vel_limit_pub_->publish(vel_limit.get());
    need_to_clear_vel_limit_ = true;
  } else {
    if (need_to_clear_vel_limit_) {
      const auto clear_vel_limit_msg = createVelocityLimitClearCommandMsg(now());
      clear_vel_limit_pub_->publish(clear_vel_limit_msg);
      need_to_clear_vel_limit_ = false;
    }
  }
}

void ObstacleVelocityPlannerNode::publishDebugData(const DebugData & debug_data) const
{
  stop_watch_.tic(__func__);

  visualization_msgs::msg::MarkerArray debug_marker;
  const auto current_time = now();

  // obstacles to cruise
  for (size_t i = 0; i < debug_data.obstacles_to_cruise.size(); ++i) {
    const auto marker = obstacle_velocity_utils::getObjectMarker(
      debug_data.obstacles_to_cruise.at(i).pose, i, "obstacles_to_cruise", 0.7, 0.7, 0.0);
    debug_marker.markers.push_back(marker);
  }

  // obstacles to stop
  for (size_t i = 0; i < debug_data.obstacles_to_stop.size(); ++i) {
    const auto marker = obstacle_velocity_utils::getObjectMarker(
      debug_data.obstacles_to_stop.at(i).pose, i, "obstacles_to_stop", 1.0, 0.0, 0.0);
    debug_marker.markers.push_back(marker);
  }

  // intentionally ignored obstacles to cruise or stop
  for (size_t i = 0; i < debug_data.intentionally_ignored_obstacles.size(); ++i) {
    const auto marker = obstacle_velocity_utils::getObjectMarker(
      debug_data.intentionally_ignored_obstacles.at(i).kinematics.initial_pose_with_covariance.pose,
      i, "intentionally_ignored_obstacles", 0.0, 1.0, 0.0);
    debug_marker.markers.push_back(marker);
  }

  {  // footprint polygons
    auto marker = tier4_autoware_utils::createDefaultMarker(
      "map", current_time, "detection_polygons", 0, visualization_msgs::msg::Marker::LINE_LIST,
      tier4_autoware_utils::createMarkerScale(0.01, 0.0, 0.0),
      tier4_autoware_utils::createMarkerColor(0.0, 1.0, 0.0, 0.999));

    for (const auto & detection_polygon : debug_data.detection_polygons) {
      for (size_t dp_idx = 0; dp_idx < detection_polygon.outer().size(); ++dp_idx) {
        const auto & current_point = detection_polygon.outer().at(dp_idx);
        const auto & next_point =
          detection_polygon.outer().at((dp_idx + 1) % detection_polygon.outer().size());

        marker.points.push_back(
          tier4_autoware_utils::createPoint(current_point.x(), current_point.y(), 0.0));
        marker.points.push_back(
          tier4_autoware_utils::createPoint(next_point.x(), next_point.y(), 0.0));
      }
    }
    debug_marker.markers.push_back(marker);
  }

  {  // collision points
    for (size_t i = 0; i < debug_data.collision_points.size(); ++i) {
      auto marker = tier4_autoware_utils::createDefaultMarker(
        "map", current_time, "collision_points", i, visualization_msgs::msg::Marker::SPHERE,
        tier4_autoware_utils::createMarkerScale(0.25, 0.25, 0.25),
        tier4_autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));
      marker.pose.position = debug_data.collision_points.at(i);
      debug_marker.markers.push_back(marker);
    }
  }

  debug_marker_pub_->publish(debug_marker);

  // wall for cruise and stop
  debug_cruise_wall_marker_pub_->publish(debug_data.cruise_wall_marker);
  debug_stop_wall_marker_pub_->publish(debug_data.stop_wall_marker);

  // print calculation time
  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleVelocityPlanner"), is_showing_debug_info_, "  %s := %f [ms]",
    __func__, calculation_time);
}

void ObstacleVelocityPlannerNode::publishCalculationTime(const double calculation_time) const
{
  Float32Stamped calculation_time_msg;
  calculation_time_msg.stamp = now();
  calculation_time_msg.data = calculation_time;
  debug_calculation_time_pub_->publish(calculation_time_msg);
}
}  // namespace motion_planning
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motion_planning::ObstacleVelocityPlannerNode)