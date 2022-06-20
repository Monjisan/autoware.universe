// Copyright 2022 TIER IV, Inc.
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

#ifndef OBJECT_CRUISE_PLANNER__NODE_HPP_
#define OBJECT_CRUISE_PLANNER__NODE_HPP_

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"
#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vector>

using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjects;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathPoint;
using nav_msgs::msg::Odometry;
using vehicle_info_util::VehicleInfo;

namespace
{
template <typename T>
size_t findForwardIndex(const T & points, const size_t begin_idx, const double forward_length)
{
  double sum_length = 0.0;
  for (size_t i = begin_idx; i < points.size() - 1; ++i) {
    sum_length += tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i + 1));
    if (sum_length > forward_length) {
      return i;
    }
  }
  return points.size() - 1;
}

template <typename T>
T clipForwardPoints(const T & points, const size_t begin_idx, const double forward_length)
{
  if (points.empty()) {
    return T{};
  }

  const size_t end_idx = findForwardIndex(points, begin_idx, forward_length);
  return T{points.begin() + begin_idx, points.begin() + end_idx};
}

template <typename T>
T clipBackwardPoints(
  const T & points, const size_t target_idx, const double backward_length,
  const double delta_length)
{
  if (points.empty()) {
    return T{};
  }

  const int begin_idx =
    std::max(0, static_cast<int>(target_idx) - static_cast<int>(backward_length / delta_length));
  return T{points.begin() + begin_idx, points.end()};
}
}  // namespace

namespace motion_planning
{
class CruiseObjectExtractorNode : public rclcpp::Node
{
public:
  explicit CruiseObjectExtractorNode(const rclcpp::NodeOptions & node_options);

private:
  struct TargetObject
  {
    TargetObject(const std::string & id, const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Twist & twist)
    {
      predicted_id = id;
      predicted_pose = pose;
      predicted_twist = twist;
    }

    bool is_valid{true};

    std::string predicted_id;
    geometry_msgs::msg::Pose predicted_pose;
    geometry_msgs::msg::Twist predicted_twist;

    geometry_msgs::msg::Twist tracked_twist;

    geometry_msgs::msg::Twist detected_twist;
  };
  std::array<std::unique_ptr<TargetObject>, 10> cruise_objects_;

  // callback functions
  void onPredictedObjects(const PredictedObjects::ConstSharedPtr msg);
  void onDetectedObjects(const DetectedObjects::ConstSharedPtr msg);
  void onTrackedObjects(const TrackedObjects::ConstSharedPtr msg);
  void onPath(const Path::ConstSharedPtr msg);

  // member functions
  void registerPredictedObjects(const PredictedObjects & predicted_objects);
  void registerDetectedObjects(const DetectedObjects & detected_objects);

  // ros parameters
  double forward_path_detection_length_;
  double backward_path_detection_length_;
  double lateral_path_detection_length_;
  double tracking_min_dist_threshold_;

  // publisher
  std::vector<rclcpp::Publisher<Odometry>::SharedPtr> predicted_object_pubs_;
  std::vector<rclcpp::Publisher<Odometry>::SharedPtr> tracked_object_pubs_;
  std::vector<rclcpp::Publisher<Odometry>::SharedPtr> detected_object_pubs_;

  // subscriber
  rclcpp::Subscription<Path>::SharedPtr path_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr predicted_objects_sub_;
  rclcpp::Subscription<TrackedObjects>::SharedPtr tracked_objects_sub_;
  rclcpp::Subscription<DetectedObjects>::SharedPtr detected_objects_sub_;

  // data for callback functions
  PredictedObjects::ConstSharedPtr predicted_objects_ptr_{nullptr};
  DetectedObjects::ConstSharedPtr detected_objects_ptr_{nullptr};
  TrackedObjects::ConstSharedPtr tracked_objects_ptr_{nullptr};
  Path::ConstSharedPtr path_ptr_{nullptr};

  // vehicle Parameters
  VehicleInfo vehicle_info_;

  // self pose listener
  tier4_autoware_utils::SelfPoseListener self_pose_listener_{this};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
}  // namespace motion_planning

#endif  // OBJECT_CRUISE_PLANNER__NODE_HPP_
