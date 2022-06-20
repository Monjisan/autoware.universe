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

#include "obstacle_cruise_planner/cruise_object_extractor.hpp"

namespace
{
bool isObjectToCruise(
  const geometry_msgs::msg::Pose & object_pose, const std::vector<PathPoint> & path_points,
  const double dist_threshold)
{
  const double dist_to_path =
    tier4_autoware_utils::calcLateralOffset(path_points, object_pose.position);
  return std::abs(dist_to_path) < dist_threshold;
}

std::string toHexString(const unique_identifier_msgs::msg::UUID & id)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +id.uuid[i];
  }
  return ss.str();
}
}  // namespace

namespace motion_planning
{
CruiseObjectExtractorNode::CruiseObjectExtractorNode(const rclcpp::NodeOptions & node_options)
: Node("cruise_object_extractor", node_options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
// , vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
  using std::placeholders::_1;

  // subscribers
  path_sub_ = create_subscription<Path>(
    "/planning/scenario_planning/lane_driving/behavior_planning/path", rclcpp::QoS{1},
    std::bind(&CruiseObjectExtractorNode::onPath, this, _1));

  // detected_objects_sub_ = create_subscription<DetectedObjects>(
  //   "/perception/object_recognition/detection/objects", rclcpp::QoS{1},
  //   std::bind(&CruiseObjectExtractorNode::onDetectedObjects, this, _1));
  predicted_objects_sub_ = create_subscription<PredictedObjects>(
    "/perception/object_recognition/objects", rclcpp::QoS{1},
    std::bind(&CruiseObjectExtractorNode::onPredictedObjects, this, _1));

  // publishers
  for (size_t i = 0; i < cruise_objects_.size(); ++i) {
    detected_object_pubs_.push_back(
      create_publisher<Odometry>("~/detection/cruise_object" + std::to_string(i), 1));
    tracked_object_pubs_.push_back(
      create_publisher<Odometry>("~/tracking/cruise_object" + std::to_string(i), 1));
    predicted_object_pubs_.push_back(
      create_publisher<Odometry>("~/prediction/cruise_object" + std::to_string(i), 1));
  }

  // parameters
  forward_path_detection_length_ =
    declare_parameter<double>("forward_path_detection_length", 100.0);
  backward_path_detection_length_ =
    declare_parameter<double>("backward_path_detection_length", -50.0);
  lateral_path_detection_length_ = declare_parameter<double>("lateral_path_detection_length", 0.0);
  tracking_min_dist_threshold_ = declare_parameter<double>("tracking_min_dist_threshold", 3.0);

  self_pose_listener_.waitForFirstPose();

  // set all cruise objects nullptr
  for (size_t i = 0; i < cruise_objects_.size(); ++i) {
    cruise_objects_.at(i) = nullptr;
  }
}

void CruiseObjectExtractorNode::onPath(const Path::ConstSharedPtr msg) { path_ptr_ = msg; }

void CruiseObjectExtractorNode::onPredictedObjects(const PredictedObjects::ConstSharedPtr msg)
{
  if (!path_ptr_) {
    return;
  }

  registerPredictedObjects(*msg);

  for (size_t i = 0; i < cruise_objects_.size(); ++i) {
    const auto & cruise_object = cruise_objects_.at(i);
    if (!cruise_object) {
      continue;
    }

    Odometry object_odometry;
    object_odometry.pose.pose = cruise_object->predicted_pose;
    object_odometry.twist.twist = cruise_object->predicted_twist;

    std::cerr << i << std::endl;
    // detected_object_pubs_.at(i)->publish(object_odometry);
    predicted_object_pubs_.at(i)->publish(object_odometry);
  }
}

void CruiseObjectExtractorNode::registerPredictedObjects(const PredictedObjects & predicted_objects)
{
  // set all cruise objects invalid first
  for (auto & cruise_object : cruise_objects_) {
    if (cruise_object) {
      cruise_object->is_valid = false;
    }
  }

  // clip path
  const auto ego_pose = self_pose_listener_.getCurrentPose()->pose;
  const size_t ego_idx =
    tier4_autoware_utils::findNearestIndex(path_ptr_->points, ego_pose.position);
  const auto forward_clipped_points =
    clipForwardPoints(path_ptr_->points, ego_idx, forward_path_detection_length_);
  const auto clipped_points =
    clipBackwardPoints(path_ptr_->points, ego_idx, backward_path_detection_length_, 1);

  for (const auto predicted_object : predicted_objects.objects) {
    const auto predicted_pose = predicted_object.kinematics.initial_pose_with_covariance.pose;
    const auto predicted_twist = predicted_object.kinematics.initial_twist_with_covariance.twist;

    // filter object to register
    const bool is_object_to_cruise = isObjectToCruise(
                                                      predicted_pose, clipped_points,
                                                      lateral_path_detection_length_ + 1.0);  // vehicle_info_.vehicle_width_m);
    if (!is_object_to_cruise) {
      continue;
    }

    // register new object
    const std::string predicted_id = toHexString(predicted_object.object_id);
    auto it = std::find_if(
                           std::begin(cruise_objects_), std::end(cruise_objects_),
                           [&](const auto & object) {
                             return object && object->predicted_id == predicted_id; });
    if (it != std::end(cruise_objects_)) {  // found
      *it = std::make_unique<TargetObject>(predicted_id, predicted_pose, predicted_twist);
    } else {  // not found
      // search nullptr object and assign there
      for (size_t i = 0; i < cruise_objects_.size(); ++i) {
        const auto & cruise_object = cruise_objects_.at(i);
        if (!cruise_object) {
          cruise_objects_.at(i) =
            std::make_unique<TargetObject>(predicted_id,  predicted_pose, predicted_twist);
          break;
        }
      }
    }
  }

  // remove obsolete cruise objects
  for (size_t i = 0; i < cruise_objects_.size(); ++i) {
    const auto & cruise_object = cruise_objects_.at(i);
    if (!cruise_object) {
      continue;
    }

    if (cruise_objects_.at(i)->is_valid) {
      continue;
    }

    cruise_objects_.at(i) = nullptr;
  }
}

/*
void CruiseObjectExtractorNode::onDetectedObjects(const DetectedObjects::ConstSharedPtr msg)
{
  if (!path_ptr_) {
    return;
  }

  registerObjects(*msg);

  // publish objects
  for (size_t i = 0; i < cruise_objects_.size(); ++i) {
    const auto & cruise_object = cruise_objects_.at(i);
    if (!cruise_object) {
      continue;
    }

    Odometry object_odometry;
    object_odometry.pose.pose = cruise_object->pose;
    object_odometry.twist.twist = cruise_object->twist;

    std::cerr << i << std::endl;
    object_pubs_.at(i)->publish(object_odometry);
  }
}

void CruiseObjectExtractorNode::registerDetectedObjects(const DetectedObjects & detected_objects)
{
  // set all cruise objects invalid first
  for (auto & cruise_object : cruise_objects_) {
    if (cruise_object) {
      cruise_object->is_valid = false;
    }
  }

  // register new objects
  const auto ego_pose = self_pose_listener_.getCurrentPose()->pose;
  const size_t ego_idx = tier4_autoware_utils::findNearestIndex(path_ptr_->points,
ego_pose.position); const auto forward_clipped_points = clipForwardPoints(path_ptr_->points,
ego_idx, forward_path_detection_length_); const auto clipped_points =
clipBackwardPoints(path_ptr_->points, ego_idx, backward_path_detection_length_, 1);

  for (const auto object : detected_objects.objects) {
    // calculate absolute obstacle pose
    geometry_msgs::msg::PoseStamped rel_object_pose_stamped;
    rel_object_pose_stamped.header = detected_objects.header;
    rel_object_pose_stamped.pose = object.kinematics.pose_with_covariance.pose;

    geometry_msgs::msg::PoseStamped abs_object_pose_stamped;
    try {
      const auto transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      tf2::doTransform(rel_object_pose_stamped, abs_object_pose_stamped, transform);
    } catch (tf2::TransformException & ex) {
      continue;
    }
    const auto abs_object_pose = abs_object_pose_stamped.pose;

    // filter object to register
    const bool is_object_to_cruise = isObjectToCruise(abs_object_pose, clipped_points,
lateral_path_detection_length_ + 1.0); //vehicle_info_.vehicle_width_m); if (!is_object_to_cruise) {
      continue;
    }

    { // register new object
      double min_dist = std::numeric_limits<double>::max();
      size_t min_idx = 0;
      bool is_found = false;
      for (size_t i = 0; i < cruise_objects_.size(); ++i) {
        const auto & cruise_object = cruise_objects_.at(i);
        if (!cruise_object) {
          continue;
        }

        const double dist_to_object = tier4_autoware_utils::calcDistance2d(cruise_object->pose,
abs_object_pose); if (dist_to_object > min_dist || dist_to_object > tracking_min_dist_threshold_) {
          continue;
        }

        min_dist = dist_to_object;
        min_idx = i;
        is_found = true;
      }

      if (is_found) {
        cruise_objects_.at(min_idx) = std::make_unique<TargetObject>(object, abs_object_pose);
      } else {
        // search nullptr object and assign there
        for (size_t i = 0; i < cruise_objects_.size(); ++i) {
          const auto & cruise_object = cruise_objects_.at(i);
          if (!cruise_object) {
            cruise_objects_.at(i) = std::make_unique<TargetObject>(object, abs_object_pose);
            break;
          }
        }
      }
    }
  }

  // remove obsolete cruise objects
  for (size_t i = 0; i < cruise_objects_.size(); ++i) {
    const auto & cruise_object = cruise_objects_.at(i);
    if (!cruise_object) {
      continue;
    }

    if (cruise_objects_.at(i)->is_valid) {
      continue;
    }

    cruise_objects_.at(i) = nullptr;
  }
}
*/
}  // namespace motion_planning

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::spin(std::make_shared<motion_planning::CruiseObjectExtractorNode>(node_options));
  rclcpp::shutdown();
  return 0;
}
