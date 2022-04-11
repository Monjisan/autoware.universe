// Copyright 2020 TierIV
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
#pragma once
#include "lidar_apollo_instance_segmentation/debugger.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <memory>

class LidarInstanceSegmentationInterface
{
public:
  LidarInstanceSegmentationInterface() {}
  virtual ~LidarInstanceSegmentationInterface() {}
  virtual bool detectDynamicObjects(
    const sensor_msgs::msg::PointCloud2 & input,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature & output) = 0;
};

class LidarInstanceSegmentationNode : public tilde::TildeNode
{
private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  tilde:;TildePublisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr
    dynamic_objects_pub_;
  std::shared_ptr<LidarInstanceSegmentationInterface> detector_ptr_;
  std::shared_ptr<Debugger> debugger_ptr_;
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

public:
  explicit LidarInstanceSegmentationNode(const rclcpp::NodeOptions & node_options);
};
