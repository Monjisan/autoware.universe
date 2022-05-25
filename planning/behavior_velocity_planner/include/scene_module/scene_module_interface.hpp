// Copyright 2020 Tier IV, Inc.
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

#ifndef SCENE_MODULE__SCENE_MODULE_INTERFACE_HPP_
#define SCENE_MODULE__SCENE_MODULE_INTERFACE_HPP_

#include "behavior_velocity_planner/planner_data.hpp"

#include <builtin_interfaces/msg/time.hpp>

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/stop_reason.hpp>
#include <tier4_planning_msgs/msg/stop_reason_array.hpp>
#include <tier4_v2x_msgs/msg/infrastructure_command_array.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <algorithm>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <unordered_map>

// Debug
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

namespace behavior_velocity_planner
{
using builtin_interfaces::msg::Time;
using unique_identifier_msgs::msg::UUID;
class SceneModuleInterface
{
public:
  explicit SceneModuleInterface(
    const int64_t module_id, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
  : module_id_(module_id), safe_(false), distance_(0.0), logger_(logger), clock_(clock)
  {
  }
  virtual ~SceneModuleInterface() = default;

  virtual bool modifyPathVelocity(
    autoware_auto_planning_msgs::msg::PathWithLaneId * path,
    tier4_planning_msgs::msg::StopReason * stop_reason) = 0;
  virtual visualization_msgs::msg::MarkerArray createDebugMarkerArray() = 0;

  int64_t getModuleId() const { return module_id_; }
  void setPlannerData(const std::shared_ptr<const PlannerData> & planner_data)
  {
    planner_data_ = planner_data;
  }

  boost::optional<tier4_v2x_msgs::msg::InfrastructureCommand> getInfrastructureCommand()
  {
    return infrastructure_command_;
  }

  void setInfrastructureCommand(
    const boost::optional<tier4_v2x_msgs::msg::InfrastructureCommand> & command)
  {
    infrastructure_command_ = command;
  }

  boost::optional<int> getFirstStopPathPointIndex() { return first_stop_path_point_index_; }

  void setActivation(const bool activated) { activated_ = activated; }
  bool isActivated() const { return activated_; }
  bool isSafe() const { return safe_; }
  double getDistance() const { return distance_; }

protected:
  const int64_t module_id_;
  bool activated_;
  bool safe_;
  double distance_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<const PlannerData> planner_data_;
  boost::optional<tier4_v2x_msgs::msg::InfrastructureCommand> infrastructure_command_;
  boost::optional<int> first_stop_path_point_index_;

  void setSafe(const bool safe) { safe_ = safe; }
  void setDistance(const double distance) { distance_ = distance; }
};

class SceneModuleManagerInterface
{
public:
  SceneModuleManagerInterface(rclcpp::Node & node, const char * module_name)
  : clock_(node.get_clock()), logger_(node.get_logger())
  {
    const auto ns = std::string("~/debug/") + module_name;
    pub_debug_ = node.create_publisher<visualization_msgs::msg::MarkerArray>(ns, 20);
    pub_stop_reason_ =
      node.create_publisher<tier4_planning_msgs::msg::StopReasonArray>("~/output/stop_reasons", 20);
    pub_infrastructure_commands_ =
      node.create_publisher<tier4_v2x_msgs::msg::InfrastructureCommandArray>(
        "~/output/infrastructure_commands", 20);
  }

  virtual ~SceneModuleManagerInterface() = default;

  virtual const char * getModuleName() = 0;

  boost::optional<int> getFirstStopPathPointIndex() { return first_stop_path_point_index_; }

  void updateSceneModuleInstances(
    const std::shared_ptr<const PlannerData> & planner_data,
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
  {
    planner_data_ = planner_data;

    launchNewModules(path);
    deleteExpiredModules(path);
  }

  virtual void modifyPathVelocity(autoware_auto_planning_msgs::msg::PathWithLaneId * path)
  {
    visualization_msgs::msg::MarkerArray debug_marker_array;
    tier4_planning_msgs::msg::StopReasonArray stop_reason_array;
    stop_reason_array.header.frame_id = "map";
    stop_reason_array.header.stamp = clock_->now();

    tier4_v2x_msgs::msg::InfrastructureCommandArray infrastructure_command_array;
    infrastructure_command_array.stamp = clock_->now();

    first_stop_path_point_index_ = static_cast<int>(path->points.size()) - 1;
    for (const auto & scene_module : scene_modules_) {
      const UUID uuid = getUUID(scene_module->getModuleId());

      scene_module->setActivation(getActivation(uuid));
      tier4_planning_msgs::msg::StopReason stop_reason;
      scene_module->setPlannerData(planner_data_);
      scene_module->modifyPathVelocity(path, &stop_reason);
      stop_reason_array.stop_reasons.emplace_back(stop_reason);

      updateRTCStatus(
        uuid, scene_module->isSafe(), scene_module->getDistance(), path->header.stamp);

      if (const auto command = scene_module->getInfrastructureCommand()) {
        infrastructure_command_array.commands.push_back(*command);
      }

      if (scene_module->getFirstStopPathPointIndex() < first_stop_path_point_index_) {
        first_stop_path_point_index_ = scene_module->getFirstStopPathPointIndex();
      }

      for (const auto & marker : scene_module->createDebugMarkerArray().markers) {
        debug_marker_array.markers.push_back(marker);
      }
    }

    if (!stop_reason_array.stop_reasons.empty()) {
      pub_stop_reason_->publish(stop_reason_array);
    }
    pub_infrastructure_commands_->publish(infrastructure_command_array);
    pub_debug_->publish(debug_marker_array);
  }

protected:
  virtual void launchNewModules(const autoware_auto_planning_msgs::msg::PathWithLaneId & path) = 0;

  virtual std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
  getModuleExpiredFunction(const autoware_auto_planning_msgs::msg::PathWithLaneId & path) = 0;

  virtual bool getActivation([[maybe_unused]] const UUID & uuid) { return false; }

  virtual void updateRTCStatus(
    [[maybe_unused]] const UUID & uuid, [[maybe_unused]] const bool safe,
    [[maybe_unused]] const double distance, [[maybe_unused]] const Time & stamp)
  {
    return;
  }

  virtual void removeRTCStatus([[maybe_unused]] const UUID & uuid) { return; }

  virtual void publishRTCStatus([[maybe_unused]] const Time & stamp) { return; }

  void deleteExpiredModules(const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
  {
    const auto isModuleExpired = getModuleExpiredFunction(path);

    // Copy container to avoid iterator corruption
    // due to scene_modules_.erase() in unregisterModule()
    const auto copied_scene_modules = scene_modules_;

    for (const auto & scene_module : copied_scene_modules) {
      if (isModuleExpired(scene_module)) {
        removeRTCStatus(getUUID(scene_module->getModuleId()));
        removeUUID(scene_module->getModuleId());
        unregisterModule(scene_module);
      }
    }
  }

  bool isModuleRegistered(const int64_t module_id)
  {
    return registered_module_id_set_.count(module_id) != 0;
  }

  void registerModule(const std::shared_ptr<SceneModuleInterface> & scene_module)
  {
    RCLCPP_INFO(
      logger_, "register task: module = %s, id = %lu", getModuleName(),
      scene_module->getModuleId());
    registered_module_id_set_.emplace(scene_module->getModuleId());
    scene_modules_.insert(scene_module);
  }

  void unregisterModule(const std::shared_ptr<SceneModuleInterface> & scene_module)
  {
    RCLCPP_INFO(
      logger_, "unregister task: module = %s, id = %lu", getModuleName(),
      scene_module->getModuleId());
    registered_module_id_set_.erase(scene_module->getModuleId());
    scene_modules_.erase(scene_module);
  }

  UUID getUUID(const int64_t & module_id) const
  {
    if (map_uuid_.count(module_id) == 0) {
      const UUID uuid;
      return uuid;
    }
    return map_uuid_.at(module_id);
  }

  void generateUUID(const int64_t & module_id)
  {
    // Generate random number
    UUID uuid;
    std::mt19937 gen(std::random_device{}());
    std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
    std::generate(uuid.uuid.begin(), uuid.uuid.end(), bit_eng);
    map_uuid_.insert({module_id, uuid});
  }

  void removeUUID(const int64_t & module_id)
  {
    const auto result = map_uuid_.erase(module_id);
    if (result == 0) {
      RCLCPP_WARN_STREAM(
        logger_, "[removeUUID] module_id = " << module_id << " is not registered.");
    }
  }

  std::set<std::shared_ptr<SceneModuleInterface>> scene_modules_;
  std::set<int64_t> registered_module_id_set_;
  std::unordered_map<int64_t, UUID> map_uuid_;

  std::shared_ptr<const PlannerData> planner_data_;

  boost::optional<int> first_stop_path_point_index_;
  rclcpp::Clock::SharedPtr clock_;
  // Debug
  rclcpp::Logger logger_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_;
  rclcpp::Publisher<tier4_planning_msgs::msg::StopReasonArray>::SharedPtr pub_stop_reason_;
  rclcpp::Publisher<tier4_v2x_msgs::msg::InfrastructureCommandArray>::SharedPtr
    pub_infrastructure_commands_;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__SCENE_MODULE_INTERFACE_HPP_
