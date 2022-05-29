//
//  Copyright 2020 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef AUTOWARE_STATE_PANEL_HPP_
#define AUTOWARE_STATE_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <tier4_control_msgs/msg/external_command_selector_mode.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

namespace rviz_plugins
{
using autoware_auto_control_msgs::msg::AckermannControlCommand;
class ManualController : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ManualController(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:
  void onClickCruiseVelocity();
  void onClickGateMode();
  void update();

protected:
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();
  void onPublishControlCommand();
  void onGateMode(const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg);
  void onSelectorMode(
    const tier4_control_msgs::msg::ExternalCommandSelectorMode::ConstSharedPtr msg);
  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Subscription<tier4_control_msgs::msg::GateMode>::SharedPtr sub_gate_mode_;
  rclcpp::Subscription<tier4_control_msgs::msg::ExternalCommandSelectorMode>::SharedPtr
    sub_selector_mode_;
  rclcpp::Publisher<tier4_control_msgs::msg::GateMode>::SharedPtr pub_gate_mode_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_control_command_;

  double criuse_velocity_{0.0};
  double steering_angle_{0.0};

  QLabel * gate_mode_label_ptr_;
  QLabel * selector_mode_label_ptr_;
  QLabel * autoware_state_label_ptr_;
  QLabel * gear_label_ptr_;
  QLabel * engage_status_label_ptr_;
  QPushButton * engage_button_ptr_;
  QPushButton * cruise_velocity_button_ptr_;
  QPushButton * gate_mode_button_ptr_;
  QPushButton * path_change_approval_button_ptr_;
  QSpinBox * cruise_velocity_input_;

  bool current_engage_;
};

}  // namespace rviz_plugins

#endif  // AUTOWARE_STATE_PANEL_HPP_
