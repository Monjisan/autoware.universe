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

#include "steering_angle.hpp"

#include <QHBoxLayout>
#include <QString>
#include <QVBoxLayout>
#include <QTimer>
#include <rviz_common/display_context.hpp>

#include <memory>
#include <string>

inline std::string Bool2String(const bool var) { return var ? "True" : "False"; }

using std::placeholders::_1;

namespace rviz_plugins
{
ManualController::ManualController(QWidget * parent) : rviz_common::Panel(parent)
{
  // Gate Mode
  auto * gate_prefix_label_ptr = new QLabel("GATE: ");
  gate_prefix_label_ptr->setAlignment(Qt::AlignRight);
  gate_mode_label_ptr_ = new QLabel("INIT");
  gate_mode_label_ptr_->setAlignment(Qt::AlignCenter);
  auto * gate_layout = new QHBoxLayout;
  gate_layout->addWidget(gate_prefix_label_ptr);
  gate_layout->addWidget(gate_mode_label_ptr_);

  // Selector Mode
  auto * selector_prefix_label_ptr = new QLabel("SELECT: ");
  selector_prefix_label_ptr->setAlignment(Qt::AlignRight);
  selector_mode_label_ptr_ = new QLabel("INIT");
  selector_mode_label_ptr_->setAlignment(Qt::AlignCenter);
  auto * selector_layout = new QHBoxLayout;
  selector_layout->addWidget(selector_prefix_label_ptr);
  selector_layout->addWidget(selector_mode_label_ptr_);

  // State
  auto * state_prefix_label_ptr = new QLabel("STATE: ");
  state_prefix_label_ptr->setAlignment(Qt::AlignRight);
  autoware_state_label_ptr_ = new QLabel("INIT");
  autoware_state_label_ptr_->setAlignment(Qt::AlignCenter);
  auto * state_layout = new QHBoxLayout;
  state_layout->addWidget(state_prefix_label_ptr);
  state_layout->addWidget(autoware_state_label_ptr_);

  // Gear
  auto * gear_prefix_label_ptr = new QLabel("GEAR: ");
  gear_prefix_label_ptr->setAlignment(Qt::AlignRight);
  gear_label_ptr_ = new QLabel("INIT");
  gear_label_ptr_->setAlignment(Qt::AlignCenter);
  auto * gear_layout = new QHBoxLayout;
  gear_layout->addWidget(gear_prefix_label_ptr);
  gear_layout->addWidget(gear_label_ptr_);

  // Engage Status
  auto * engage_prefix_label_ptr = new QLabel("Engage: ");
  engage_prefix_label_ptr->setAlignment(Qt::AlignRight);
  engage_status_label_ptr_ = new QLabel("INIT");
  engage_status_label_ptr_->setAlignment(Qt::AlignCenter);
  auto * engage_status_layout = new QHBoxLayout;
  engage_status_layout->addWidget(engage_prefix_label_ptr);
  engage_status_layout->addWidget(engage_status_label_ptr_);

  // Gate Mode Button
  gate_mode_button_ptr_ = new QPushButton("Gate Mode");
  connect(gate_mode_button_ptr_, SIGNAL(clicked()), SLOT(onClickGateMode()));

  // Velocity Limit
  {
    cruise_velocity_button_ptr_ = new QPushButton("Set Cruise Velocity");
    cruise_velocity_input_ = new QSpinBox();
    cruise_velocity_input_->setRange(-100.0, 100.0);
    cruise_velocity_input_->setValue(0.0);
    cruise_velocity_input_->setSingleStep(5.0);
    connect(cruise_velocity_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickCruiseVelocity()));
  }

  // Layout
  auto * v_layout = new QVBoxLayout;
  auto * gate_mode_path_change_approval_layout = new QHBoxLayout;
  auto * velocity_limit_layout = new QHBoxLayout();
  v_layout->addLayout(gate_layout);
  v_layout->addLayout(selector_layout);
  v_layout->addLayout(state_layout);
  v_layout->addLayout(gear_layout);
  v_layout->addLayout(engage_status_layout);
  v_layout->addLayout(engage_status_layout);
  gate_mode_path_change_approval_layout->addWidget(gate_mode_button_ptr_);
  v_layout->addLayout(gate_mode_path_change_approval_layout);
  velocity_limit_layout->addWidget(cruise_velocity_button_ptr_);
  velocity_limit_layout->addWidget(cruise_velocity_input_);
  velocity_limit_layout->addWidget(new QLabel("  [km/h]"));
  v_layout->addLayout(velocity_limit_layout);
  setLayout(v_layout);

  QTimer * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &ManualController::update);
  timer->start(30);
}

void ManualController::update()
{
  AckermannControlCommand ackermann;
  {
    // ackermann.stamp;
    // ackermann.lateral.stamp;
    ackermann.lateral.steering_tire_angle=steering_angle_;
    // ackermann.longitudinal.stamp;
    ackermann.longitudinal.speed= criuse_velocity_;
    ackermann.longitudinal.acceleration= 1.0;
  }
  GearCommand gear_cmd;
  {
    const double eps =0.001;
    if(ackermann.longitudinal.speed>eps){
      gear_cmd.command = GearCommand::DRIVE;
    } else if(ackermann.longitudinal.speed < -eps) {
      gear_cmd.command=GearCommand::REVERSE;
    } else{
      gear_cmd.command=GearCommand::PARK;
    }
  }
  pub_control_command_->publish(ackermann);
  pub_gear_cmd_->publish(gear_cmd);
  // akermann.lateral
}

void ManualController::onClickCruiseVelocity()
{
  criuse_velocity_ = cruise_velocity_input_->value() / 3.6;
  std::cout<<"cruuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuse"<<criuse_velocity_<<std::endl;
}

void ManualController::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  sub_gate_mode_ = raw_node_->create_subscription<tier4_control_msgs::msg::GateMode>(
    "/control/current_gate_mode", 10, std::bind(&ManualController::onGateMode, this, _1));

  sub_selector_mode_ =
    raw_node_->create_subscription<tier4_control_msgs::msg::ExternalCommandSelectorMode>(
      "/control/external_cmd_selector/current_selector_mode", 10,
      std::bind(&ManualController::onSelectorMode, this, _1));

  pub_gate_mode_ = raw_node_->create_publisher<tier4_control_msgs::msg::GateMode>(
    "/control/gate_mode_cmd", rclcpp::QoS(1));

  pub_control_command_ = raw_node_->create_publisher<AckermannControlCommand>(
    "/external/selected/control_cmd", rclcpp::QoS(1));
  pub_gear_cmd_ = raw_node_->create_publisher<GearCommand>("/external/selected/gear_cmd", 1);
 
}

void ManualController::onGateMode(const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg)
{
  switch (msg->data) {
    case tier4_control_msgs::msg::GateMode::AUTO:
      gate_mode_label_ptr_->setText("AUTO");
      gate_mode_label_ptr_->setStyleSheet("background-color: #00FF00;");
      break;

    case tier4_control_msgs::msg::GateMode::EXTERNAL:
      gate_mode_label_ptr_->setText("EXTERNAL");
      gate_mode_label_ptr_->setStyleSheet("background-color: #FFFF00;");
      break;

    default:
      gate_mode_label_ptr_->setText("UNKNOWN");
      gate_mode_label_ptr_->setStyleSheet("background-color: #FF0000;");
      break;
  }
}

void ManualController::onSelectorMode(
  const tier4_control_msgs::msg::ExternalCommandSelectorMode::ConstSharedPtr msg)
{
  switch (msg->data) {
    case tier4_control_msgs::msg::ExternalCommandSelectorMode::REMOTE:
      selector_mode_label_ptr_->setText("REMOTE");
      selector_mode_label_ptr_->setStyleSheet("background-color: #00FF00;");
      break;

    case tier4_control_msgs::msg::ExternalCommandSelectorMode::LOCAL:
      selector_mode_label_ptr_->setText("LOCAL");
      selector_mode_label_ptr_->setStyleSheet("background-color: #FFFF00;");
      break;

    case tier4_control_msgs::msg::ExternalCommandSelectorMode::NONE:
      selector_mode_label_ptr_->setText("NONE");
      selector_mode_label_ptr_->setStyleSheet("background-color: #FF0000;");
      break;

    default:
      selector_mode_label_ptr_->setText("UNKNOWN");
      selector_mode_label_ptr_->setStyleSheet("background-color: #FF0000;");
      break;
  }
}

void ManualController::onClickGateMode()
{
  const auto data = gate_mode_label_ptr_->text().toStdString() == "AUTO"
                      ? tier4_control_msgs::msg::GateMode::EXTERNAL
                      : tier4_control_msgs::msg::GateMode::AUTO;
  RCLCPP_INFO(raw_node_->get_logger(), "data : %d", data);
  pub_gate_mode_->publish(
    tier4_control_msgs::build<tier4_control_msgs::msg::GateMode>().data(data));
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ManualController, rviz_common::Panel)
