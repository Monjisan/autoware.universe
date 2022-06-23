// Copyright 2015-2019 Autoware Foundation
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
#include "vehicle_cmd_gate/vehicle_cmd_filter.hpp"

#include <algorithm>
#include <cmath>

VehicleCmdFilter::VehicleCmdFilter() {}

void VehicleCmdFilter::limitLongitudinalWithVel(
  autoware_auto_control_msgs::msg::AckermannControlCommand & input) const
{
  input.longitudinal.speed = std::max(
    std::min(static_cast<double>(input.longitudinal.speed), param_.vel_lim), -param_.vel_lim);
}

void VehicleCmdFilter::limitLongitudinalWithAcc(
  const double dt, autoware_auto_control_msgs::msg::AckermannControlCommand & input) const
{
  input.longitudinal.acceleration = std::max(
    std::min(static_cast<double>(input.longitudinal.acceleration), param_.lon_acc_lim),
    -param_.lon_acc_lim);
  input.longitudinal.speed =
    limitDiff(input.longitudinal.speed, prev_cmd_.longitudinal.speed, param_.lon_acc_lim * dt);
}

void VehicleCmdFilter::VehicleCmdFilter::limitLongitudinalWithJerk(
  const double dt, autoware_auto_control_msgs::msg::AckermannControlCommand & input) const
{
  input.longitudinal.acceleration = limitDiff(
    input.longitudinal.acceleration, prev_cmd_.longitudinal.acceleration, param_.lon_jerk_lim * dt);
}

void VehicleCmdFilter::limitLateralWithLatAcc(
  [[maybe_unused]] const double dt,
  autoware_auto_control_msgs::msg::AckermannControlCommand & input) const
{
  double latacc = calcLatAcc(input);
  if (std::fabs(latacc) > param_.lat_acc_lim) {
    double v_sq =
      std::max(static_cast<double>(input.longitudinal.speed * input.longitudinal.speed), 0.001);
    double steer_lim = std::atan(param_.lat_acc_lim * param_.wheel_base / v_sq);
    input.lateral.steering_tire_angle = latacc > 0.0 ? steer_lim : -steer_lim;
  }
}

void VehicleCmdFilter::limitLateralWithLatJerk(
  const double dt, autoware_auto_control_msgs::msg::AckermannControlCommand & input) const
{
  double curr_latacc = calcLatAcc(input);
  double prev_latacc = calcLatAcc(prev_cmd_);

  const double latacc_max = prev_latacc + param_.lat_jerk_lim * dt;
  const double latacc_min = prev_latacc - param_.lat_jerk_lim * dt;

  if (curr_latacc > latacc_max) {
    input.lateral.steering_tire_angle = calcSteerFromLatacc(input.longitudinal.speed, latacc_max);
  } else if (curr_latacc < latacc_min) {
    input.lateral.steering_tire_angle = calcSteerFromLatacc(input.longitudinal.speed, latacc_min);
  }
}

void VehicleCmdFilter::limitActualSteerDiff(
  const double current_steer_angle,
  autoware_auto_control_msgs::msg::AckermannControlCommand & input) const
{
  auto ds = input.lateral.steering_tire_angle - current_steer_angle;
  ds = std::clamp(ds, -param_.actual_steer_diff_lim, param_.actual_steer_diff_lim);
  input.lateral.steering_tire_angle = current_steer_angle + ds;
}

bool VehicleCmdFilter::compareCmd(const std::string & name, const autoware_auto_control_msgs::msg::AckermannControlCommand & b, const autoware_auto_control_msgs::msg::AckermannControlCommand & a) const
{
  const auto ep = 1.0e-3;
  const auto vb = b.longitudinal.speed;
  const auto ab = b.longitudinal.acceleration;
  const auto sb = b.lateral.steering_tire_angle;
  const auto srb = b.lateral.steering_tire_rotation_rate;

  const auto va = a.longitudinal.speed;
  const auto aa = a.longitudinal.acceleration;
  const auto sa = a.lateral.steering_tire_angle;
  const auto sra = a.lateral.steering_tire_rotation_rate;


  const auto dv = std::abs(va - vb);
  const auto da = std::abs(aa - ab);
  const auto ds = std::abs(sa - sb);
  const auto dsv = std::abs(sra - srb);

  const bool has_change = dv > ep || da > ep || ds > ep || dsv > ep;
  if (has_change) {
    RCLCPP_ERROR(rclcpp::get_logger("cmd_filter"), "[%s] has change: v: %f -> %f, a: %f -> %f, s: %f -> %f, sr: %f -> %f", name.c_str(), vb, va, ab, aa, sb, sa, srb, sra);
  }

  return has_change;
}

void VehicleCmdFilter::filterAll(
  const double dt, const double current_steer_angle,
  autoware_auto_control_msgs::msg::AckermannControlCommand & cmd) const
{

  auto tmp = cmd;
  limitLongitudinalWithVel(cmd);
  compareCmd("limitLongitudinalWithVel", tmp, cmd);
  tmp = cmd;
  limitLongitudinalWithAcc(dt, cmd);
  compareCmd("limitLongitudinalWithAcc", tmp, cmd);
  tmp = cmd;
  limitLongitudinalWithJerk(dt, cmd);
  compareCmd("limitLongitudinalWithJerk", tmp, cmd);
  tmp = cmd;
  limitLateralWithLatAcc(dt, cmd);
  compareCmd("limitLateralWithLatAcc", tmp, cmd);
  tmp = cmd;
  limitLateralWithLatJerk(dt, cmd);
  compareCmd("limitLateralWithLatJerk", tmp, cmd);
  tmp = cmd;
  limitActualSteerDiff(current_steer_angle, cmd);
  compareCmd("limitActualSteerDiff", tmp, cmd);
  return;
}

double VehicleCmdFilter::calcSteerFromLatacc(const double v, const double latacc) const
{
  const double v_sq = std::max(v * v, 0.001);
  return std::atan(latacc * param_.wheel_base / v_sq);
}

double VehicleCmdFilter::calcLatAcc(
  const autoware_auto_control_msgs::msg::AckermannControlCommand & cmd) const
{
  double v = cmd.longitudinal.speed;
  return v * v * std::tan(cmd.lateral.steering_tire_angle) / param_.wheel_base;
}

double VehicleCmdFilter::limitDiff(
  const double curr, const double prev, const double diff_lim) const
{
  double diff = std::max(std::min(curr - prev, diff_lim), -diff_lim);
  return prev + diff;
}
