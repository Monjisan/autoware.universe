// Copyright 2018-2021 The Autoware Foundation
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

#include "trajectory_follower/vehicle_model/vehicle_model_4ws_kinematics.hpp"

#include <cmath>
#include <iostream>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
FwsModel::FwsModel(const float64_t wheelbase, const float64_t steer_lim, const float64_t steer_tau)
: VehicleModelInterface(/* dim_x */ 4, /* dim_u */ 2, /* dim_y */ 2, wheelbase)
{
  m_steer_lim = steer_lim;
  m_steer_tau = steer_tau;
}

void FwsModel::calculateDiscreteMatrix(
  Eigen::MatrixXd & a_d, Eigen::MatrixXd & b_d, Eigen::MatrixXd & c_d, Eigen::MatrixXd & w_d,
  const float64_t dt)
{
  const float64_t delta_fr = calculateReferenceFrontSteer();
  const float64_t delta_rr = calculateReferenceRearSteer();

  float64_t velocity = m_velocity;
  float64_t psid = m_posture;
  float64_t sigma_1 = velocity*cos(psid+delta_rr);
  float64_t sigma_2 = sin(psid+delta_rr);
  float64_t sigma_3 = m_wheelbase * std::sqrt(std::pow(tan(delta_rr), 2) + 1);
  if (std::abs(m_velocity) < 1e-04) {
    velocity = 1e-04 * (m_velocity >= 0 ? 1 : -1);
  }

  const float64_t a_d_1_2 = velocity * sqrt((pow(tan(delta_fr), 2) + 1)) / sigma_3;
  const float64_t a_d_1_3 =
    -velocity * (1 + tan(delta_fr) * tan(delta_rr)-m_curvature*sigma_2*sigma_3) / sigma_3;
  a_d << 0.0, sigma_1, 0.0,                sigma_1,
         0.0, m_curvature*velocity*sigma_2,      a_d_1_2,            a_d_1_3,
         0.0, 0.0,      -1.0 / m_steer_tau, 0.0,
         0.0, 0.0,      0.0,               -1.0 / m_steer_tau;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_dim_x, m_dim_x);
  a_d = (I - dt * 0.5 * a_d).inverse() * (I + dt * 0.5 * a_d);  // bilinear discretization
  b_d << 0.0,               0.0,
	       0.0,               0.0,
	       1.0 / m_steer_tau, 0.0,
	       0.0,               1.0 / m_steer_tau;
  b_d *= dt;

  c_d << 1.0, 0.0, 0.0, 0.0,
	       0.0, 1.0, 0.0, 0.0;

  float64_t sigma_4=std::sqrt(std::pow(tan(delta_rr), 2) + 1);
  float64_t theta=psid+delta_rr;
  
  const auto w_d_0 = velocity*sin(theta)-velocity*psid*cos(theta)-velocity*delta_rr*cos(theta);
  const auto w_d_1 = velocity*(tan(delta_fr)-tan(delta_rr)-delta_fr*(std::pow(tan(delta_fr), 2) + 1)+delta_rr*(tan(delta_fr)*tan(delta_rr)-m_wheelbase*m_curvature*sin(theta)*sigma_4+1))/(m_wheelbase*sigma_4)
  -m_curvature*velocity*(cos(theta)+psid*sin(theta));
  w_d << w_d_0, w_d_1, 0.0, 0.0;

  w_d *= dt;
}

float64_t FwsModel::calculateReferenceFrontSteer()
{
  auto sign = [](float64_t x) { return (x > 0.0) - (x < 0.0); };

  float64_t beta_r = -m_posture;
  float64_t delta_fr = std::atan((sin(beta_r) + m_wheelbase * m_curvature) / cos(beta_r));
  if (std::abs(delta_fr) >= m_steer_lim) {
    delta_fr = m_steer_lim * static_cast<float64_t>(sign(delta_fr));
  }
  return delta_fr;
}

float64_t FwsModel::calculateReferenceRearSteer()
{
  auto sign = [](float64_t x) { return (x > 0.0) - (x < 0.0); };

  float64_t beta_r = -m_posture;
  float64_t delta_rr = beta_r;
  if (std::abs(delta_rr) >= m_steer_lim) {
    delta_rr = m_steer_lim * static_cast<float64_t>(sign(delta_rr));
  }
  return delta_rr;
}

void FwsModel::calculateReferenceInput(Eigen::MatrixXd & u_ref)
{
  u_ref(0, 0) = calculateReferenceFrontSteer();
  u_ref(1, 0) = calculateReferenceRearSteer();
}

}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
