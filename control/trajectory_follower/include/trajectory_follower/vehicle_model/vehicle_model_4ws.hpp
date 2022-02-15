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

/*
 *    Representation
 * e      : lateral error
 * th     : heading angle error
 * steer_f  : front steering angle
 * steer_fd: desired front steering angle (input)
 * steer_r  : rear steering angle
 * steer_rd: desired rear steering angle (input)
 * v      : velocity
 * W      : wheelbase length
 * tau    : time constant for steering dynamics
 *
 *    State & Input
 * x = [e, th, steer_f ,steer_r]^T
 * u = steer_fd ,steer_rd
 *
 *    Nonlinear model
 * dx1/dt = v * sin(x2)
 * dx2/dt = v * tan(x3) / W
 * dx3/dt = -(x3 - u) / tau
 *
 *    Linearized model around reference point (v = v_r, th = th_r, steer = steer_r)
 *         [0,  vr,                   0]       [    0]       [                           0]
 * dx/dt = [0,   0, vr/W/cos(steer_r)^2] * x + [    0] * u + [-vr*steer_r/W/cos(steer_r)^2]
 *         [0,   0,               1/tau]       [1/tau]       [                           0]
 *
 */

#ifndef TRAJECTORY_FOLLOWER__VEHICLE_MODEL__VEHICLE_MODEL_4WS_HPP_
#define TRAJECTORY_FOLLOWER__VEHICLE_MODEL__VEHICLE_MODEL_4WS_HPP_

#include "trajectory_follower/vehicle_model/vehicle_model_interface.hpp"

#include "trajectory_follower/visibility_control.hpp"

#include "common/types.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
using autoware::common::types::float64_t;
/**
 * Vehicle model class of bicycle kinematics
 * @brief calculate model-related values
 */
class TRAJECTORY_FOLLOWER_PUBLIC FwsModel : public VehicleModelInterface
{
public:
  /**
   * @brief constructor with parameter initialization
   * @param [in] wheelbase wheelbase length [m]
   * @param [in] steer_lim steering angle limit [rad]
   * @param [in] steer_tau steering time constant for 1d-model [s]
   */
  FwsModel(
    const float64_t wheelbase, const float64_t steer_lim, const float64_t steer_tau);

  /**
   * @brief destructor
   */
  ~FwsModel() = default;

  /**
   * @brief calculate discrete model matrix of x_k+1 = a_d * xk + b_d * uk + w_d, yk = c_d * xk
   * @param [out] a_d coefficient matrix
   * @param [out] b_d coefficient matrix
   * @param [out] c_d coefficient matrix
   * @param [out] w_d coefficient matrix
   * @param [in] dt Discretization time [s]
   */
  void calculateDiscreteMatrix(
    Eigen::MatrixXd & a_d, Eigen::MatrixXd & b_d, Eigen::MatrixXd & c_d, Eigen::MatrixXd & w_d,
    const float64_t dt) override;

  /**
   * @brief calculate reference input
   * @param [out] u_ref input
   */
  void calculateReferenceInput(Eigen::MatrixXd & u_ref) override;

private:
  float64_t m_steer_lim;                                                //!< @brief steering angle limit [rad]
  float64_t m_steer_tau;                                                //!< @brief steering time constant for 1d-model [s]
};
}                          // namespace trajectory_follower
}                  // namespace control
}          // namespace motion
}  // namespace autoware
#endif  // TRAJECTORY_FOLLOWER__VEHICLE_MODEL__VEHICLE_MODEL_4WS_HPP_
