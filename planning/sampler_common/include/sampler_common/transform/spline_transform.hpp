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

#ifndef SAMPLER_COMMON__TRANSFORM__SPLINE_TRANSFORM_HPP
#define SAMPLER_COMMON__TRANSFORM__SPLINE_TRANSFORM_HPP

#include "sampler_common/structures.hpp"

#include <vector>

namespace sampler_common::transform
{
using sampler_common::FrenetPoint;
using sampler_common::Point;

class Spline
{
  std::vector<double> a_;
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;
  std::vector<double> h_;

public:
  Spline() = delete;
  Spline(const std::vector<double> & base_index, const std::vector<double> & base_value);
  explicit Spline(const std::vector<Point> & points);
  bool interpolate(
    const std::vector<double> & base_index, const std::vector<double> & base_value,
    const std::vector<double> & return_index, std::vector<double> & return_value);
  [[nodiscard]] double value(const double query, const std::vector<double> & base_index) const;
  [[nodiscard]] double velocity(const double query, const std::vector<double> & base_index) const;
  [[nodiscard]] double acceleration(
    const double query, const std::vector<double> & base_index) const;

private:
  void generateSpline(
    const std::vector<double> & base_index, const std::vector<double> & base_value);
  [[nodiscard]] static bool isIncrease(const std::vector<double> & x);
  [[nodiscard]] static bool isValidInput(
    const std::vector<double> & base_index, const std::vector<double> & base_value,
    const std::vector<double> & return_index);
  [[nodiscard]] std::vector<double> solveLinearSystem(
    const double omega, const size_t max_iter) const;
  [[nodiscard]] static bool isConvergeL1(
    const std::vector<double> & r1, const std::vector<double> & r2, const double converge_range);
};

class Spline2D
{
  std::vector<double> s_;
  Spline x_spline_;
  Spline y_spline_;

public:
  Spline2D(const std::vector<double> & x, const std::vector<double> & y);
  [[nodiscard]] FrenetPoint frenet(const Point & p) const;
  [[nodiscard]] Point cartesian(const double s) const;
  [[nodiscard]] Point cartesian(const FrenetPoint & fp) const;
  [[nodiscard]] double curvature(const double s) const;
  [[nodiscard]] double yaw(const double s) const;

private:
  static std::vector<double> arcLength(
    const std::vector<double> & x, const std::vector<double> & y);
};
}  // namespace sampler_common::transform

#endif  // SAMPLER_COMMON__TRANSFORM__SPLINE_TRANSFORM_HPP