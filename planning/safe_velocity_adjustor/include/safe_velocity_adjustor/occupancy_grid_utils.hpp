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

#ifndef SAFE_VELOCITY_ADJUSTOR__OCCUPANCY_GRID_UTILS_HPP_
#define SAFE_VELOCITY_ADJUSTOR__OCCUPANCY_GRID_UTILS_HPP_

#include "grid_map_core/Polygon.hpp"
#include "grid_map_core/TypeDefs.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "safe_velocity_adjustor/collision_distance.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <boost/geometry.hpp>

#include <opencv2/imgproc/types_c.h>

namespace safe_velocity_adjustor
{

inline void maskPolygons(grid_map::GridMap & grid_map, const multipolygon_t & polygons)
{
  for (const auto & polygon : polygons) {
    grid_map::Polygon grid_map_poly;
    for (const auto & point : polygon.outer()) {
      grid_map_poly.addVertex(grid_map::Position(point.x(), point.y()));
    }
    for (grid_map::PolygonIterator iterator(grid_map, grid_map_poly); !iterator.isPastEnd();
         ++iterator) {
      grid_map.at("layer", *iterator) = 0;
    }
  }
}

inline void threshold(grid_map::GridMap & grid_map, const float occupied_threshold)
{
  for (grid_map::GridMapIterator iter(grid_map); !iter.isPastEnd(); ++iter) {
    auto & val = grid_map.at("layer", *iter);
    if (val < occupied_threshold) {
      val = 0.0;
    } else {
      val = 127;
    }
  }
}

inline void denoise(cv::Mat & cv_image, const int num_iter = 2)
{
  cv::dilate(cv_image, cv_image, cv::Mat(), cv::Point(-1, -1), num_iter);
  cv::erode(cv_image, cv_image, cv::Mat(), cv::Point(-1, -1), num_iter);
}

inline multilinestring_t extractStaticObstaclePolygons(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const multipolygon_t & dynamic_obstacle_polygons, const int8_t occupied_threshold)
{
  cv::Mat cv_image;
  grid_map::GridMap grid_map;
  grid_map::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "layer", grid_map);
  maskPolygons(grid_map, dynamic_obstacle_polygons);
  threshold(grid_map, occupied_threshold);
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(grid_map, "layer", CV_8UC1, cv_image);
  denoise(cv_image);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(cv_image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  multilinestring_t polygons;
  const auto & info = occupancy_grid.info;
  for (const auto & contour : contours) {
    linestring_t polygon;
    for (const auto & point : contour) {
      polygon.emplace_back(
        (info.width - 1.0 - point.y) * info.resolution + info.origin.position.x,
        (info.height - 1.0 - point.x) * info.resolution + info.origin.position.y);
    }
    polygons.push_back(polygon);
  }
  return polygons;
}
}  // namespace safe_velocity_adjustor

#endif  // SAFE_VELOCITY_ADJUSTOR__OCCUPANCY_GRID_UTILS_HPP_