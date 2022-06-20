cmake_minimum_required(VERSION 3.14)
project(obstacle_cruise_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

# node
ament_auto_add_library(obstacle_cruise_planner_core SHARED
  src/node.cpp
  src/utils.cpp
  src/polygon_utils.cpp
  src/optimization_based_planner/resample.cpp
  src/optimization_based_planner/box2d.cpp
  src/optimization_based_planner/velocity_optimizer.cpp
  src/optimization_based_planner/optimization_based_planner.cpp
  src/pid_based_planner/pid_based_planner.cpp
)

rclcpp_components_register_node(obstacle_cruise_planner_core
  PLUGIN "motion_planning::ObstacleCruisePlannerNode"
  EXECUTABLE obstacle_cruise_planner
)

# cruis object extractor script
ament_auto_add_executable(cruise_object_extractor
  src/cruise_object_extractor.cpp
)
ament_target_dependencies(cruise_object_extractor)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)

install(PROGRAMS
  scripts/trajectory_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)
