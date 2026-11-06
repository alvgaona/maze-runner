cmake_minimum_required(VERSION 3.20)
project(maze_runner_nodes LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sdformat_vendor REQUIRED)
find_package(sdformat REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED COMPONENTS graphics)
find_package(maze_runner_core REQUIRED)

add_library(world_map_loader STATIC src/common/world_map_loader.cpp)
target_include_directories(world_map_loader PUBLIC src)
target_link_libraries(world_map_loader PUBLIC maze_runner_core::maze_runner_core sdformat::sdformat gz-common::graphics)
target_compile_options(world_map_loader PRIVATE -Wall -Wextra -Wpedantic)

add_executable(ekf_lines_node src/localization/ekf_lines_node.cpp)
target_link_libraries(ekf_lines_node
  world_map_loader
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
)
target_compile_options(ekf_lines_node PRIVATE -Wall -Wextra -Wpedantic)

add_executable(nmpc_cbf_node src/control/nmpc_cbf_node.cpp)
target_link_libraries(nmpc_cbf_node
  maze_runner_core::maze_runner_core
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2_ros::tf2_ros
)
target_include_directories(nmpc_cbf_node PRIVATE src)
target_compile_options(nmpc_cbf_node PRIVATE -Wall -Wextra -Wpedantic)

add_executable(planner_node src/planning/planner_node.cpp)
target_link_libraries(planner_node
  world_map_loader
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  tf2_ros::tf2_ros
)
target_compile_options(planner_node PRIVATE -Wall -Wextra -Wpedantic)

add_executable(plan_benchmark src/planning/plan_benchmark.cpp)
target_link_libraries(plan_benchmark world_map_loader)
target_compile_options(plan_benchmark PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS ekf_lines_node nmpc_cbf_node planner_node plan_benchmark DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_world_map_loader test/common/test_world_map_loader.cpp)
  target_link_libraries(test_world_map_loader world_map_loader)
endif()

ament_package()
