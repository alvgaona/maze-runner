cmake_minimum_required(VERSION 3.20)
project(maze_runner_gazebo NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds config launch DESTINATION share/${PROJECT_NAME})

ament_package()
