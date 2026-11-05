cmake_minimum_required(VERSION 3.20)
project(maze_runner_description NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config DESTINATION share/${PROJECT_NAME})

ament_package()
