cmake_minimum_required(VERSION 2.8.3)
project(ur10_gripper_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  control_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(ur10_control_example src/ur10_control_example.cpp)
target_link_libraries(ur10_control_example ${catkin_LIBRARIES})

add_executable(real_robot_control src/real_robot_control.cpp)
target_link_libraries(real_robot_control ${catkin_LIBRARIES})

add_executable(jsonparser src/json_parser.cpp)