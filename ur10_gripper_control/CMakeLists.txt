cmake_minimum_required(VERSION 2.8.3)
project(ur10_gripper_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  control_msgs
  actionlib
  message_generation
)

add_service_files(
  FILES
  MoveRobot.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs control_msgs actionlib 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(ur10_control_example src/ur10_control_example.cpp)
target_link_libraries(ur10_control_example ${catkin_LIBRARIES})

add_executable(real_robot_control src/real_robot_control.cpp)
target_link_libraries(real_robot_control ${catkin_LIBRARIES})

add_executable(ur10_control_service_example src/rosservice_example.cpp)
target_link_libraries(ur10_control_service_example ${catkin_LIBRARIES})

add_executable(ur10_control_service_client_example src/rosservice_client_example.cpp)
target_link_libraries(ur10_control_service_client_example ${catkin_LIBRARIES})

add_executable(jsonparser src/json_parser.cpp)


catkin_install_python(PROGRAMS scripts/rosservice_client_example.py scripts/ros_socket_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)