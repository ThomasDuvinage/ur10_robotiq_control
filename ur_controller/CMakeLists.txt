cmake_minimum_required(VERSION 2.8.3)
project(ur_controller)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
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
  sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs control_msgs actionlib 
)

add_subdirectory(lib)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  lib/VSTaskPlanner/include
)

add_executable(TrajectoryControllerMain src/TrajectoryControllerMain.cpp src/TrajectoryController.cpp)
target_link_libraries(TrajectoryControllerMain ${catkin_LIBRARIES})

add_executable(DemoTrajectoryController src/DemoTrajectoryController src/TrajectoryController.cpp)
target_link_libraries(DemoTrajectoryController ${catkin_LIBRARIES})

add_executable(EVS_Manager src/TrajectoryController.cpp src/VSManager.cpp src/EVS_Manager.cpp)
target_link_libraries(EVS_Manager ${catkin_LIBRARIES} VSTaskPlanner)

add_executable(ur10_control_service_client_example src/rosservice_client_example.cpp)
target_link_libraries(ur10_control_service_client_example ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS scripts/rosservice_client_example.py scripts/ros_socket_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)