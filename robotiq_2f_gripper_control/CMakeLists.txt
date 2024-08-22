# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(robotiq_2f_gripper_control)

find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation actionlib_msgs std_msgs actionlib)

add_action_files(
  DIRECTORY action
  FILES grip.action
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs 
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS rospy message_runtime roscpp actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gripper_action_client src/gripper_action_client.cpp)
target_link_libraries(gripper_action_client ${catkin_LIBRARIES})
add_dependencies(gripper_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS src/robotiq_2f_gripper_control/robotiq_gripper.py src/robotiq_2f_gripper_control/gripper_action_client.py src/robotiq_2f_gripper_control/gripper_action_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)