#include "ros/ros.h"
 
// Include the srv file that we already created
#include <ur10_gripper_control/MoveRobot.h>
 
// Add two numbers and output the sum
bool move(ur10_gripper_control::MoveRobot::Request  &req, ur10_gripper_control::MoveRobot::Response &res) {
    // Move the robot 
    // ........

    ROS_INFO("Moving the robot.....");

    res.result = true;
    
    return true;
}
 
// Main ROS method
int main(int argc, char **argv) {
     
  // Initialize the node and set the name
  ros::init(argc, argv, "ur10_gripper_controller_server");
   
  // Create the main access point for the node
  // This piece of code enables the node to communicate with the ROS system.
  ros::NodeHandle n;
 
  // Create the service and advertise it to the ROS computational network
  ros::ServiceServer service = n.advertiseService("ur10_gripper_controller/MoveRobot", move);
   
  // Print message to terminal window
  ROS_INFO("ur10_gripper_controller_server has started");
   
  // Keep processing information over and over again
  ros::spin();
 
  // Program completed successfully
  return 0;
}