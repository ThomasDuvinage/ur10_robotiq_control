#include "ros/ros.h"
#include <ur10_gripper_control/MoveRobot.h>
#include <geometry_msgs/Pose.h>
 
#include <cstdlib>
 
int main(int argc, char **argv) {
 
    // Initialize the node and set the name
    ros::init(argc, argv, "ur10_gripper_controller_client");
    
    ros::NodeHandle n;
    
    ros::ServiceClient client = n.serviceClient<ur10_gripper_control::MoveRobot>("ur10_gripper_controller/MoveRobot");
    
    // Create an instance of the srv request type
    ur10_gripper_control::MoveRobot srv;

    geometry_msgs::Pose pose;
    
    // Fill in the two values that will be sent to the Service Server
    srv.request.Pose = pose;
    
    // Call the service, and send the data
    if (client.call(srv))
    {
        ROS_INFO("Result: %d", (bool)srv.response.result);
    }
    else
    {
        ROS_ERROR("Failed to call service adder_server");
        return 1;
    }
    
    return 0;
}