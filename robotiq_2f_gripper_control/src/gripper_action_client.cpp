#include "ros/console.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <robotiq_2f_gripper_control/gripAction.h>

// Define a type for the action client
typedef actionlib::SimpleActionClient<robotiq_2f_gripper_control::gripAction> GripperClient;


int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "gripper_action_client");
    ros::NodeHandle nh("~");

    int goal_position = 0;
    nh.getParam("goal_position", goal_position);
    ROS_INFO("Goal position : %d", goal_position);

    // Create an action client
    GripperClient client("gripper_action_server", true);

    // Wait for the action server to start
    ROS_INFO("Waiting for action server to start...");
    client.waitForServer();
    ROS_INFO("Action server started, sending goal...");

    // Create a goal to send
    robotiq_2f_gripper_control::gripGoal goal;
    goal.position = goal_position;  // Desired position
    goal.speed = 50;      // Speed
    goal.force = 20;      // Force

    // Send the goal and register callbacks
    client.sendGoal(goal);

    // Wait for the result
    client.waitForResult();

    // If you want to explicitly check the result after waiting, you can do so here
    ROS_INFO("Result: %d", client.getResult()->result);

    return 0;
}
