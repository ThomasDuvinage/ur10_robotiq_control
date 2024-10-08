#include "geometry_msgs/Twist.h"
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose.h>

#include <ur_controller/MoveRobot.h>
 
int main(int argc, char **argv) {
 
    // Initialize the node and set the name
    ros::init(argc, argv, "ur_controller_client");
    
    ros::NodeHandle nh;
    
    ros::ServiceClient client = nh.serviceClient<ur_controller::MoveRobot>("/ur_controller/MoveRobot");
    
    // Create an instance of the srv request type
    ur_controller::MoveRobot srv;

    /****** CARTESIAN CONTROL ******/

    geometry_msgs::Pose pose;
    pose.position.x = 0.640;
    pose.position.y = 0.280;
    pose.position.z = 0.600;
    pose.orientation.x = 0.4905349;
    pose.orientation.y = -0.59114;
    pose.orientation.z = 0.4857658;
    pose.orientation.w = -0.4170858;
    
    // Fill in the two values that will be sent to the Service Server
    srv.request.control_mode = "Cartesian"; // Cartesian or Joint or Velocity
    srv.request.poses = {pose};         // Poses to move to
    srv.request.time_actions = {5};     // time per action in s

    // Example to use the gripper at the end of the move
    std_msgs::Int16 gripper_cmd;
    gripper_cmd.data = 120;
    srv.request.gripper = {gripper_cmd};

    /*******************************/

    /****** JOINT CONTROL ******/

    // sensor_msgs::JointState joints;
    // joints.position = {-0.03298672, -1.52978109, -1.72229091 , 3.06846336 ,-1.65125601, -1.58929682};
    // joints.velocity  = {0.001, 0, 0, 0, 0, 0};
    
    // srv.request.control_mode = "Joint";     // Cartesian or Joint or Velocity
    // srv.request.jointStates = {joints}; // Poses to move to
    // srv.request.time_actions = {5};     // time per action in s

    /*******************************/

    /****** VELOCITY CONTROL ******/
    
    // geometry_msgs::Twist vel;
    // vel.linear.x = 0.1;
    // // Fill the other cordinates

    // srv.request.control_mode = "Velocity";     
    srv.request.cmd_vel = geometry_msgs::Twist(); 

    /*******************************/
    
    // Call the service, and send the data
    if (client.call(srv))
    {
        ROS_INFO("Result: %d", (bool)srv.response.result);
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    
    return 0;
}