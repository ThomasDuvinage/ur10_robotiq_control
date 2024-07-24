#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur_robot_publisher");
    ros::NodeHandle nh;

    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

    ros::Rate loop_rate(10); // Publish at 10 Hz

    while (ros::ok())
    {
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_msg.name = {"finger_joint","shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        joint_state_msg.position = {0, 1.0, 0.5, 0.3, 0.9, 1.5, 2.0}; // Example joint positions

        joint_state_pub.publish(joint_state_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}