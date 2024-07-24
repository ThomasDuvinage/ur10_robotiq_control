#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "real_robot_control");
    ros::NodeHandle nh;

    ros::Publisher joint_trajectory_pub = nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal", 10);

    ros::Rate loop_rate(10); // Publish at 10 Hz

    while (ros::ok())
    {
        control_msgs::FollowJointTrajectoryActionGoal joint_trajectory_msg;

        joint_trajectory_msg.goal.trajectory.header.stamp = ros::Time::now();
        joint_trajectory_msg.goal.trajectory.joint_names = {"elbow_joint", "shoulder_pan_joint", "shoulder_lift_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = {1.822124, -2.513274, -1.382301, 0.628319, 0.376991, 0.879646}; // Example joint positions
        point.time_from_start = ros::Duration(5.0); // Example duration to reach this point

        joint_trajectory_msg.goal.trajectory.points.push_back(point);

        joint_trajectory_pub.publish(joint_trajectory_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
