#pragma once

#include <string>
#include <vector>
#include <cstring>

#include <ros/ros.h>
#include <ros/duration.h>
#include <ros/exception.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_client.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryAction.h>
#include <cartesian_control_msgs/CartesianTrajectoryPoint.h>

#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ListControllersRequest.h>
#include <controller_manager_msgs/ListControllersResponse.h>
#include <controller_manager_msgs/LoadControllerRequest.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchControllerRequest.h>
#include <controller_manager_msgs/SwitchController.h>

#include <ur_controller/MoveRobot.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;
typedef actionlib::SimpleActionClient<cartesian_control_msgs::FollowCartesianTrajectoryAction> CartesianTrajectoryClient;

class TrajectoryController {
    public: 
        TrajectoryController(ros::NodeHandle &nh);

        void sendTwistCommand(const geometry_msgs::Twist &twist);
        void switchToVelocityControl();
        void sendGripperCmd(const std_msgs::Int16 &cmd, float wait_time);

        bool sendJointTrajectory(const sensor_msgs::JointState &js, float time_action, const std_msgs::Int16 &gripper_cmd = std_msgs::Int16());

        bool sendCartesianTrajectory(const geometry_msgs::Pose &pose, float time_action, const std_msgs::Int16 &gripper_cmd = std_msgs::Int16());

    public:
        geometry_msgs::Pose init_pose;

    private:
        ros::NodeHandle nh_, nh;
        std::string _control_mode;

        TrajectoryClient joint_trajectory_client_;
        CartesianTrajectoryClient cartesian_trajectory_client_;
        std::string joint_trajectory_controller_, cartesian_trajectory_controller_;

        ros::ServiceClient load_srv;
        ros::ServiceClient list_srv;
        ros::ServiceClient switch_srv;

        ros::ServiceServer service;

        bool _use_gripper;
        ros::Publisher gripper_pub;
        
        ros::Publisher twist_publisher;

        const std::vector<std::string> links = {    "shoulder_pan_joint",
                                                "shoulder_lift_joint",
                                                "elbow_joint",
                                                "wrist_1_joint",
                                                "wrist_2_joint",
                                                "wrist_3_joint"};

        std::vector<std::string> joint_trajectory_controllers = {"scaled_pos_joint_traj_controller",
                                                                "scaled_vel_joint_traj_controller",
                                                                "pos_joint_traj_controller",
                                                                "vel_joint_traj_controller",
                                                                "forward_joint_traj_controller"};

        std::vector<std::string> cartesian_trajectory_controllers = {"pose_based_cartesian_traj_controller",
                                                                    "joint_based_cartesian_traj_controller",
                                                                    "forward_cartesian_traj_controller"};    

        std::vector<std::string> conflicting_controllers = {"joint_group_vel_controller", "twist_controller"};

        void switch_controller(const std::string &target_controller);

        bool move(ur_controller::MoveRobot::Request  &req, ur_controller::MoveRobot::Response &res);

};