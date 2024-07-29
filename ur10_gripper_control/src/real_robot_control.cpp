#include "cartesian_control_msgs/CartesianTrajectoryPoint.h"
#include "geometry_msgs/Pose.h"
#include "ros/duration.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/service_client.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryAction.h>

#include <std_msgs/Int16.h>

#include <string>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <vector>

#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ListControllersRequest.h>
#include <controller_manager_msgs/ListControllersResponse.h>

#include <controller_manager_msgs/LoadControllerRequest.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchControllerRequest.h>
#include <controller_manager_msgs/SwitchController.h>

#include <ur10_gripper_control/MoveRobot.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;
typedef actionlib::SimpleActionClient<cartesian_control_msgs::FollowCartesianTrajectoryAction> CartesianTrajectoryClient;

class TrajectorySender {
public:
    TrajectorySender() : nh(), nh_("~"),joint_trajectory_client_("/scaled_pos_joint_traj_controller/follow_joint_trajectory", true), joint_trajectory_controller_("my_controller"), cartesian_trajectory_client_("/pose_based_cartesian_traj_controller/follow_cartesian_trajectory", true) {
        
        init_pose.position.x = 0.664;
        init_pose.position.y = 0.728;
        init_pose.position.z = 0.158;

        init_pose.orientation.x = 0.9998099;
        init_pose.orientation.y = 0.003164;
        init_pose.orientation.z = -0.016769;
        init_pose.orientation.w = -0.0094337;


        switch_srv = nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
        load_srv = nh_.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
        list_srv = nh_.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");

        joint_trajectory_controller_ = joint_trajectory_controllers[0];
        cartesian_trajectory_controller_ = cartesian_trajectory_controllers[0];

        gripper_pub = nh.advertise<std_msgs::Int16>("/gripperCmd", 10);

        service = nh.advertiseService("/ur10_gripper_controller/MoveRobot", &TrajectorySender::move, this);
        ROS_INFO("rosservice runnning to handle llm request : /ur10_gripper_controller/MoveRobot");
    }

    void swtich_controller(const std::string &target_controller){
        std::vector<std::string> other_controllers;
        other_controllers.insert( other_controllers.end(), joint_trajectory_controllers.begin(), joint_trajectory_controllers.end() );
        other_controllers.insert( other_controllers.end(), cartesian_trajectory_controllers.begin(), cartesian_trajectory_controllers.end() );
        other_controllers.insert(other_controllers.end(), conflicting_controllers.begin(),conflicting_controllers.end() );
    
        auto it = std::find(other_controllers.begin(), other_controllers.end(), target_controller); 

        if (it != other_controllers.end()) { 
            other_controllers.erase(it); 
        }    

        controller_manager_msgs::ListControllers listing_srv;

        if (list_srv.call(listing_srv))
        {
            for (const auto& controller : listing_srv.response.controller)
            {
                if (controller.name == target_controller && controller.state == "running")
                {
                    ROS_WARN("Controller is already running..");
                    return;
                }
            }
        }
        else
        {
            ROS_ERROR("Failed to call service list_controllers");
            return;
        }

        // Load the target controller
        controller_manager_msgs::LoadController loading_srv;
        loading_srv.request.name = target_controller;
        if (!load_srv.call(loading_srv))
        {
            ROS_ERROR("Failed to call service load_controller");
            return;
        }

        // Switch controllers
        controller_manager_msgs::SwitchController switching_srv;
        switching_srv.request.stop_controllers = other_controllers;
        switching_srv.request.start_controllers.push_back(target_controller);
        switching_srv.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;

        if (!switch_srv.call(switching_srv))
        {
            ROS_ERROR("Failed to call service switch_controller");
        }

        ROS_INFO("Controllers have been switched");

    }

    void sendJointTrajectory() {
        swtich_controller(joint_trajectory_controller_);

        // Create and fill trajectory goal
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names = links;

        // Define trajectory points
        std::vector<std::vector<double>> position_list = {
            {0.0, -1.57, -1.57, 0.0, 0.0, 0.0},
            {0.2, -1.57, -1.57, 0.0, 0.0, 0.0},
            {-0.5, -1.57, -1.2, 0.0, 0.0, 0.0}
        };

        std::vector<std::vector<double>> velocity_list = {
            {0.1, 0.0, 0.0, 0.0, 0.0, 0.0},
            {-0.1, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
        };

        std::vector<double> duration_list = {3.0, 7.0, 10.0};

        for (size_t i = 0; i < position_list.size(); ++i) {
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions = position_list[i];
            point.velocities = velocity_list[i];
            point.time_from_start = ros::Duration(duration_list[i]);
            goal.trajectory.points.push_back(point);
        }

        ROS_INFO("Executing trajectory using the %s", joint_trajectory_controller_.c_str());

        // Send goal
        joint_trajectory_client_.sendGoal(goal);

        // Wait for trajectory execution to complete
        joint_trajectory_client_.waitForResult();

        // Check result
        actionlib::SimpleClientGoalState state = joint_trajectory_client_.getState();
        ROS_INFO("Trajectory execution finished in state: %s", state.toString().c_str());
    }

    bool sendCartesianTrajectory(const geometry_msgs::Pose &pose, bool gripper, float time_action = 5.0) {
        swtich_controller(cartesian_trajectory_controller_);

        if (!cartesian_trajectory_client_.waitForServer(ros::Duration(5.0))) {
            ROS_ERROR("Could not connect to action server");
            ros::shutdown();
        }
        else {
            ROS_INFO("Connected to action server");
        }

        std::cout << "Moving to : " << pose << std::endl;

        // Create and fill trajectory goal
        cartesian_control_msgs::FollowCartesianTrajectoryGoal goal;

        cartesian_control_msgs::CartesianTrajectoryPoint point;
        point.pose = pose;
        point.time_from_start = ros::Duration(time_action);
        goal.trajectory.points.push_back(point);
        
        ROS_INFO("Executing trajectory using the %s", "cartesian_trajectory_controller");

        cartesian_trajectory_client_.sendGoal(goal);
        cartesian_trajectory_client_.waitForResult();

        auto result = cartesian_trajectory_client_.getResult();
        ROS_INFO("Trajectory execution finished in state %d", result->error_code);

        std_msgs::Int16 gripper_msg;

        if(!result->error_code){
            if(gripper){
                gripper_msg.data = 120;
                gripper_pub.publish(gripper_msg);
            }
            else{
                gripper_msg.data = 0;
                gripper_pub.publish(gripper_msg);
            }

            ros::Duration(2).sleep(); // Delay for the gripper to move

            return true;
        }

        return false;
    }

    bool move(ur10_gripper_control::MoveRobot::Request  &req, ur10_gripper_control::MoveRobot::Response &res) {
        if(req.poses.size() == req.gripper.size()){
            for (int i = 0; i < req.poses.size();i++) {

                geometry_msgs::Pose pose = req.poses[i];
                // Change coordinate from camera to base
                pose.position.x = init_pose.position.x + req.poses[i].position.y;
                pose.position.y = init_pose.position.y + req.poses[i].position.x;

                if(!sendCartesianTrajectory(pose, req.gripper[i])){
                    res.result = false;
                    return true;
                }
            }
        }

        res.result = true;

        return true;
    }

public:
    geometry_msgs::Pose init_pose;


private:
    ros::NodeHandle nh_, nh;
    TrajectoryClient joint_trajectory_client_;
    CartesianTrajectoryClient cartesian_trajectory_client_;
    std::string joint_trajectory_controller_, cartesian_trajectory_controller_;

    ros::ServiceClient load_srv;
    ros::ServiceClient list_srv;
    ros::ServiceClient switch_srv;

    ros::ServiceServer service;

    ros::Publisher gripper_pub;

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

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_sender");

    ros::NodeHandle nh;

    TrajectorySender sender;

    ros::spin();

    return 0;
}
