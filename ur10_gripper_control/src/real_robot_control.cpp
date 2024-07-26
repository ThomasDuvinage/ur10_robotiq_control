#include "cartesian_control_msgs/CartesianTrajectoryPoint.h"
#include "ros/service_client.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryAction.h>

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

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;
typedef actionlib::SimpleActionClient<cartesian_control_msgs::FollowCartesianTrajectoryAction> CartesianTrajectoryClient;

class TrajectorySender {
public:
    TrajectorySender() : nh_("~"),joint_trajectory_client_("/scaled_pos_joint_traj_controller/follow_joint_trajectory", true), joint_trajectory_controller_("my_controller"), cartesian_trajectory_client_("/pose_based_cartesian_traj_controller/follow_cartesian_trajectory", true) {
        if (!cartesian_trajectory_client_.waitForServer(ros::Duration(5.0))) {
            ROS_ERROR("Could not connect to action server");
            ros::shutdown();
        }

        switch_srv = nh_.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
        load_srv = nh_.serviceClient<controller_manager_msgs::LoadController>("controller_manager/load_controller");
        list_srv = nh_.serviceClient<controller_manager_msgs::ListControllers>("controller_manager/list_controllers");

        joint_trajectory_controller_ = joint_trajectory_controllers[0];
        cartesian_trajectory_controller_ = cartesian_trajectory_controllers[0];
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



    void sendCartesianTrajectory() {
        swtich_controller(cartesian_trajectory_controller_);

        // Create and fill trajectory goal
        cartesian_control_msgs::FollowCartesianTrajectoryGoal goal;

        std::vector<geometry_msgs::Pose> pose_list = {
            createPose(0.4, -0.1, 0.4)
        };

        // ,
        //     createPose(0.4, -0.1, 0.6),
        //     createPose(0.4, 0.3, 0.6),
        //     createPose(0.4, 0.3, 0.4),
        //     createPose(0.4, -0.1, 0.4)

        std::vector<double> duration_list = {5.0}; // , 4.0, 5.0, 6.0, 7.0

        for (size_t i = 0; i < pose_list.size(); ++i) {
            cartesian_control_msgs::CartesianTrajectoryPoint point;
            point.pose = pose_list[i];
            point.time_from_start = ros::Duration(duration_list[i]);
            goal.trajectory.points.push_back(point);
        }

        ROS_INFO("Executing trajectory using the %s", "cartesian_trajectory_controller");

        cartesian_trajectory_client_.sendGoal(goal);
        cartesian_trajectory_client_.waitForResult();

        auto result = cartesian_trajectory_client_.getResult();
        ROS_INFO("Trajectory execution finished in state %d", result->error_code);
    }



private:
    ros::NodeHandle nh_;
    TrajectoryClient joint_trajectory_client_;
    CartesianTrajectoryClient cartesian_trajectory_client_;
    std::string joint_trajectory_controller_, cartesian_trajectory_controller_;

    ros::ServiceClient load_srv;
    ros::ServiceClient list_srv;
    ros::ServiceClient switch_srv;

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

    geometry_msgs::Pose createPose(double x, double y, double z) {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        return pose;
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_sender");

    TrajectorySender sender;
    sender.sendCartesianTrajectory();

    return 0;
}
