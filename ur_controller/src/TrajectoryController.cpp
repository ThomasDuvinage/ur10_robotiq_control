#include "ros/node_handle.h"
#include <ur_controller/TrajectoryController.h>


TrajectoryController::TrajectoryController(ros::NodeHandle &nh) : nh(nh), nh_("~"),joint_trajectory_client_("/scaled_pos_joint_traj_controller/follow_joint_trajectory", true), cartesian_trajectory_client_("/pose_based_cartesian_traj_controller/follow_cartesian_trajectory", true) {
            
    // Get parameters given by the user
    nh_.param<std::string>("controlMode", _control_mode, "Joint"); // Joint, Velocity, Cartesian
    nh_.param<bool>("use_gripper",_use_gripper, false);

    switch_srv = nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    load_srv = nh_.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
    list_srv = nh_.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");

    joint_trajectory_controller_ = joint_trajectory_controllers[3];
    cartesian_trajectory_controller_ = cartesian_trajectory_controllers[0];

    twist_publisher = nh.advertise<geometry_msgs::Twist>("/twist_controller/command", 10);

    if(_use_gripper){
        gripper_pub = nh.advertise<std_msgs::Int16>("/ur_control/gripperCmd", 10);
    }
    
    try {
        service = nh.advertiseService("/ur_controllerler/MoveRobot", &TrajectoryController::move, this);
        ROS_INFO("Rosservice runnning to handle request : /ur_controllerler/MoveRobot");
    } catch (ros::Exception &e) {
        ROS_ERROR("Cannot connect to /ur_controllerler/MoveRobot : %s", e.what());
    }
}

/**
 * @brief Switch to desired controller
 * 
 * @param target_controller 
 */
void TrajectoryController::switch_controller(const std::string &target_controller){
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
                ROS_WARN("Controller is already running...");
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

        
/**
* @brief Send command on gripper topic
* 
* @param cmd 0-255 
* @param wait_time Time to wait for execution
*/
void TrajectoryController::sendGripperCmd(const std_msgs::Int16 &cmd, float wait_time = 2.0){
    gripper_pub.publish(cmd);
    ros::Duration(wait_time).sleep(); 
}


void TrajectoryController::sendTwistCommand(const geometry_msgs::Twist &twist){
    switch_controller("twist_controller");

    twist_publisher.publish(twist);
}

/**
 * @brief Move robot to joint state
 * 
 * @param js sensor_msgs::JointState
 * @param gripper_cmd [optinal] move gripper to desired value at the end of the move
 * @param time_action [optinal] float (default: 5s)
 * @return true OnSuccess
 * @return false OnFail
 */
bool TrajectoryController::sendJointTrajectory(const sensor_msgs::JointState &js, float time_action = 0.5, const std_msgs::Int16 &gripper_cmd) {
    switch_controller(joint_trajectory_controller_);

    if (!joint_trajectory_client_.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("Could not connect to action server");
        ros::shutdown();
    }
    else {
        ROS_INFO("Connected to action server");
    }

    // Create and fill trajectory goal
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = links;

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = js.position;
    point.velocities = js.velocity;
    point.time_from_start = ros::Duration(time_action);
    goal.trajectory.points.push_back(point);

    ROS_INFO("Executing trajectory using the %s", joint_trajectory_controller_.c_str());

    joint_trajectory_client_.sendGoal(goal);

    joint_trajectory_client_.waitForResult();

    actionlib::SimpleClientGoalState state = joint_trajectory_client_.getState();
    ROS_INFO("Trajectory execution finished in state: %s", state.toString().c_str());

    if(state.isDone()){
        if(_use_gripper) sendGripperCmd(gripper_cmd);
        return true;
    }

    return false;
}

/**
 * @brief Move End effector to cartesian position. 
 * 
 * @param pose geometry_msgs::Pose
 * @param gripper_cmd [optinal] move gripper to desired value at the end of the move
 * @param time_action [optinal] float (default: 5s)
 * @return true OnSuccess
 * @return false OnFail
 */
bool TrajectoryController::sendCartesianTrajectory(const geometry_msgs::Pose &pose, float time_action = 5.0, const std_msgs::Int16 &gripper_cmd) {
    switch_controller(cartesian_trajectory_controller_);

    if (!cartesian_trajectory_client_.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("Could not connect to action server");
        ros::shutdown();
    }
    else {
        ROS_INFO("Connected to action server");
    }

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

    if(!result->error_code){
        if(_use_gripper) sendGripperCmd(gripper_cmd);
        return true;
    }

    return false;
}

/**
 * @brief Callback method for the rosservice MoveRobot.  
    Please mention the control mode and the poses or joint states according to the control mode. 
    You can also specify the gripper command to execute at the end of each move.
 * 
 * @param req 
 * @param res 
 * @return true 
 * @return false 
 */
bool TrajectoryController::move(ur_controller::MoveRobot::Request  &req, ur_controller::MoveRobot::Response &res) {
    if(std::strcmp("Cartesian", req.control_mode.c_str()) == 0){
        if(_use_gripper && req.poses.size() == req.gripper.size()){
            for (int i = 0; i < req.poses.size();i++) { 
                if(!sendCartesianTrajectory(req.poses[i], req.time_actions[i], req.gripper[i])){ 
                    res.result = false;
                    return true;
                }
            }
            res.result = true;
        }
        else if(!_use_gripper){
            for (int i = 0; i < req.poses.size();i++) {
                if(!sendCartesianTrajectory(req.poses[i])){ 
                    res.result = false;
                    return true;
                }
            }
            res.result = true;
        }
        else {
            ROS_ERROR("Cannot execute move command, please check the size of poses and gripper commands");
            res.result = false;
        }
    }
    else if (std::strcmp("Joint", req.control_mode.c_str()) == 0) {
        for (auto &js : req.jointStates) {
            sendJointTrajectory(js);
        }
    }
    else if (std::strcmp("Velocity", req.control_mode.c_str()) == 0) {
        sendTwistCommand(req.cmd_vel);
    }

    return true;
}

