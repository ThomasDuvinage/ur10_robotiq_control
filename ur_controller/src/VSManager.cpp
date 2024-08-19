#include "VSTaskPlanner/VSTask.h"
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <mutex>
#include <sensor_msgs/Image.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"

#include <unistd.h>
#include <ur_controller/VSManager.h>


VSManager::VSManager(ros::NodeHandle &nh) : nh(nh), controller(nh){
    nh.getParam("/EVS_Manager/method", method);
    
    XmlRpc::XmlRpcValue param;
    if (nh.getParam("/"+method, param))
    {
        if (param.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
            for (auto it = param.begin(); it != param.end(); ++it)
            {
                std::string key = it->first;
                std::string value = static_cast<std::string>(it->second);
                method_infos[key] = value;
            }
        }
        else
            ROS_WARN("Parameter %s is not a struct", method.c_str());
    }
    else
        ROS_WARN("Failed to get parameter %s", method.c_str());
}

void VSManager::captureDesiredImage(){
    if(method_infos.count("camera_launch_cmd") != 0){
        pid_camera_launch = fork();
        if(!pid_camera_launch)
            system(method_infos["camera_launch_cmd"].c_str());
        else
            ROS_ERROR("Fail to get PID of the camera_launch_command");
    }
    
    if(method_infos.count("camera_topic") != 0){
        boost::shared_ptr<sensor_msgs::Image const> sharedEdge;
        sensor_msgs::Image edge;
        sharedEdge = ros::topic::waitForMessage<sensor_msgs::Image>(method_infos["camera_topic"], nh);
        
        if(sharedEdge != NULL){
            edge = *sharedEdge;
        }
        else {
            ROS_ERROR("Failed to capture desired image on rostopic %s", method_infos["camera_topic"].c_str());
        }
    }
    else{
        // TODO get the image 
    }

}

void VSManager::moveToJointPose(const Eigen::VectorXd &pose){

    sensor_msgs::JointState init_pose;
    init_pose.position = std::vector<double>(pose.data(), pose.data() + pose.rows() * pose.cols());

    controller.sendJointTrajectory(init_pose, 5);
}


void VSManager::runTask(const VSPlanner::VSTask &task, double vs_threshold, long timeout_ms){
    pid_visual_servoing = fork();
    if(!pid_visual_servoing)
        system("echo 'Run command to launch vs'"); // TODO change 
    else
        ROS_ERROR("Fail to get the PID for visual servoing");
    
    std::cout << "Executing task : " << task << std::endl;

    auto start = std::chrono::system_clock::now();

    while (task_error >= vs_threshold && std::chrono::duration_cast<std::chrono::milliseconds>(
              start.time_since_epoch()).count() < timeout_ms) {
        // 
    }

    // TODO Subscribe to the VS msg and publish the twist member to twist_controller/command

}

void VSManager::stopTask(const VSPlanner::VSTask &task){
    ROS_INFO("Stop task id:%d", task.getId());
    try {
        kill(pid_camera_launch, SIGINT);
        kill(pid_visual_servoing, SIGINT);
    } catch (...) {
        ROS_ERROR("Cannot kill task id:%d", task.getId());
    }

    std::lock_guard<std::mutex> lock_guard(cmd_vel_mutex);
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;

    // TODO Publish cmd_vel here 
}

// TODO Change the message type 
void VSManager::callbackVisualServoing(const std_msgs::Float32ConstPtr &error){
    task_error = error->data;
}
