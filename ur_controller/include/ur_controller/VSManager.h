#pragma once 

#include "geometry_msgs/Twist.h"
#include "ros/subscriber.h"
#include <sched.h>
#include <string>
#include <map>
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <ur_controller/TrajectoryController.h>
#include <VSTaskPlanner/VSTask.h>

class VSManager{
    public:

        VSManager(ros::NodeHandle &nh);

        inline const std::string& getMethodName() const {return method;}
        inline const std::map<std::string, std::string>  &getMethodInfos() const {return method_infos;}

        void moveToJointPose(const Eigen::VectorXd &pose);
        void captureDesiredImage();
        void runTask(const VSPlanner::VSTask &task, double vs_threshold, long timeout_ms);
        void stopTask(const VSPlanner::VSTask &task);

        void callbackVisualServoing(const std_msgs::Float32ConstPtr &error);

        friend std::ostream& operator<<(std::ostream& os, const VSManager& vsm) 
        {
            for(const auto &infos : vsm.getMethodInfos()){
                os << infos.first << ": " << infos.second << std::endl;
            }

            os << std::endl;
            return os;
        }

    private:
        ros::NodeHandle nh;
        std::string method;
        std::map<std::string, std::string> method_infos;

        std::atomic<double> task_error;
        
        std::mutex cmd_vel_mutex;
        geometry_msgs::Twist cmd_vel;

        ros::Subscriber camera_subscriber;
        ros::Subscriber task_error_subscriber;

        TrajectoryController controller;

        pid_t pid_camera_launch, pid_visual_servoing;

};