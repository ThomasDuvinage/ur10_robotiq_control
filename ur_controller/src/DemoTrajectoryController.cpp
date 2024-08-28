#include <ur_controller/TrajectoryController.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_sender");

    ros::NodeHandle nh;

    TrajectoryController sender(nh);

    sensor_msgs::JointState joints;
    joints.position = {-0.03298672, -1.52978109, -1.72229091 , 3.06846336 ,-1.65125601, -1.58929682};
    joints.velocity  = {0.001, 0, 0, 0, 0, 0};

    sender.sendJointTrajectory(joints, 5);

    ros::spin();

    return 0;
}