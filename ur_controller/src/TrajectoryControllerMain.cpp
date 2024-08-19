#include <ur_controller/TrajectoryController.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_controller");

    ros::NodeHandle nh;

    TrajectoryController controller(nh);

    ros::spin();

    return 0;
}