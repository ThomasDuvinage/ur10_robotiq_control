#include <VSTaskPlanner/VSTaskPlanner.h>
#include <iostream>


int main(int argc, char** argv) {
    VSPlanner::VSTasksPlanner planner;
    planner.loadTasksFromFile("/home/tduvinage/devel/src/ur_ws/src/ur10_robotiq_control/ur10_gripper_control/etc/test.yaml");

    std::cout << planner << std::endl;

    return 0;
}
