#include <ur_controller/VSManager.h>
#include <VSTaskPlanner/VSTaskPlanner.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "EVS_Manager");

    ros::NodeHandle nh;

    std::string tasks_file;
    nh.getParam("/EVS_Manager/tasks_file", tasks_file);

    VSPlanner::VSTasksPlanner planner;
    planner.loadTasksFromFile(tasks_file);
    std::cout << planner << std::endl;

    VSManager vs_manager(nh);
    std::cout << vs_manager << std::endl;

    for(const auto &task : planner.getTasksList()){
        vs_manager.moveToJointPose(task.second.getDesiredPose());
        vs_manager.captureDesiredImage();
        vs_manager.moveToJointPose(task.second.getStartPose());
        vs_manager.runTask(task.second, 0.4, 3000); // task, vs_threshold, vs_timeout
    }   

    ros::spin();

    return 0;
}