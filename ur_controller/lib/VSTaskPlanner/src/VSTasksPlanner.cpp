#include <VSTaskPlanner/VSTaskPlanner.h>
#include <iterator>

using namespace VSPlanner;

VSTasksPlanner::VSTasksPlanner(const std::string &path) : _filePath(path)
{
    loadTasks();
}

void operator>>(const YAML::Node &node, VSTask &task){
    task.setId(node["task_id"].as<int>());

    std::vector<std::vector<double>> start_pose = node["starting_pose"].as<std::vector<std::vector<double>>>();
    task.setStartPose(start_pose);

    std::vector<std::vector<double>> des_pose = node["desired_pose"].as<std::vector<std::vector<double>>>();
    task.setDesiredPose(des_pose);

    if(node["parameters"]){
        YAML::Node parametersNode = node["parameters"];

        for(YAML::const_iterator it=parametersNode.begin();it != parametersNode.end();++it) {
            task.addParameter(it->first.as<std::string>(), it->second.as<double>());
        }
    }
}

bool VSTasksPlanner::loadTasks(){

    if(boost::filesystem::exists(_filePath)){
        YAML::Node config = YAML::LoadFile(_filePath);

        for(unsigned int i = config.size() ; i-- > 0 ;){
            VSTask task;
            config[i] >> task;
            _tasks.insert(std::pair<int, VSTask>(task.getId(), task));
        }
        return true;
    }

    return false;
}


bool VSTasksPlanner::loadTasksFromFile(const std::string &path){
    _filePath = path;
    return loadTasks();
}