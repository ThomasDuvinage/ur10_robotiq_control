#include <VSTaskPlanner/VSTask.h>

using namespace VSPlanner;

void VSTask::addParameter(const std::string &key, double value){
    _params.insert({key, value});
}

void VSTask::setStartPose(const std::vector<std::vector<double>> &pose){
    _startPose = pose;

    _startPose_eigen.resize(countNonZerosDOF(pose));

    int index = 0;
    for (auto i = 0; i < _startPose.size(); i++) {
        for(auto o = 0; o < _startPose[i].size(); o++){
            _startPose_eigen[index] = _startPose[i][o];
            index++;
        }
    }
}

void VSTask::setDesiredPose(const std::vector<std::vector<double>> &pose){
    _desiredPose = pose;
    
    _desiredPose_eigen.resize(countNonZerosDOF(pose));

    int index = 0;
    for (auto i = 0; i < _desiredPose.size(); i++) {
        for(auto o = 0; o < _desiredPose[i].size(); o++){
            _desiredPose_eigen[index] = _desiredPose[i][o];
            index++;
        }
    }
}

int VSTask::countNonZerosDOF(const std::vector<std::vector<double>> &pose) const
{
    int size = 0; 

    for(size_t i = 0; i < pose.size(); i++){
        for(size_t o = 0; o < pose[i].size(); o++){
            size++;
        }
    }

    return size;
}