#pragma once

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <eigen3/Eigen/Geometry>
#include <atomic>
#include <cstddef>


namespace VSPlanner{
    class VSTask
    {        
    public:
        VSTask() : _robotPose(RobotPoseGoal::DESIRED) {};
        ~VSTask() = default;

        inline void setId(int id) { _id = id;}
        inline int getId(){return _id;}
        inline int getId() const {return _id;}

        void addParameter(const std::string &key, double value);
        inline const std::map<std::string, double> &getParameters() const {return _params;}

        void setStartPose(const std::vector<std::vector<double>> &pose);
        inline Eigen::VectorXd getStartPose() const {return _startPose_eigen;}
        inline std::vector<std::vector<double>> getStartPoseStdVector() const {return _startPose;}

        void setDesiredPose(const std::vector<std::vector<double>> &pose);
        inline Eigen::VectorXd getDesiredPose() const {return _desiredPose_eigen;}
        inline std::vector<std::vector<double>> getDesiredPoseStdVector() const {return _desiredPose;}

        friend std::ostream& operator<<(std::ostream& os,const VSTask& task){
                os << "  ------- Task " << task.getId() << " -------" << std::endl;
                os << "     starting pose : " << task.getStartPose().transpose() << std::endl;
                os << "     desired pose : " << task.getDesiredPose().transpose() << std::endl;

                os << "     Parameters : " << std::endl;
                
                for(auto it = task.getParameters().begin(); it != task.getParameters().end(); it++){
                    os << "         - " << it->first << " : " << it->second << std::endl;
                }  

                return os;
        }

        enum RobotPoseGoal{
            STARTING,
            DESIRED
        };

        inline RobotPoseGoal getRobotPoseGoal() const {return _robotPose; }
        inline void setRobotPoseGoal(RobotPoseGoal pose) {_robotPose = pose;}

    
    private:
        int _id;
        Eigen::VectorXd _startPose_eigen;
        Eigen::VectorXd _desiredPose_eigen;

        std::vector<std::vector<double>> _startPose;
        std::vector<std::vector<double>> _desiredPose;

        RobotPoseGoal _robotPose; // TODO check for the need of mutex
        
        std::map<std::string, double> _params;

        int countNonZerosDOF(const std::vector<std::vector<double>> &pose) const;
    };
};