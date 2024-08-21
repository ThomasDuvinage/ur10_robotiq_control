#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <map>

#include <boost/filesystem.hpp>

#include <yaml-cpp/yaml.h>

#include <VSTaskPlanner/VSTask.h>

namespace VSPlanner{
    class VSTasksPlanner
    {
    public:
        VSTasksPlanner() = default;
        VSTasksPlanner(const std::string &path);
        ~VSTasksPlanner() {};

        inline std::string getFilePath() const {return _filePath;}

        bool loadTasksFromFile(const std::string &path);

        inline const std::unordered_map<int, VSTask> &getTasksList() const {return _tasks;}

        friend std::ostream& operator<<(std::ostream& os, const VSTasksPlanner& planner){
            os << " ######## VSPlanner ########" << std::endl;
            os << "Loaded tasks file path: " <<  planner.getFilePath() << std::endl;

            for(auto it = planner.getTasksList().begin() ; it != planner.getTasksList().end() ; ++it){
                os << it->second << std::endl;
            }

            return os;
        }

        class Iterator {
            public:
                Iterator(std::unordered_map<int, VSTask>& tasks) : tasks(tasks), it(tasks.begin()) {}

                VSTask &at(int index) {
                    std::advance( it, index);
                    return it->second;
                }

                // Check if there is a next element
                bool hasNext() const {
                    return it != tasks.end();
                }

                // Access the current task and move to the next
                VSTask getNext() {
                    if (hasNext()) {
                        current = it->second;
                        ++it;
                        return current;
                    } else {
                        throw std::out_of_range("No more elements");
                    }
                }

                VSTask &getCurrent(){return current;}

            private:
                const std::unordered_map<int, VSTask>& tasks;
                std::unordered_map<int, VSTask>::iterator it;
                VSTask current;
        };

        // Create an iterator for the task container
        Iterator iterator() {
            return Iterator(_tasks);
        }

    private:
        std::string _filePath;

        std::unordered_map<int, VSTask> _tasks;
        bool loadTasks();

        friend void operator>>(const YAML::Node &node, VSTask &task);
    };
};



