#include <iostream>
#include <vector>
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/istreamwrapper.h"
#include <fstream>

using namespace rapidjson;

struct Action {
    std::vector<double> pos_end_effector;
    bool gripper;
};

int main() {
    std::ifstream ifs; 
    ifs.open("/home/tduvinage/devel/src/ur_ws/src/robotiq/ur10_gripper_control/etc/task.json"); 

    IStreamWrapper isw(ifs);

    // Parse the JSON string
    Document document;
    document.ParseStream(isw);

    // Extract actions from JSON
    std::vector<Action> actions;
    if (document.IsArray()) {
        for (SizeType i = 0; i < document.Size(); ++i) {
            const Value& action = document[i];

            Action a;
            const Value& pos_end_effector = action["pos_end_effector"];
            for (SizeType j = 0; j < pos_end_effector.Size(); ++j) {
                a.pos_end_effector.push_back(pos_end_effector[j].GetDouble());
            }
            a.gripper = action["gripper"].GetBool();

            actions.push_back(a);
        }
    }

    // Print extracted actions
    for (const auto& action : actions) {
        std::cout << "pos_end_effector: [";
        for (size_t i = 0; i < action.pos_end_effector.size(); ++i) {
            std::cout << action.pos_end_effector[i];
            if (i != action.pos_end_effector.size() - 1)
                std::cout << ", ";
        }
        std::cout << "], gripper: " << (action.gripper ? "true" : "false") << std::endl;
    }

    return 0;
}