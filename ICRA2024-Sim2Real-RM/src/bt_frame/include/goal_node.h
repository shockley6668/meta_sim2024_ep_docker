#pragma once

#include "behaviortree_cpp/bt_factory.h"
using namespace BT;
namespace BT
{
    //output a goal
    template<> inline void convertFromString(StringView str)
    {
        auto parts = splitString(str, ';');
        if(parts.size() != 2)
        {
            throw RuntimeError("invalid input!");
        }
        else
        {
            //organize a goal
            //x = convertFromString<double>(parts[0]);
            //y = convertFromString<double>(parts[1]);
            //yaw = convertFromString<double>(parts[2]);
            //return .
        }
    }
}

class Goal:public SyncActionNode
{
public:
    Goal(const std::string& name, const NodeConfig& config):SyncActionNode(name, config){}
    static PortsList providedPorts()
    {
        return {
                InputPort<std::string>("board"), 
                InputPort<std::string>("blocks"),
                InputPort<std::string>("ps"), //board or block
                OutputPort<std::string>("next_goal")
                };
    }
    NodeStatus tick() override
    {
        std::cout << "------     Robot goal sending    ------\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));//just for connecting
        std::cout << "------     Robot goal sent       ------\n";
        return NodeStatus::SUCCESS;
    }
private:
    //goals
};