#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorators/loop_node.h"
#include "deque"
using namespace BT;
using namespace std;
struct Pose2D
{
    double x, y, yaw; 
};

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
                InputPort<Pose2D>("board"), 
                InputPort<SharedQueue<Pose2D>>("blocks"),
                InputPort<std::string>("ps"), //board or block
                OutputPort<Pose2D>("goal")
                };
    }
    NodeStatus tick() override
    {
        std::cout << "------     Robot goal sending    ------\n";
        auto msg = getInput<std::string>("ps");
        if(msg == "board")
        {
            auto borad_location = getInput<Pose2D>("board");
            //topic
        }
        else if(msg == "block")
        {
            //init a deque to restore blocks location
            auto shared_queue = std::make_shared<std::deque<Pose2D>>();
            auto blocks_location = getInput<deque<Pose2D>>("blocks");

            //according to situations to arrange properties
            
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));//just for connecting
        std::cout << "------     Robot goal sent       ------\n";
        return NodeStatus::SUCCESS;
    }
private:
    //goals
};