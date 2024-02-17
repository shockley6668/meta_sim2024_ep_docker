#pragma once

#include "behaviortree_cpp/bt_factory.h"
using namespace BT;
class SimplePlanner:public SyncActionNode
{
public:
    SimplePlanner(const std::string& name, const NodeConfig& config):SyncActionNode(name, config){}
    static PortsList providedPorts()
    {
        return {InputPort<std::string>("goal")};
    }
    NodeStatus checkBlock()
    {
        
    }
    NodeStatus tick() override
    {
        
        return NodeStatus::SUCCESS;
    }
private:
    //current goal
};