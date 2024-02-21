#include <ros/ros.h>
#include "bt_frame/ep_goal.h"
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/Int32.h"
using namespace BT;
class Check_done:public SyncActionNode
{
public:
    Check_done(const std::string& name, const NodeConfig& config):SyncActionNode(name, config){}
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override
    {
        std::cout << "------     checking      ------\n";
        return NodeStatus::FAILURE;
    }
private:
    
};