#pragma
#include "behaviortree_cpp/bt_factory.h"
#include "PIDController.h"
using namespace BT;
class Take:public SyncActionNode
{
public:
    Take(const std::string& name, const NodeConfig& config):SyncActionNode(name, config), aim_block(0, 0, 0, 0){}
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus reachArm()
    {

    }
    NodeStatus closeGripper()
    {

    }
    NodeStatus checkBlock()
    {
        
    }
    NodeStatus tick() override
    {
        std::cout << "------     Robot to take block      ------\n";

        std::cout << "------     Robot taking block       ------\n";
        return NodeStatus::SUCCESS;
    }
private:
    PIDController aim_block;
};