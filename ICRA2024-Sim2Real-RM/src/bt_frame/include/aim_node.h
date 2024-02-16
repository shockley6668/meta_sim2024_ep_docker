#pragma
#include "behaviortree_cpp/bt_factory.h"
#include "PIDController.h"
using namespace BT;
class Aim:public SyncActionNode
{
public:
    Aim(const std::string& name, const NodeConfig& config):SyncActionNode(name, config), aim_block(0, 0, 0, 0){}
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override
    {
        std::cout << "------     Robot aim to block      ------\n";

        std::cout << "------     Robot aimed             ------\n";
        return NodeStatus::SUCCESS;
    }
private:
    PIDController aim_block;
};