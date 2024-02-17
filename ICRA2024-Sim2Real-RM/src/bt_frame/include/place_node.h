#pragma
#include "behaviortree_cpp/bt_factory.h"
#include "PIDController.h"
using namespace BT;
class Place:public SyncActionNode
{
public:
    Place(const std::string& name, const NodeConfig& config):SyncActionNode(name, config), aim_board(0, 0, 0, 0){}
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus reachArm()
    {

    }
    NodeStatus openGripper()
    {
        
    }
    NodeStatus tick() override
    {
        std::cout << "------     Robot placing block      ------\n";

        std::cout << "------     Robot placed block       ------\n";
        return NodeStatus::SUCCESS;
    }
private:
    PIDController aim_board;
};