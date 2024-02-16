#pragma
#include "behaviortree_cpp/bt_factory.h"
using namespace BT;
class Observe:public SyncActionNode
{
public:
    Observe(const std::string& name, const NodeConfig& config):SyncActionNode(name, config){}
    static PortsList providedPorts()
    {
        return {
            OutputPort<std::string>("blocks")
        };
    }
    void Update()
    {
        //update blocks that observed
    }
    NodeStatus tick() override
    {
        std::cout << "------     Robot observing      ------\n";
        Update();
        std::cout << "------     Robot observed       ------\n";
        return NodeStatus::SUCCESS;
    }
private:
    //blocks
};