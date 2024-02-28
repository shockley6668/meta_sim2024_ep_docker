#include <chrono>
#include "ros/ros.h"
#include "behaviortree_cpp/bt_factory.h"
using namespace BT;
class Timeout : public StatefulActionNode
{
public:
    Timeout(ros::NodeHandle& Handle, const std::string& name, const NodeConfig& config):StatefulActionNode(name, config)
    {
        nh = Handle;
        start = std::chrono::high_resolution_clock::now();
    }
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus onStart() override
    {
        return BT::NodeStatus::RUNNING;
    }
    NodeStatus onRunning() override
    {
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end-start;
        std::cout << "Time from start: " << diff.count() << std::endl;
        if(diff.count() > 265)
        {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }
    void onHalted() override
    {
        std::cout << "Timeout interrupted" << std::endl;
    }
private:
    ros::NodeHandle nh;
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
};