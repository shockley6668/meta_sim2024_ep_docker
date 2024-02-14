#include <ros/ros.h>
#include <actionlib_msgs/GoalStatus.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
// #include <rmus_solution/switch.h>
// #include <rmus_solution/graspsignal.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <thread>
#include <cmath>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <deque>

// #include "behaviortree_cpp/bt_factory.h"
// #include "behaviortree_cpp/xml_parsing.h"
// #include "behaviortree_cpp/dummy_nodes.h"

#include "bt_factory.h"
#include "xml_parsing.h"
#include "dummy_nodes.h"

using namespace BT;
using namespace std;
namespace chr = std::chrono;

move_base_msgs::MoveBaseGoal create_goal(double x, double y, double yaw)
{
    move_base_msgs::MoveBaseGoal goal_pose;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal_pose.target_pose.header.frame_id = "map";
    goal_pose.target_pose.pose.position.x = x;
    goal_pose.target_pose.pose.position.y = y;
    goal_pose.target_pose.pose.position.z = 0;
    goal_pose.target_pose.pose.orientation.x = q.getX();
    goal_pose.target_pose.pose.orientation.y = q.getY();
    goal_pose.target_pose.pose.orientation.z = q.getZ();
    goal_pose.target_pose.pose.orientation.w = q.getW();

    return goal_pose;
}

// class MoveBaseAction: StatefulActionNode
// {
// public:
//     MoveBaseAction(const std::string& name, const BT::NodeConfig& config) : StatefulActionNode(name, config){}
    
//     static PortsList providedPorts()
//     {
//         return {};
//     }

//     NodeStatus onStart()override;
//     NodeStatus onRunning()override;
//     void onHalted()override;
// private:
//     Pose2D _goal;
//     chr::system_clock::time_point _completion_time;
// };

class PIDController 
{
private:
    double kp, ki, kd, threshold;
    double errorSum, lastError;

public:
    PIDController(double kp, double ki, double kd, double threshold) : kp(kp), ki(ki), kd(kd), threshold(threshold), errorSum(0), lastError(0) {}

    double update(double error, double dt = 0.01) 
    {
        errorSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kp * error + ki * errorSum + kd * derivative;

        if (output > threshold) {
            output = threshold;
        } else if (output < -threshold) {
            output = -threshold;
        }

        return output;
    }
};

class Stop:public SyncActionNode
{
public:
    Stop(const std::string& name, const NodeConfig& config):SyncActionNode(name, config){}
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override
    {
        cout << "------     Robot stopping      ------\n";

        cout << "------     Robot stopped       ------\n";
        return NodeStatus::SUCCESS;
    }
};

class Goal:public SyncActionNode
{
public:
    Goal(const std::string& name, const NodeConfig& config):SyncActionNode(name, config){}
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override
    {
        cout << "------     Robot goal sending    ------\n";

        cout << "------     Robot goal sent       ------\n";
        return NodeStatus::SUCCESS;
    }
};

class MoveToBoard:public SyncActionNode
{
public:
    MoveToBoard(const std::string& name, const NodeConfig& config):SyncActionNode(name, config), name_(name){}
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override
    {
        cout << "------     Robot is moving to board        ------\n";

        move_base_msgs::MoveBaseActionGoal goal_toBoard;
        goal_toBoard.goal = create_goal(-0.05, 1.60, 0.0);
        cout << "now move to board : board goal[-0.05, 1.6, 0.0]" << endl;

        cout << "------     Robot arrived the board         ------\n";
        return NodeStatus::SUCCESS;
    }
private:
    const std::string& name_;
};

class MoveToBlock:public SyncActionNode
{
public:
    MoveToBlock(const std::string& name, const NodeConfig& config):SyncActionNode(name, config), name_(name){}
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override
    {
        if(name_ == "moveto_board")
        {
            move_base_msgs::MoveBaseActionGoal goal_toBoard;
            goal_toBoard.goal = create_goal(-0.05, 1.60, 0.0);
            cout << "now move to board : board goal[-0.05, 1.6, 0.0]" << endl;
        }
        cout << "------     Robot is moving toward the block        ------\n";

        // cout << "here name is :" << name_ << endl;

        cout << "------     Robot arrived to the block              ------\n";
        return NodeStatus::SUCCESS;
    }
private:
    const std::string& name_;
};

class Watch:public SyncActionNode
{
public:
    Watch(const std::string& name, const NodeConfig& config):SyncActionNode(name, config){}
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override
    {
        cout << "------     Robot watching      ------\n";

        cout << "------     Robot watched       ------\n";
        return NodeStatus::SUCCESS;
    }
};

class Get:public SyncActionNode
{
public:
    Get(const std::string& name, const NodeConfig& config):SyncActionNode(name, config){}
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override
    {
        cout << "------     Robot getting message      ------\n";

        cout << "------     Robot got                  ------\n";
        return NodeStatus::SUCCESS;
    }
};

class Update:public SyncActionNode
{
public:
    Update(const std::string& name, const NodeConfig& config):SyncActionNode(name, config){}
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override
    {
        cout << "------     Robot message updating      ------\n";

        cout << "------     Robot updated               ------\n";
        return NodeStatus::SUCCESS;
    }
};

class Aim:public SyncActionNode
{
public:
    Aim(const std::string& name, const NodeConfig& config):SyncActionNode(name, config){}
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override
    {
        cout << "------     Robot aim to block      ------\n";

        cout << "------     Robot aimed             ------\n";
        return NodeStatus::SUCCESS;
    }
};

class Take:public SyncActionNode
{
public:
    Take(const std::string& name, const NodeConfig& config):SyncActionNode(name, config){}
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override
    {
        cout << "------     Robot to take block      ------\n";

        cout << "------     Robot taking block       ------\n";
        return NodeStatus::SUCCESS;
    }
};

class Place:public SyncActionNode
{
public:
    Place(const std::string& name, const NodeConfig& config):SyncActionNode(name, config){}
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override
    {
        cout << "------     Robot placing block      ------\n";

        cout << "------     Robot placed block       ------\n";
        return NodeStatus::SUCCESS;
    }
};

static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree" _fullpath="">
        <Sequence name="Sequence">
            <Sequence name="goto_watch_board">
                <Sequence name="aim_to_board">
                    <Goal name="board"/>
                    <SaySomething   message="robot start moving toward board"/>
                    <MoveBase name="moveto_board"/>
                    <SaySomething   message="robot goal is sended"/>
                </Sequence>
                <Watch name="watch_need_block"/>
                <Get name="get_need_block"/>
            </Sequence>
            <Sequence name="find_there_and_back">
                <Update name="update_need_block"/>
                <Sequence name="block_place">
                    <Goal name="next_block"/>
                    <MoveBase name="moveto_block"/>
                </Sequence>
                <Aim name="aim_block"/>
                <Take name="take_block"/>
                <Sequence name="aim_to_board">
                    <Goal name="board"/>
                    <MoveBase name="moveto_board"/>
                </Sequence>
                <Place name="place_block"/>
            </Sequence>
            <Stop name="stop"/>
        </Sequence>
    </BehaviorTree>
</root>
)";

int main()
{
    BehaviorTreeFactory factory;

    factory.registerNodeType<Stop>("Stop");
    factory.registerNodeType<Goal>("Goal");
    // factory.registerNodeType<MoveBaseAction>("MoveBase");
    factory.registerNodeType<DummyNodes::SaySomething>("SaySomething");

    factory.registerNodeType<MoveToBlock>("MoveToBlock");
    factory.registerNodeType<Watch>("Watch");
    factory.registerNodeType<Get>("Get");
    factory.registerNodeType<Aim>("Aim");
    factory.registerNodeType<Take>("Take");
    factory.registerNodeType<Place>("Place");
    factory.registerNodeType<Update>("Update");


    factory.registerBehaviorTreeFromText(xml_text);
    auto tree = factory.createTree("MainTree");

    std::cout << "----------- XML file  ----------\n"
        << BT::WriteTreeToXML(tree, false, false)
        << "--------------------------------\n";

    tree.tickWhileRunning();
    return 0;
}