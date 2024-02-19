#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace BT;
class GotoWatchBoard : public SyncActionNode
{
public:
    GotoWatchBoard(ros::NodeHandle& Handle,const std::string& name, const BT::NodeConfiguration& config)
        : SyncActionNode(name, config), node_(Handle)
    {
        // Initialize the action client
        action_client_ = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        // Get the goal pose from the input port
        //const geometry_msgs::PoseStamped& goal = getInput<geometry_msgs::PoseStamped>("goal");

        // Create the goal message for the action
        ROS_INFO("Sending goal");
        while(!action_client_->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
        }
        move_base_msgs::MoveBaseGoal goal_msg;
        goal_msg.target_pose.header.frame_id = "map";
        goal_msg.target_pose.header.stamp = ros::Time::now();
        goal_msg.target_pose.pose.position.x = 0;
        goal_msg.target_pose.pose.position.y = 1.37;
        goal_msg.target_pose.pose.orientation.w = 1.0;
        
        // Send the goal to the action server
        action_client_->sendGoal(goal_msg);

        // Wait for the action to complete
        action_client_->waitForResult();

        // Check the result of the action
        if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("success");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_INFO("failure");
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    ros::NodeHandle node_;
    std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> action_client_;
};

