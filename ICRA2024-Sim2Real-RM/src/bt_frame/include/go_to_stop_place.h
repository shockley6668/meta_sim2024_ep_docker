#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <algorithm> 

class GotoStop : public StatefulActionNode
{
public:
    GotoStop(ros::NodeHandle& Handle,const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config), node_(Handle)
    {
        // Initialize the action client
        action_client_ = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
        cmd_vel_pub = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    }

    static BT::PortsList providedPorts()
    {
        return {
            
        };
    }


    NodeStatus onStart() override
    {
        move_base_msgs::MoveBaseGoal goal_msg;
        goal_msg.target_pose.header.frame_id = "map";
        goal_msg.target_pose.header.stamp = ros::Time::now();
        goal_msg.target_pose.pose.position.x = 0;
        goal_msg.target_pose.pose.position.y = 1.37;
        goal_msg.target_pose.pose.orientation.w = 1.0;
        // Send the goal to the action server
        action_client_->sendGoal(goal_msg);
        ROS_INFO("Sending goal"); 
        return BT::NodeStatus::RUNNING;
    }
    NodeStatus onRunning() override
    {


        // Check the result of the action
        if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("success");
            return BT::NodeStatus::SUCCESS;
        }
        else if(action_client_->getState() == actionlib::SimpleClientGoalState::ABORTED)
        {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.5;
            cmd_vel_pub.publish(cmd_vel);
            ros::Duration(0.4).sleep();
            move_base_msgs::MoveBaseGoal goal_msg;
            goal_msg.target_pose.header.frame_id = "map";
            goal_msg.target_pose.header.stamp = ros::Time::now();
            goal_msg.target_pose.pose.position.x = 0;
            goal_msg.target_pose.pose.position.y = 1.37;
            goal_msg.target_pose.pose.orientation.w = 1.0;
            // Send the goal to the action server
            action_client_->sendGoal(goal_msg);
            ROS_INFO("Sending goal"); 
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }
    void onHalted() override
    {
      // nothing to do here...
      tag_sub.shutdown();
      std::cout << "goto_stop interrupted" << std::endl;
    }

private:

    ros::NodeHandle node_;
    std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> action_client_;
    ros::Publisher cmd_vel_pub;


};

