#pragma once
#include <ros/ros.h>
#include "bt_frame/ep_goal.h"
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/Int32.h"
#include "std_srvs/Empty.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
using namespace BT;
class SimplePlanner:public StatefulActionNode
{
public:
    SimplePlanner(ros::NodeHandle& Handle,const std::string& name, const NodeConfig& config):
    StatefulActionNode(name, config),node_(Handle){
        goal_sub_=node_.subscribe("/ep_next_goal", 1, &SimplePlanner::goalCallback,this);
        goal_status_pubs_=node_.advertise<std_msgs::Int32>("ep_goal_status", 1);
        cmd_vel_pub_=node_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        goal_pub_ = node_.advertise<geometry_msgs::PointStamped>("/clicked_point", 6);
        move_base_clear = node_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        goal_received=false;
        goal_reached=false;
        move_base_send=false;
        action_client_ = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
        
    }
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus onStart() override
    {
        goal_status_sub_ = node_.subscribe("/cmd_vel", 2, &SimplePlanner::goalStatusCallback, this);
        goal_received=false;
        goal_reached=false;
        move_base_send=false;
        return BT::NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        if(goal_received)
        {
            //movebase
            if(goal_.type==0)
            {
                if(move_base_send==false)
                {
                    move_base_msgs::MoveBaseGoal goal_msg;
                    goal_msg.target_pose.header.frame_id = "map";
                    goal_msg.target_pose.header.stamp = ros::Time::now();
                    goal_msg.target_pose.pose.position.x = goal_.x;
                    goal_msg.target_pose.pose.position.y = goal_.y;
                    tf::Quaternion q=tf::createQuaternionFromYaw(goal_.yaw);
                    goal_msg.target_pose.pose.orientation.w = q.w();
                    goal_msg.target_pose.pose.orientation.x = q.x();
                    goal_msg.target_pose.pose.orientation.y = q.y();
                    goal_msg.target_pose.pose.orientation.z = q.z();
                    action_client_->sendGoal(goal_msg);
                    move_base_send=true;
                }
                else{
                    
                    if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        std::cout<<"move base Goal reached"<<std::endl;
                        goal_reached=true;
                    }
                    if(action_client_->getState() == actionlib::SimpleClientGoalState::ABORTED)
                    {
                        //clear costmap
                        std_srvs::Empty srv;
                        move_base_clear.call(srv);
                        geometry_msgs::Twist recover;
                        recover.linear.x=0.1;
                        cmd_vel_pub_.publish(recover);
                        ros::Duration(0.4).sleep();
                        recover.linear.x=0;
                        cmd_vel_pub_.publish(recover);
                        std::cout<<"move base Goal aborted"<<std::endl;
                        move_base_send=false;
                    }
                    
                }
                

            }
            //simple_planner
            else if(goal_.type==1)
            {
                if(move_base_send==false)
                {
                    ROS_INFO("SimplePlanner: send goal");
                    ros::Duration(0.5).sleep();
                    for(int i=0;i<goal_.points.size();i++)
                    {
                        geometry_msgs::PointStamped p;
                        p.header.frame_id = "map";
                        p.header.stamp = ros::Time::now();
                        p.header.seq = i;
                        p.point.x = goal_.points[i].point.x;
                        p.point.y = goal_.points[i].point.y;
                        goal_pub_.publish(p);
                        ros::Duration(0.05).sleep();
                    }
                    move_base_send=true;
                }
            }
            if(goal_reached)
            {
                goal_reached=false;
                std_msgs::Int32 msg;
                msg.data=1;
                // for(int i=0;i<10;i++)
                {
                    goal_status_pubs_.publish(msg);
                }
                std::cout<<"fuck reached"<<std::endl;
                goal_status_sub_.shutdown();
                return BT::NodeStatus::SUCCESS;
            }
            else{
                std_msgs::Int32 msg;
                msg.data=0;
                goal_status_pubs_.publish(msg);
            }
        }
        
        
        
        return BT::NodeStatus::RUNNING;
    }
    void onHalted() override
    {
      // nothing to do here...
      std::cout << "SimplePlanner interrupted" << std::endl;
    }
private:
    void goalCallback(const bt_frame::ep_goal::ConstPtr& msg)
    {
        goal_=*msg;
        goal_received=true;
    }

    void goalStatusCallback(const geometry_msgs::Twist& msg)
    {
        
        if (goal_reached==false&&msg.linear.z == 1&&goal_.type==1)
        {
            ROS_INFO("simple planner Goal reached");
            goal_reached=true;
        }
    }
    ros::NodeHandle node_;
    std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> action_client_;
    ros::Subscriber goal_sub_;
    ros::Publisher goal_pub_;
    ros::Subscriber goal_status_sub_;
    ros::Publisher goal_status_pubs_;
    ros::ServiceClient move_base_clear;
    ros::Publisher cmd_vel_pub_;
    bool goal_received=false;
    bool goal_reached=false;

    bool move_base_send=false;
    bt_frame::ep_goal goal_;
    //current goal
};