#pragma once
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
#include "PIDController.h"
using namespace BT;
class Place_Down : public StatefulActionNode
{
public:
    Place_Down(ros::NodeHandle& Handle,const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config), node_(Handle)
    {
        action_client_ = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
        cmd_pub = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        arm_gripper_pub=node_.advertise<geometry_msgs::Point>("arm_gripper", 2);
        arm_position_pub=node_.advertise<geometry_msgs::Pose>("arm_position", 2);
        reached_flag = false;
        detected=false;
        y_pid = PIDController(y_param);
        mid_tag_id = -1;
    }
    static BT::PortsList providedPorts()
    {
        return {};
    }
    NodeStatus onStart() override
    {
        move_base_msgs::MoveBaseGoal goal_msg;
        sendBaseVel(0,0,0);
        goal_msg.target_pose.header.frame_id = "map";
        goal_msg.target_pose.header.stamp = ros::Time::now();
        goal_msg.target_pose.pose.position.x = 1.18;
        goal_msg.target_pose.pose.position.y = 1.78;
        tf::Quaternion q=tf::createQuaternionFromYaw(0.0);
        goal_msg.target_pose.pose.orientation.w = q.w();
        goal_msg.target_pose.pose.orientation.x = q.x();
        goal_msg.target_pose.pose.orientation.y = q.y();
        goal_msg.target_pose.pose.orientation.z = q.z();
        action_client_->sendGoal(goal_msg);
        tag_sub=node_.subscribe("/tag_detections", 1, &Place_Down::tagCallback,this);
        nav_done=false;   
        return BT::NodeStatus::RUNNING;     
    }
    NodeStatus onRunning() override
    {
        cout<<action_client_->getState().toString()<<endl;
        if(action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED&&nav_done==false)
        {
            nav_done=true;
            sendBaseVel(0,0,0);
        }
        else if(nav_done==false&&action_client_->getState() == actionlib::SimpleClientGoalState::ABORTED)
        {
            move_base_msgs::MoveBaseGoal goal_msg;
            goal_msg.target_pose.header.frame_id = "map";
            goal_msg.target_pose.header.stamp = ros::Time::now();
            goal_msg.target_pose.pose.position.x = 1.2;
            goal_msg.target_pose.pose.position.y = 1.78;
            tf::Quaternion q=tf::createQuaternionFromYaw(0.0);
            goal_msg.target_pose.pose.orientation.w = q.w();
            goal_msg.target_pose.pose.orientation.x = q.x();
            goal_msg.target_pose.pose.orientation.y = q.y();
            goal_msg.target_pose.pose.orientation.z = q.z();
            action_client_->sendGoal(goal_msg);
        }
        if (nav_done&&detected)
        {
            std::cout<<"box:"<<mid_tag.pose.pose.pose.position.x<<endl;
            if(abs(aim_set-mid_tag.pose.pose.pose.position.x)>0.01)
            {
                float cmd_y=y_pid.calculate(aim_set, mid_tag.pose.pose.pose.position.x);
                sendBaseVel(0,cmd_y,0);
            }
            else{
                
                    sendBaseVel(0.23,0,0);
                    place_arm();
                    ros::Duration(0.8).sleep();
                    sendBaseVel(0,0,0);
                    ros::Duration(1).sleep();
                    open_gripper();
                    ros::Duration(1).sleep();
                    sendBaseVel(-0.25,0,0);
                    reset_arm();
                    ros::Duration(0.8).sleep();
                    sendBaseVel(0,0,0);
                    return BT::NodeStatus::SUCCESS;
            }
        }        
        else if(nav_done&&detected==false)
        {
            sendBaseVel(0,0,0);
            ROS_INFO("no detected");
        }
        return BT::NodeStatus::RUNNING;   
    }
    void onHalted() override
    {
        
    }

private:
        void sendBaseVel(float x,float y,float yaw)
    {
        geometry_msgs::Twist twist;
        twist.linear.x = x;
        twist.linear.y = y;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = yaw;
        cmd_pub.publish(twist);
    }

    void place_arm()
    {
        geometry_msgs::Pose arm_position;
        arm_position.position.x = 0.18;
        arm_position.position.y = -0.03;
        arm_position_pub.publish(arm_position);
        for(int i=0;i<3;i++)
        {
            arm_gripper_pub.publish(arm_position.position);
            ros::Duration(0.05).sleep();
        }
    }
    void reset_arm()
    {
        geometry_msgs::Pose arm_position;
        arm_position.position.x = 0.1;
        arm_position.position.y = 0.12;
        arm_position_pub.publish(arm_position);
        for(int i=0;i<3;i++)
        {
            arm_gripper_pub.publish(arm_position.position);
            ros::Duration(0.05).sleep();
        }
    }
    void open_gripper()
    {
        geometry_msgs::Point point;
        point.x = 0.0;
        point.y = 0.0;
        point.z = 0.0;
        ROS_INFO("Opening gripper");
        for(int i=0;i<3;i++)
        {
            arm_gripper_pub.publish(point);
            ros::Duration(0.05).sleep();
        }
    }
    void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &_msg)
    {
        if(_msg->detections.empty())
            detected = false;
        else if(!_msg->detections.empty() && nav_done)
        {
            for(apriltag_ros::AprilTagDetection tag : _msg->detections)
            {
                if(fabs(tag.pose.pose.pose.position.x) < 0.01)
                {
                    mid_tag = tag;
                    mid_tag_id = tag.id[0];
                    std::cout << "mid tag is " << mid_tag_id << std::endl;
                }
            }
        }
    }
    ros::NodeHandle node_;
    std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> action_client_;
    ros::Subscriber tag_sub;
    apriltag_ros::AprilTagDetection mid_tag;  //tag on 'O'
    ros::Publisher arm_position_pub;
    ros::Publisher arm_gripper_pub;
    ros::Publisher cmd_pub;
    int mid_tag_id;
    bool reached_flag=false;
    bool detected=false;
    bool nav_done;
    vector<double> y_param = {1.1, 0.06, 0};    
    const float aim_set=0.045;
    PIDController y_pid;    
};

