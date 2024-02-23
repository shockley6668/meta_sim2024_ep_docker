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
using namespace std;
class Place : public StatefulActionNode
{
public:
    Place(ros::NodeHandle& Handle,const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config), node_(Handle)
    {
        // Initialize the action client
        action_client_ = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
        cmd_pub = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        arm_gripper_pub=node_.advertise<geometry_msgs::Point>("arm_gripper", 2);
        arm_position_pub=node_.advertise<geometry_msgs::Pose>("arm_position", 2);
        reached_flag = false;
        detected=false;
        y_pid = PIDController(y_param);
        take_cube_num=-1;
        aim_tag_id=-1;
    }

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<int>("takeing_cube_num"),
            InputPort<int>("target_cube_num1"),
            InputPort<int>("target_cube_num2"),
            InputPort<int>("target_cube_num3")
        };
    }

    NodeStatus onStart() override
    {
        // Get the goal pose from the input port
        //const geometry_msgs::PoseStamped& goal = getInput<geometry_msgs::PoseStamped>("goal");
        
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
        tag_sub=node_.subscribe("/tag_detections", 1, &Place::tagCallback,this);
        auto res=getInput<int>("takeing_cube_num");
        take_cube_num=res.value();
        auto res1=getInput<int>("target_cube_num1");
        if( !res1 )
        {
            throw RuntimeError("error reading port [target]:", res1.error());
        }
        auto res2=getInput<int>("target_cube_num2");
        if( !res2 )
        {
            throw RuntimeError("error reading port [target]:", res2.error());
        }
        auto res3=getInput<int>("target_cube_num3");
        if( !res3 )
        {
            throw RuntimeError("error reading port [target]:", res3.error());
        }
        if(take_cube_num==res1.value())
        {
            aim_tag_id=6;
        }
        if(take_cube_num==res2.value())
        {
            aim_tag_id=7;
        }
        if(take_cube_num==res3.value())
        {
            aim_tag_id=8;
        }
        nav_done=false;

        std::cout<<"take_cube_num:"<<take_cube_num<<std::endl;
        std::cout<<"aimtagid:"<<aim_tag_id<<std::endl;
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
            std::cout<<"box:"<<aim_tag.pose.pose.pose.position.x<<endl;
            if(abs(aim_set-aim_tag.pose.pose.pose.position.x)>0.01)
            {
                float cmd_y=y_pid.calculate(aim_set, aim_tag.pose.pose.pose.position.x);
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
      // nothing to do here...
      tag_sub.shutdown();
      std::cout << "goto_watchboard interrupted" << std::endl;
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
        if(_msg->detections.size()>=1&&aim_tag_id!=-1)
        {
            
            for(size_t i=0; i<_msg->detections.size();i++)
            {
                if(_msg->detections[i].id[0]==aim_tag_id)
                {
                    aim_tag=_msg->detections[i];
                    detected=true;
                    std::cout<<aim_tag.pose.pose.pose.position.x<<std::endl;
                    break;
                }
                else{
                    detected=false;
                
                }
            }
        }
        else{
            detected=false;
        }
    }
    ros::NodeHandle node_;
    std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> action_client_;
    ros::Publisher goal_pub_;
    ros::Subscriber goal_status_sub_;
    ros::Subscriber tag_sub;
    apriltag_ros::AprilTagDetection aim_tag;
    ros::Publisher arm_position_pub;
    ros::Publisher arm_gripper_pub;
    int aim_tag_id;
    ros::Publisher cmd_pub;
    bool reached_flag=false;
    bool detected=false;
    bool nav_done;
    vector<double> y_param = {1.1, 0.06, 0};
    int take_cube_num;
    const float aim_set=0.045;
    PIDController y_pid;
};

