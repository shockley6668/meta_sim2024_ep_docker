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
#include "utility.h"

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
        tf_listener_ = std::make_shared<tf::TransformListener>();
        tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();
        cmd_pub = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        arm_gripper_pub=node_.advertise<geometry_msgs::Point>("arm_gripper", 2);
        arm_position_pub=node_.advertise<geometry_msgs::Pose>("arm_position", 2);
        move_base_clear = node_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        simple_planner_pub=node_.advertise<geometry_msgs::PointStamped>("/clicked_point", 6);
        
        
        y_pid = PIDController(y_param);
        x_pid = PIDController(x_param);
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

        goal_status_sub_ = node_.subscribe("/cmd_vel", 2, &Place::goalStatusCallback, this);
        y_done=false;
        first_tag=false;
        detected=false;
        watchboard_pose.header.frame_id = "none";
        // Get the goal pose from the input port
        //const geometry_msgs::PoseStamped& goal = getInput<geometry_msgs::PoseStamped>("goal");
        GetGlobalRobotPose(tf_listener_, "map", robot_gobal_pose_);
        ros::Duration(0.5).sleep();
        if(robot_gobal_pose_.pose.position.y>2.4)
        {
            vector<float> goal_x={1.24,1.24,1.20,1.20,1.15};
            vector<float> goal_y={3.09,2.78,2.5,2.0,1.78};
            for(int i=0;i<5;i++)
            {
                geometry_msgs::PointStamped p;
                p.header.frame_id = "map";
                p.header.stamp = ros::Time::now();
                p.header.seq = i;
                p.point.x = goal_x[i];
                p.point.y = goal_y[i];
                simple_planner_pub.publish(p);
                ros::Duration(0.05).sleep();
            }
        }
        else if(robot_gobal_pose_.pose.position.y>0.45)
        {
            for(int i=0;i<5;i++)
            {
                geometry_msgs::PointStamped p;
                p.header.frame_id = "map";
                p.header.stamp = ros::Time::now();
                p.header.seq = i;
                p.point.x = 1.15;
                p.point.y = 1.78;
                simple_planner_pub.publish(p);
                ros::Duration(0.05).sleep();
            }
        }
        else{
            vector<float> goal_x={2.00,2.00,1.50,1,1.15};
            vector<float> goal_y={0.45,0.96,1.0,1,1.78};
            for(int i=0;i<5;i++)
            {
                geometry_msgs::PointStamped p;
                p.header.frame_id = "map";
                p.header.stamp = ros::Time::now();
                p.header.seq = i;
                p.point.x = goal_x[i];
                p.point.y = goal_y[i];
                simple_planner_pub.publish(p);
                ros::Duration(0.05).sleep();
            }
        }

        // move_base_msgs::MoveBaseGoal goal_msg;
        // sendBaseVel(0,0,0);
        // watchboard_pose.header.frame_id = "none";
        // y_done=false;
        // first_tag=false;
        // goal_msg.target_pose.header.frame_id = "map";
        // goal_msg.target_pose.header.stamp = ros::Time::now();
        // goal_msg.target_pose.pose.position.x = 1.18;
        // goal_msg.target_pose.pose.position.y = 1.78;
        // tf::Quaternion q=tf::createQuaternionFromYaw(0.0);
        // goal_msg.target_pose.pose.orientation.w = q.w();
        // goal_msg.target_pose.pose.orientation.x = q.x();
        // goal_msg.target_pose.pose.orientation.y = q.y();
        // goal_msg.target_pose.pose.orientation.z = q.z();
        // action_client_->sendGoal(goal_msg);

        
        
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
        
        
        if(nav_done==true)
        {
            
            std::cout<<"nav done"<<std::endl;
            tag_sub=node_.subscribe("/tag_detections", 10, &Place::tagCallback,this);
            sendBaseVel(0,0,0);
        }
        // else if(nav_done==false&&action_client_->getState() == actionlib::SimpleClientGoalState::ABORTED)
        // {
        //     std_srvs::Empty srv;
        //     move_base_clear.call(srv);
        //     std::cout<<"nav failed"<<std::endl;
        //     move_base_msgs::MoveBaseGoal goal_msg;
        //     goal_msg.target_pose.header.frame_id = "map";
        //     goal_msg.target_pose.header.stamp = ros::Time::now();
        //     goal_msg.target_pose.pose.position.x = 1.18;
        //     goal_msg.target_pose.pose.position.y = 1.78;
        //     tf::Quaternion q=tf::createQuaternionFromYaw(0.0);
        //     goal_msg.target_pose.pose.orientation.w = q.w();
        //     goal_msg.target_pose.pose.orientation.x = q.x();
        //     goal_msg.target_pose.pose.orientation.y = q.y();
        //     goal_msg.target_pose.pose.orientation.z = q.z();
        //     action_client_->sendGoal(goal_msg);
        // }
        if (nav_done&&detected&&y_done==false)
        {
            std::cout<<"box:"<<aim_tag.pose.pose.pose.position.x<<endl;
            if(abs(aim_set-aim_tag.pose.pose.pose.position.x)>0.01&&y_done==false)
            {
                float cmd_y=y_pid.calculate(aim_set, aim_tag.pose.pose.pose.position.x);
                sendBaseVel(0,cmd_y,0);
                
            }
            else{
                    sendBaseVel(0,0,0);
                    y_done=true;
                    
            }
            
        }

        if(y_done==true)
        {
            GetGlobalRobotPose(tf_listener_, "map", robot_gobal_pose_);
            if(detected&&first_tag==false)
            {
                ros::Duration(0.5).sleep();
                tf::Transform transform;
                tf::poseMsgToTF(aim_tag.pose.pose.pose, transform);
                tf_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_color_optical_frame", "watchboard_aim_tag"));
                tf::Stamped<tf::Pose> watchboard_map_tf;
                tf::Stamped<tf::Pose> watchboard_tf;
                watchboard_tf.setIdentity();
                watchboard_tf.frame_id_ = "watchboard_aim_tag";
                watchboard_tf.stamp_ = ros::Time();
                try{
                    tf_listener_->waitForTransform("map", "watchboard_aim_tag", ros::Time(0), ros::Duration(0.5));
                    tf_listener_->transformPose( "map", watchboard_tf, watchboard_map_tf);
                }
                catch (tf::TransformException &ex) {
                    ROS_ERROR("Failed to transform watchboard_aim_tag pose: %s", ex.what());
                }
                tf::poseStampedTFToMsg(watchboard_map_tf, watchboard_pose);
                first_tag=true;
            }
            if(first_tag)
            {
                float target_x=watchboard_pose.pose.position.x-0.28;
                std::cout<<"target_x"<<target_x<<std::endl;
                if(abs(target_x-robot_gobal_pose_.pose.position.x)>0.01)
                {
                    float cmd_x=x_pid.calculate(target_x, robot_gobal_pose_.pose.position.x);
                    sendBaseVel(cmd_x,0,0);
                }
                else{
                    sendBaseVel(0,0,0);
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
                    move_base_msgs::MoveBaseGoal goal_msg;
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
                    ros::Duration(2).sleep();
                    tag_sub.shutdown();
                    goal_status_sub_.shutdown();
                    return BT::NodeStatus::SUCCESS;
                }
                
                
            }
            
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
    void goalStatusCallback(const geometry_msgs::Twist& msg)
    {
        
        if (nav_done==false&&msg.linear.z == 1)
        {
            ROS_INFO("simple planner Goal reached");
            nav_done=true;
        }
    }
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
        arm_position.position.y = -0.05;
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
    geometry_msgs::PoseStamped watchboard_pose;
    std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf::TransformListener> tf_listener_;
    geometry_msgs::PoseStamped robot_gobal_pose_;
    ros::ServiceClient move_base_clear;
    ros::Publisher simple_planner_pub;
    bool goal_reached;
    int aim_tag_id;
    ros::Publisher cmd_pub;

    bool detected=false;
    bool nav_done;
    bool y_done;
    bool first_tag;
    vector<double> x_param = {1, 0.01, 0.01};
    vector<double> y_param = {1, 0.01, 0.01};
    int take_cube_num;
    const float aim_set=0.045;
    PIDController y_pid;
    PIDController x_pid;
};

