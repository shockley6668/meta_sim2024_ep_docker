#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "apriltag_ros/AprilTagDetectionArray.h"

#include "ros/ros.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <vector>

#include "PIDController.h"
using namespace BT;
class Take : public StatefulActionNode
{
public:
    Take(ros::NodeHandle& Handle, const std::string& name, const NodeConfiguration& cfg) : StatefulActionNode(name, cfg)
    {
        wall_beside = false;
        node_ = Handle;
        cmd_pub = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 6);
        tag_detection_status_sub = node_.subscribe("/tag_detections", 2, &Take::poseCallback, this);
        tag_detection_pose = geometry_msgs::PoseWithCovarianceStamped();
        x_pid = PIDController(0, 0, 0, 0);
        y_pid = PIDController(0, 0, 0, 0);
        current_time = ros::Time::now();
        last_time = ros::Time(-1);
    }
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus onStart() override
    {
        ROS_INFO("Trying to take the block");
        return NodeStatus::RUNNING;
    }
    NodeStatus onRunning() override
    {
        if(!wall_beside)
        {
            double x_output = x_pid.calculate(tag_detection_pose.pose.pose.position.x, (current_time - last_time).toSec());
            double y_output = y_pid.calculate(tag_detection_pose.pose.pose.position.y, (current_time - last_time).toSec());
            geometry_msgs::Twist twist;
            twist.linear.x = x_output;
            twist.linear.y = y_output;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = 0;
            cmd_pub.publish(twist);
            ros::Duration(0.05).sleep();
        }
        
        return NodeStatus::RUNNING;
    }
    void onHalted() override
    {

    }
private:
    void poseCallback(const apriltag_ros::AprilTagDetectionArray& msg)
    {
        ROS_INFO("poseCallBack./");
        tag_detection_pose = msg.detections[0].pose;

        last_time = current_time;
        current_time = ros::Time::now();
        
        //determine if tag is near the wall


    }
    bool wall_beside;
    ros::NodeHandle node_;
    ros::Publisher cmd_pub;
    ros::Subscriber tag_detection_status_sub;
    geometry_msgs::PoseWithCovarianceStamped tag_detection_pose;
    ros::Time current_time;
    ros::Time last_time;

    PIDController x_pid, y_pid;
};