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
        x_pid = PIDController(1, 0, 0, 10);
        y_pid = PIDController(1, 0, 0, 10);
        z_pid = PIDController(1, 0, 0, 10);
        current_time = ros::Time::now();
        last_time = ros::Time(-1);
        tag_id = -1;
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
            double z_output = z_pid.calculate(tag_detection_pose.pose.pose.position.z, (current_time - last_time).toSec());
            geometry_msgs::Twist twist;
            twist.linear.x = x_output;
            twist.linear.y = y_output;
            twist.linear.z = z_output;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = 0;
            cmd_pub.publish(twist);
            ros::Duration(0.05).sleep();

            std::cout << "pid output in x : " << x_output << std::endl;
            std::cout << "pid output in y : " << y_output << std::endl;
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
        tag_id = msg.detections[0].id[0];//any other id

        last_time = current_time;
        current_time = ros::Time::now();
        
        std::cout << "distance from target block : " << std::endl;
        std::cout << "x : " << tag_detection_pose.pose.pose.position.x << std::endl;
        std::cout << "y : " << tag_detection_pose.pose.pose.position.y << std::endl;
        std::cout << "z : " << tag_detection_pose.pose.pose.position.z << std::endl;

        //determine if tag is near the wall


    }
    bool wall_beside;
    ros::NodeHandle node_;
    ros::Publisher cmd_pub;
    ros::Subscriber tag_detection_status_sub;
    geometry_msgs::PoseWithCovarianceStamped tag_detection_pose;
    ros::Time current_time;
    ros::Time last_time;
    int32_t tag_id;

    PIDController x_pid, y_pid, z_pid;
};