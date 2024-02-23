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
        block_detected = false;
        node_ = Handle;
        cmd_pub = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        tag_detection_pose = geometry_msgs::PoseWithCovarianceStamped();
        tag_detection_status_sub = node_.subscribe("/tag_detections", 10, &Take::poseCallback, this);
        x_pid = PIDController(x_param);
        y_pid = PIDController(y_param);
        z_pid = PIDController(z_param);
        tag_id = -1;
    }
    void poseCallback(const apriltag_ros::AprilTagDetectionArray& msg)
    {
        ROS_INFO("poseCallBack./");
        if(msg.detections.empty())
        {
            ROS_INFO("No blocks are currently detected!!");
            block_detected = false;
            return;
        }
        else
        {
            block_detected = true;
            tag_detection_pose = msg.detections[0].pose;
            std::cout << "x : " << tag_detection_pose.pose.pose.position.x << std::endl;
            std::cout << "y : " << tag_detection_pose.pose.pose.position.y << std::endl;
            std::cout << "z : " << tag_detection_pose.pose.pose.position.z << std::endl;
        }

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
        if(!block_detected)
        {
            geometry_msgs::Twist twist;
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = 0;
            cmd_pub.publish(twist);
            ros::Duration(0.05).sleep();
        }
        if(!wall_beside && block_detected)
        {
            double y_output = -y_pid.calculate(-0.1, tag_detection_pose.pose.pose.position.x);
            double x_output = -x_pid.calculate(0.2, tag_detection_pose.pose.pose.position.z);

            tf2::Quaternion quaternion(tag_detection_pose.pose.pose.orientation.x, 
                                        tag_detection_pose.pose.pose.orientation.y, 
                                        tag_detection_pose.pose.pose.orientation.z, 
                                        tag_detection_pose.pose.pose.orientation.w); 
            tf2::Matrix3x3 mat(quaternion);
            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);
            double z_output = z_pid.calculate(0, yaw);

            geometry_msgs::Twist twist;
            twist.linear.x = x_output;
            twist.linear.y = y_output;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = z_output * 0.8;
            cmd_pub.publish(twist);
            ros::Duration(0.05).sleep();

            std::cout << "pid output in x : " << x_output << std::endl;
            std::cout << "pid output in y : " << y_output << std::endl;
            std::cout << "pid output in z : " << z_output << std::endl;
        }
        
        return NodeStatus::RUNNING;
    }
    void onHalted() override
    {

    }
private:
    bool wall_beside;
    bool block_detected;
    ros::NodeHandle node_;
    ros::Publisher cmd_pub;
    ros::Subscriber tag_detection_status_sub;
    geometry_msgs::PoseWithCovarianceStamped tag_detection_pose;
    int32_t tag_id;

    PIDController x_pid, y_pid, z_pid;
    vector<double> x_param = {1, 0, 0};
    vector<double> y_param = {1, 0, 0};
    vector<double> z_param = {1, 0, 0};

};