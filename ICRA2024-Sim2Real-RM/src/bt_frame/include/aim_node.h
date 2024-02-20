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

#define KP 0
#define KI 1
#define KD 2

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
        x_param = {1.0, 0.f, 0.f};
        y_param = {1.0, 0.f, 0.f};
        z_param = {1.0, 0.f, 0.f};
        node_.param<double>("xp", x_param[KP], 0.1);
        node_.param<double>("xi", x_param[KI], 0.0);
        node_.param<double>("xd", x_param[KD], 0.0);

        node_.param<double>("yp", y_param[KP], 0.1);
        node_.param<double>("yi", y_param[KI], 0.0);
        node_.param<double>("yd", y_param[KD], 0.0);

        node_.param<double>("zp", z_param[KP], 0.1);
        node_.param<double>("zi", z_param[KI], 0.0);
        node_.param<double>("zd", z_param[KD], 0.0);

        x_pid = PIDController(x_param);
        y_pid = PIDController(y_param);
        z_pid = PIDController(z_param);

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
            double x_output = x_pid.calculate(0, tag_detection_pose.pose.pose.position.x);
            double y_output = y_pid.calculate(0, tag_detection_pose.pose.pose.position.y);
            double z_output = z_pid.calculate(0, tag_detection_pose.pose.pose.position.z);
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
            std::cout << "pid output in z : " << z_output << std::endl;
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
    int32_t tag_id;

    PIDController x_pid, y_pid, z_pid;
    vector<double> x_param, y_param, z_param;
};