#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "ros/ros.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "PIDController.h"
#include "utility.h"
#include <tf/transform_broadcaster.h>
using namespace BT;
class Take_Up : public StatefulActionNode
{
public:
    Take_Up(ros::NodeHandle& Handle, const std::string& name, const NodeConfiguration& cfg) : StatefulActionNode(name, cfg)
    {
        node_ = Handle;
        
        cmd_pub = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        tag_detection_pose = geometry_msgs::PoseWithCovarianceStamped();
        arm_position_pub=node_.advertise<geometry_msgs::Pose>("arm_position", 2);
        arm_gripper_pub=node_.advertise<geometry_msgs::Point>("arm_gripper", 2);
        no_move_pub=node_.advertise<std_msgs::Int32>("no_move", 10);
        x_pid = PIDController(x_param);
        y_pid = PIDController(y_param);
        z_pid = PIDController(z_param);
        tf_listener_ = std::make_shared<tf::TransformListener>();
        take_node_state=0;
        // target_cube_num_pub=node_.advertise<std_msgs::Int32MultiArray>("target_cube_num", 10);

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
    void close_gripper()
    {
        geometry_msgs::Point point;
        point.x = 1.0;
        point.y = 0.0;
        point.z = 0.0;
        ROS_INFO("Closing gripper");
        for(int i=0;i<3;i++)
        {
            arm_gripper_pub.publish(point);
            ros::Duration(0.05).sleep();
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
    void positionStateCallback(const std_msgs::Int32::ConstPtr & msg)
    {
        if(msg->data==3)
        {
            difilute_position = true;
        }
        else
        {
            difilute_position = false;
        }
    }

    void poseCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr & msg)
    {
        if(msg->detections.empty())
        {
            ROS_INFO("No blocks are currently detected!!");
            block_detected = false;
            return;
        }
        else
        {
            
            if(take_node_state==0)
            {
                //第一次进入，找到目标方块
                if(tag_id==-1)
                {
                    for(size_t i = 0; i < msg->detections.size(); i++)
                    {
                        tag_id = msg->detections[i].id[0];
                        break;
                    }
                }
                //第二次进入，锁定目标方块
                else
                {
                    vector<int> detected_id;
                    for(size_t i = 0; i < msg->detections.size(); i++)
                    {
                        if(msg->detections[i].id[0]==tag_id)
                        {
                            detected_id.push_back(i);
                        }
                    }
                    if(detected_id.size()==0)
                    {
                        ROS_INFO("The target block is not detected!!");
                        block_detected = false;
                        return;
                    }
                    else if(detected_id.size()==1)
                    {
                        tag_detection_pose = msg->detections[detected_id[0]].pose;
                        block_detected = true;
                    }
                    else if(detected_id.size()==2)
                    {
                        tag_detection_pose.header = msg->detections[detected_id[0]].pose.header;
                        tag_detection_pose.pose.pose.position.x = (msg->detections[detected_id[0]].pose.pose.pose.position.x + msg->detections[detected_id[1]].pose.pose.pose.position.x)/2;
                        tag_detection_pose.pose.pose.position.y = (msg->detections[detected_id[0]].pose.pose.pose.position.y + msg->detections[detected_id[1]].pose.pose.pose.position.y)/2;
                        tag_detection_pose.pose.pose.position.z = (msg->detections[detected_id[0]].pose.pose.pose.position.z + msg->detections[detected_id[1]].pose.pose.pose.position.z)/2;
                        block_detected = true;
                    }
                    
                }
                
            }
            
        }
    }
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus onStart() override
    {
        ROS_INFO("Trying to take the block");
        // target_cube_num={tu1.value(),tu2.value(),tu3.value()};
        
        tag_detection_status_sub = node_.subscribe("/tag_detections", 10, &Take_Up::poseCallback, this);
        position_state_sub=node_.subscribe("/position_state", 10, &Take_Up::positionStateCallback, this);
        tag_id = -1;
        y_done=false;
        no_need_block_time=0;
        block_detected = false;
        wall_beside = false;
        difilute_position=false;
        return NodeStatus::RUNNING;
    }
    NodeStatus onRunning() override
    {
        std_msgs::Int32 t;
        t.data=1;
        no_move_pub.publish(t);
        if(tag_id==-1)
        {
            no_need_block_time++;
        }
        if(no_need_block_time>30)
        {
            tag_detection_status_sub.shutdown();
            cout<<"No need block"<<endl;
            return NodeStatus::FAILURE;
        }
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
            if(difilute_position)
            {
                ROS_INFO("The robot is in difilute position, please move the robot to the correct position!!");
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(tag_detection_pose.pose.pose.position.x, tag_detection_pose.pose.pose.position.y, tag_detection_pose.pose.pose.position.z));
                tf::Quaternion q(tag_detection_pose.pose.pose.orientation.x, tag_detection_pose.pose.pose.orientation.y, tag_detection_pose.pose.pose.orientation.z, tag_detection_pose.pose.pose.orientation.w);
                transform.setRotation(q);
                tf_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_color_optical_frame", "target_tag"));
                
            }

            double y_output;
            double x_output;
            if(abs(tag_detection_pose.pose.pose.position.x-y_target)>0.005&&y_done==false)
            {
                y_output = y_pid.calculate(y_target, tag_detection_pose.pose.pose.position.x);
            }
            else{
                y_output=0;
                y_done=true;
            }
            if(abs(tag_detection_pose.pose.pose.position.z-x_target)>0.06&&y_done)
            {
                x_output = -x_pid.calculate(x_target, tag_detection_pose.pose.pose.position.z);
            }
            else if(abs(tag_detection_pose.pose.pose.position.z-x_target)<=0.06&&y_done) 
            {
                geometry_msgs::Twist twist;
                x_output=0;
                twist.linear.x = 0;
                twist.linear.y = 0;
                cmd_pub.publish(twist);
                open_gripper();
                ros::Duration(0.7).sleep();
                geometry_msgs::Pose arm_position;
                arm_position.position.x = 0.19;
                arm_position.position.y = -0.08;
                arm_position_pub.publish(arm_position);
                ros::Duration(2).sleep();
                sendBaseVel(0.25,0,0);
                ros::Duration(0.4).sleep();
                sendBaseVel(0,0,0);
                ros::Duration(1).sleep();
                close_gripper();
                ros::Duration(1).sleep();
                reset_arm();
                ros::Duration(1).sleep();
                tag_detection_status_sub.shutdown();
                return NodeStatus::SUCCESS;
            }
            geometry_msgs::Twist twist;
            twist.linear.x = x_output;
            twist.linear.y = y_output;
            cmd_pub.publish(twist);
            ros::Duration(0.05).sleep();
            std::cout << "x : " << tag_detection_pose.pose.pose.position.x << std::endl;
        }
        
        return NodeStatus::RUNNING;
    }
    void onHalted() override
    {

    }
private:
    bool wall_beside;
    bool block_detected;
    bool y_done;
    bool difilute_position;
    std::shared_ptr<tf::TransformListener> tf_listener_;
    ros::NodeHandle node_;
    ros::Publisher cmd_pub;
    ros::Subscriber tag_detection_status_sub;
    geometry_msgs::PoseWithCovarianceStamped tag_detection_pose;
    tf::StampedTransform targetTag2map_transform;
    std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
    ros::Publisher arm_gripper_pub;
    ros::Publisher arm_position_pub;
    ros::Publisher no_move_pub;
    // ros::Publisher target_cube_num_pub;
    ros::Subscriber position_state_sub;
    int tag_id;
    int take_node_state;

    const float x_target = 0.18;
    const float y_target = 0.045;
    int no_need_block_time;

    PIDController x_pid, y_pid, z_pid;
    vector <int> target_cube_num;
    vector<double> x_param = {1.1, 0.06, 0};
    vector<double> y_param = {1.1, 0.06, 0};
    vector<double> z_param = {1, 0, 0};

};