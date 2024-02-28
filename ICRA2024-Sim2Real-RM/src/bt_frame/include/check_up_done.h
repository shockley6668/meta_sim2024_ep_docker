#include <ros/ros.h>
#include "bt_frame/ep_goal.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/Int32.h"
#include "utility.h"
using namespace BT;
class Check_up_done:public StatefulActionNode
{
public:
    Check_up_done(ros::NodeHandle& Handle, const std::string& name, const NodeConfig& config):StatefulActionNode(name, config)
    {
        nh = Handle;
        taking_tag_id_pub = nh.advertise<std_msgs::Int32>("taking_tag_id", 10);
    }
    static PortsList providedPorts()
    {
        return {
            OutputPort<int>("arm_high"),
        };
    }
    NodeStatus onStart() override
    {
        tag_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 10, &Check_up_done::tag_callback, this);
        tf_listener_=std::make_shared<tf::TransformListener>();
        count=0;
        return BT::NodeStatus::RUNNING;
        
    }
    NodeStatus onRunning() override
    {
        count++;
        if(count>6)
        {
            UpdateTransform(tf_listener_, "map", "camera_color_optical_frame",ros::Time(0), cam_to_map);
            geometry_msgs::PoseStamped highest_tag_map_pose;
            geometry_msgs::PoseStamped highest_tag_pose;
            
            highest_tag_pose.pose = highest_tag.pose.pose.pose;

            TransformPose(cam_to_map,highest_tag_pose, highest_tag_map_pose);
            std::cout<<"highest_tag_id: "<<highest_tag.id[0]<<std::endl;
            std::cout << "highest_tag_map_posez: " << highest_tag_map_pose.pose.position.z << std::endl;
            if(highest_tag_map_pose.pose.position.z > 0.12)
            {
                setOutput<int>("arm_high", 2);
                return BT::NodeStatus::FAILURE;
            }
            else if(highest_tag_map_pose.pose.position.z > 0.07)
            {
                setOutput<int>("arm_high", 1);
                return BT::NodeStatus::FAILURE;
            }
            else
            {
                setOutput<int>("arm_high", 0);
                return BT::NodeStatus::FAILURE;
            }
            
        }
        else{
            return BT::NodeStatus::RUNNING;
        }
    }
    void onHalted() override
    {
        tag_sub.shutdown();
    }
private:
    void tag_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
    {
        //find highest tag
        apriltag_ros::AprilTagDetection t;
        for(int i = 0; i < msg->detections.size(); i++)
        {
            if(msg->detections[i].pose.pose.pose.position.y < t.pose.pose.pose.position.y)
            {
                t = msg->detections[i];
            }
            std_msgs::Int32 tag;
            tag.data = msg->detections[i].id[0];
            taking_tag_id_pub.publish(tag);
        }
        highest_tag = t;
    }
    ros::NodeHandle nh;
    ros::Subscriber tag_sub;
    tf::StampedTransform cam_to_map;
    std::shared_ptr<tf::TransformListener> tf_listener_;
    apriltag_ros::AprilTagDetection highest_tag;
    ros::Publisher taking_tag_id_pub;
    int fal_limit = 0;
    int count;
};