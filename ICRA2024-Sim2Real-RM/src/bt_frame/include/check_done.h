#include <ros/ros.h>
#include "bt_frame/ep_goal.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/Int32.h"
using namespace BT;
class Check_done:public StatefulActionNode
{
public:
    Check_done(ros::NodeHandle& Handle, const std::string& name, const NodeConfig& config):StatefulActionNode(name, config)
    {
        nh = Handle;
        place_fns = false;
        check_done_pub=nh.advertise<std_msgs::Int32>("/check_done", 10);
        taking_tag_id_pub = nh.advertise<std_msgs::Int32>("taking_tag_id", 10);
    }
    static PortsList providedPorts()
    {
        return {
            InputPort<int>("target_cube_num1"),
            InputPort<int>("target_cube_num2"),
            InputPort<int>("target_cube_num3"),
        };
    }
    NodeStatus onStart() override
    {
        tag_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 10, &Check_done::tag_callback, this);
        count=0;
        return BT::NodeStatus::RUNNING;
        
    }
    NodeStatus onRunning() override
    {
        count++;
        std::cout << "checking if blocks are ready\n";
        if(place_fns)
        {
            tag_sub.shutdown();
            ROS_INFO("blocks are ready");
            for(int i = 0; i < 3; i++)
            {
                std_msgs::Int32 msg;
                msg.data = 1;
                check_done_pub.publish(msg);
            }
            return BT::NodeStatus::SUCCESS;
        }
        ROS_INFO("continue searching block");
        if(count>20)
        {
            std_msgs::Int32 msg;
            msg.data = 0;
            check_done_pub.publish(msg);
            tag_sub.shutdown();
            return BT::NodeStatus::FAILURE;
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
        std::cout << msg->detections.size() << " blocks detected\n";
        if(msg->detections.size() >= 3)
        {
            int count = 0;
            tag_msg = *msg;
            auto tagId1 = getInput<int>("target_cube_num1");
            auto tagId2 = getInput<int>("target_cube_num2");
            auto tagId3 = getInput<int>("target_cube_num3");
            bool one_ready, two_ready, three_ready;
            one_ready = two_ready = three_ready = false;
            for(auto tag : tag_msg.detections)
            {
                if(tagId1 == tag.id[0])
                {
                    one_ready = true;
                    std_msgs::Int32 tag;
                    tag.data = tagId1.value();
                    taking_tag_id_pub.publish(tag);
                    std::cout << "1\n";
                    count++;
                }
                if(tagId2 == tag.id[0])
                {
                    two_ready = true;
                    std_msgs::Int32 tag;
                    tag.data = tagId2.value();
                    taking_tag_id_pub.publish(tag);
                    std::cout << "2\n";
                    count++;
                }
                if(tagId3 == tag.id[0])
                {
                    three_ready = true;
                    std_msgs::Int32 tag;
                    tag.data = tagId3.value();
                    taking_tag_id_pub.publish(tag);
                    std::cout << "3\n";
                    count++;
                }
            }
            std::cout << "now " << count << " blocks are ready\n";
            if(one_ready && two_ready && three_ready)
                place_fns = true;
        }
        else
        {
            place_fns = false;
        }
    }
    ros::NodeHandle nh;
    ros::Subscriber tag_sub;
    ros::Publisher check_done_pub;
    apriltag_ros::AprilTagDetectionArray tag_msg;
    ros::Publisher taking_tag_id_pub;
    int fal_limit = 0;
    bool place_fns;
    int count;
};