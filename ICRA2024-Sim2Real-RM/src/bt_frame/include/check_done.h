#include <ros/ros.h>
#include "bt_frame/ep_goal.h"
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/Int32.h"
using namespace BT;
class Check_done:public StatefulActionNode
{
public:
    Check_done(ros::NodeHandle& Handle, const std::string& name, const NodeConfig& config):StatefulActionNode(name, config){}
    {
        nh = Handle;
        place_fns = false;
    }
    static PortsList providedPorts()
    {
        return {
            InputPort<int>("target_cube_num1"),
            InputPort<int>("target_cube_num2"),
            InputPort<int>("target_cube_num3")
        };
    }
    NodeStatus onStart() override
    {
        tag_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detection", 10, &Check_done::tag_callback, this);
        return BT::NodeStatus::RUNNING;
    }
    NodeStatus onRunning() override
    {
        if(place_fns)
            return BT::NodeStatus::SUCCESS;
        return BT::NodeStatus::RUNNING;
    }
    void onHalted() override
    {
        tag_sub.shutdown();
    }
private:
    void tag_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
    {
        if(msg->detections.size() != 0)
        {
            static int count = 0;
            tag_msg = *msg;
            auto tagId1 = getInput<int>("target_cube_num1");
            auto tagId2 = getInput<int>("target_cube_num2");
            auto tagId3 = getInput<int>("target_cube_num3");
            for(auto tag : tag_msg.detections)
                if(tag.id[0] == tagId1.value() || tag.id[0] == tagId2.value() || tag.id[0] == tagId3.value())
                    count++;
            if(count == 3)
            {
                place_fns = true;
                count = 0;
            }
        }
        else
        {
            place_fns = false;
        }
    }
    ros::NodeHandle nh;
    ros::Subscriber tag_sub;
    apriltag_ros::AprilTagDetectionArray tag_msg;
    bool place_fns;
};