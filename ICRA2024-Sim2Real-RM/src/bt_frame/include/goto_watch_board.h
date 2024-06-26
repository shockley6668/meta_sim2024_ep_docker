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
#include <algorithm> 
using namespace BT;
typedef struct{
    float x;
    float y;
}Point2D;

// Template specialization to converts a string to Position2D.
namespace BT
{
    template <> inline vector<int> convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size()==0)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            vector<int> output;
            // output.x     = convertFromString<double>(parts[0]);
            // output.y     = convertFromString<double>(parts[1]);

            for(int i=0;i<parts.size();i++)
            {
                output.push_back(convertFromString<int>(parts[i]));
            }
            return output;
        }
    }
} // end namespace BT

class GotoWatchBoard : public StatefulActionNode
{
public:
    GotoWatchBoard(ros::NodeHandle& Handle,const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config), node_(Handle)
    {
        // Initialize the action client
        action_client_ = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
        
        reached_flag = false;

    }

    static BT::PortsList providedPorts()
    {
        return {
            OutputPort<int>("target_cube_num1"),
            OutputPort<int>("target_cube_num2"),
            OutputPort<int>("target_cube_num3")
        };
    }
    NodeStatus onStart() override
    {
        // Get the goal pose from the input port
        //const geometry_msgs::PoseStamped& goal = getInput<geometry_msgs::PoseStamped>("goal");
     
        second_goal_send=false;
        tag_sub=node_.subscribe("/tag_detections", 1, &GotoWatchBoard::tagCallback,this);
        // Create the goal message for the action
        ROS_INFO("Sending goal");
        while(!action_client_->waitForServer(ros::Duration(10.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
        }
        move_base_msgs::MoveBaseGoal goal_msg;
        goal_msg.target_pose.header.frame_id = "map";
        goal_msg.target_pose.header.stamp = ros::Time::now();
        goal_msg.target_pose.pose.position.x = 0.6;
        goal_msg.target_pose.pose.position.y = 0.8;
        goal_msg.target_pose.pose.orientation.w = 1.0;
        action_client_->sendGoal(goal_msg);
        return BT::NodeStatus::RUNNING;
    }
    NodeStatus onRunning() override
    {
        if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED&&second_goal_send==false)
        {
            ROS_INFO("success");
            ros::Duration(0.2).sleep();
            move_base_msgs::MoveBaseGoal goal_msg;
            goal_msg.target_pose.header.frame_id = "map";
            goal_msg.target_pose.header.stamp = ros::Time::now();
            goal_msg.target_pose.pose.position.x = 0;
            goal_msg.target_pose.pose.position.y = 1.37;
            goal_msg.target_pose.pose.orientation.w = 1.0;
            action_client_->sendGoal(goal_msg);
            second_goal_send=true;
            
        }
        action_client_->waitForResult();
        if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED&&second_goal_send==true)
        {
            reached_flag=true;
            if(target_cube_num.size()==3)
            {
                for (size_t i = 0; i < target_cube_num.size(); i++)
                {
                    cout<<target_cube_num[i]<<endl;
                }
                setOutput<int>("target_cube_num1",target_cube_num[0]);
                setOutput<int>("target_cube_num2",target_cube_num[1]);
                setOutput<int>("target_cube_num3",target_cube_num[2]);
                tag_sub.shutdown();
                return BT::NodeStatus::SUCCESS;
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

    void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &_msg)
    {
        if(_msg->detections.size()>=6&&reached_flag)
        {
            apriltag_ros::AprilTagDetectionArray temp;
            temp=*_msg;
            std::sort(temp.detections.begin(), temp.detections.end(), 
              [](const apriltag_ros::AprilTagDetection& det1, const apriltag_ros::AprilTagDetection& det2) {
                  return det1.pose.pose.pose.position.x < det2.pose.pose.pose.position.x;
              });
            for(size_t i=0; i<temp.detections.size();i++)
            {
                cout<<"tempid:"<<temp.detections[i].id[0]<<endl;
                if(temp.detections[i].id[0]<6 && temp.detections[i].pose.pose.pose.position.y<-0.2)
                {
                    for(int n:target_cube_num)
                    {
                        if(n==temp.detections[i].id[0])
                        {
                            continue;
                        }
                    }
                    target_cube_num.push_back(temp.detections[i].id[0]);
                }
            }
        }
    }
    ros::NodeHandle node_;
    std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> action_client_;
  
    ros::Subscriber tag_sub;
    bool reached_flag=false;
    vector<int> target_cube_num;
    vector<int> target_cube_num_sort;
    std::vector<Point2D> path_list;
    bool second_goal_send;
};

