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
using namespace BT;
typedef struct{
    float x;
    float y;
}Point2D;
using namespace std;
class GotoPlaceBlock : public StatefulActionNode
{
public:
    GotoPlaceBlock(ros::NodeHandle& Handle,const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config), node_(Handle)
    {
        // Initialize the action client
        action_client_ = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
        goal_pub_ = node_.advertise<geometry_msgs::PointStamped>("/clicked_point", 6);
        goal_status_sub_ = node_.subscribe("/cmd_vel", 2, &GotoPlaceBlock::goalStatusCallback, this);
        tag_sub=node_.subscribe("/tag_detections", 1, &GotoPlaceBlock::tagCallback,this);
        reached_flag = false;
   
        path_list={{0.226,0},{0.4,0},{0.4,0.8},{0.3,1.25},{0,1.5}};
    }

    static BT::PortsList providedPorts()
    {
        return {
            OutputPort<int>("target_cube_num1"),
            OutputPort<int>("target_cube_num2"),
            OutputPort<int>("target_cube_num3")
        };
    }

    // BT::NodeStatus tick() override
    // {
    //     // Get the goal pose from the input port
    //     //const geometry_msgs::PoseStamped& goal = getInput<geometry_msgs::PoseStamped>("goal");

    //     // Create the goal message for the action
    //     ROS_INFO("Sending goal");
    //     while(!action_client_->waitForServer(ros::Duration(5.0))){
    //     ROS_INFO("Waiting for the move_base action server to come up");
    //     }
    //     move_base_msgs::MoveBaseGoal goal_msg;
    //     goal_msg.target_pose.header.frame_id = "map";
    //     goal_msg.target_pose.header.stamp = ros::Time::now();
    //     goal_msg.target_pose.pose.position.x = 0;
    //     goal_msg.target_pose.pose.position.y = 1.37;
    //     goal_msg.target_pose.pose.orientation.w = 1.0;
        
    //     // Send the goal to the action server
    //     action_client_->sendGoal(goal_msg);

    //     // Wait for the action to complete
    //     action_client_->waitForResult();

    //     // Check the result of the action
    //     if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //     {
    //         ROS_INFO("success");
    //         return BT::NodeStatus::SUCCESS;
    //     }
    //     else
    //     {
    //         ROS_INFO("failure");
    //         return BT::NodeStatus::FAILURE;
    //     }
    // }
    NodeStatus onStart() override
    {
        // Get the goal pose from the input port
        //const geometry_msgs::PoseStamped& goal = getInput<geometry_msgs::PoseStamped>("goal");

        // Create the goal message for the action
        ROS_INFO("Sending goal");
        ros::Duration(0.5).sleep();
        for(int i=0;i<path_list.size();i++){  
            geometry_msgs::PointStamped p;
            p.header.frame_id = "map";
            p.header.stamp = ros::Time::now();
            p.header.seq = i;
            p.point.x = path_list[i].x;
            p.point.y = path_list[i].y;
            goal_pub_.publish(p);
            ros::Duration(0.05).sleep();
        }
        return BT::NodeStatus::RUNNING;
    }
    NodeStatus onRunning() override
    {
        
        if(reached_flag&&target_cube_num.size()==3){
            //reached_flag=false;
            for (size_t i = 0; i < target_cube_num.size(); i++)
            {
                cout<<target_cube_num[i]<<endl;
            }
            setOutput<int>("target_cube_num1",target_cube_num[0]);
            setOutput<int>("target_cube_num2",target_cube_num[1]);
            setOutput<int>("target_cube_num3",target_cube_num[2]);
            
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::RUNNING;
    }
    void onHalted() override
    {
      // nothing to do here...
      std::cout << "goto_watchboard interrupted" << std::endl;
    }

private:
    void goalStatusCallback(const geometry_msgs::Twist& msg)
    {
        
        if (msg.linear.z == 1)
        {
            ROS_INFO("Goal reached");
            reached_flag=true;
        }
    }
    void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &_msg)
    {
        if(_msg->detections.size()>=3&&reached_flag)
        {
            
            for(size_t i=0; i<_msg->detections.size();i++)
            {
                if(_msg->detections[i].id[0]<6)
                {
                    for(int n:target_cube_num)
                    {
                        if(n==_msg->detections[i].id[0])
                        {
                            continue;
                        }
                    }
                    target_cube_num.push_back(_msg->detections[i].id[0]);
                }
            }
        }
    }
    ros::NodeHandle node_;
    std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> action_client_;
    ros::Publisher goal_pub_;
    ros::Subscriber goal_status_sub_;
    ros::Subscriber tag_sub;
    bool reached_flag=false;
    vector<int> target_cube_num;
    std::vector<Point2D> path_list;

};

