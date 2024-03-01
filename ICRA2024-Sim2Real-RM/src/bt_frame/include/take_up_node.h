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
#include "kalmanFilter.h"
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
        tf_broadcaster_=std::make_shared<tf::TransformBroadcaster>();
        tf_listener_ = std::make_shared<tf::TransformListener>();
        take_node_state=0;
        target_cube_num_pub=node_.advertise<std_msgs::Int32MultiArray>("target_cube_num", 10);
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
    // void tartagCallback(const std_msgs::Int32::ConstPtr & msg)
    // {
    //     tag_id = msg->data;
    // }

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
            else{

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
                }
                else if(detected_id.size()==1)
                {

                    tag_detection_pose = msg->detections[detected_id[0]].pose;
                    block_detected = true;
                    tag_kalman_filter.Predict();
                    tag_kalman_filter.Update(Eigen::Vector2d(tag_detection_pose.pose.pose.position.x,tag_detection_pose.pose.pose.position.z));
                    tag_detection_pose.pose.pose.position.x = tag_kalman_filter.GetState()(0);
                    tag_detection_pose.pose.pose.position.z = tag_kalman_filter.GetState()(1);
                }
                else if(detected_id.size()==2)
                {
                    tag_detection_pose.header = msg->detections[detected_id[0]].pose.header;
                    tag_detection_pose.pose.pose.position.x = (msg->detections[detected_id[0]].pose.pose.pose.position.x + msg->detections[detected_id[1]].pose.pose.pose.position.x)/2;
                    tag_detection_pose.pose.pose.position.y = (msg->detections[detected_id[0]].pose.pose.pose.position.y + msg->detections[detected_id[1]].pose.pose.pose.position.y)/2;
                    tag_detection_pose.pose.pose.position.z = (msg->detections[detected_id[0]].pose.pose.pose.position.z + msg->detections[detected_id[1]].pose.pose.pose.position.z)/2;
                    tag_detection_pose.pose.pose.orientation.x = msg->detections[detected_id[0]].pose.pose.pose.orientation.x;
                    tag_detection_pose.pose.pose.orientation.y = msg->detections[detected_id[0]].pose.pose.pose.orientation.y;
                    tag_detection_pose.pose.pose.orientation.z = msg->detections[detected_id[0]].pose.pose.pose.orientation.z;
                    tag_detection_pose.pose.pose.orientation.w = msg->detections[detected_id[0]].pose.pose.pose.orientation.w;
                    block_detected = true;
                    tag_kalman_filter.Predict();
                    tag_kalman_filter.Update(Eigen::Vector2d(tag_detection_pose.pose.pose.position.x,tag_detection_pose.pose.pose.position.z));
                    tag_detection_pose.pose.pose.position.x = tag_kalman_filter.GetState()(0);
                    tag_detection_pose.pose.pose.position.z = tag_kalman_filter.GetState()(1);
                }
                
            }

            

        }

    }
    static PortsList providedPorts()
    {
        return {
            // InputPort<int>("target_cube_num1"),
            // InputPort<int>("target_cube_num2"),
            // InputPort<int>("target_cube_num3"),
            // OutputPort<int>("takeing_cube_num"),
        };
    }
    NodeStatus onStart() override
    {
        ROS_INFO("Trying to take the block");
        // auto tu1=getInput<int>("target_cube_num1");
        // auto tu2=getInput<int>("target_cube_num2");
        // auto tu3=getInput<int>("target_cube_num3");
        // target_cube_num={tu1.value(),tu2.value(),tu3.value()};

        tag_detection_status_sub = node_.subscribe("/tag_detections", 10, &Take_Up::poseCallback, this);
        position_state_sub=node_.subscribe("/position_state", 10, &Take_Up::positionStateCallback, this);
        // tar_tag_sub = node_.subscribe("tar_tag", 10, &Take_Up::tartagCallback, this);

        target_tag_map_pose.header.frame_id = "none";
        tag_id = -1;
        y_done=false;
        no_need_block_time=0;
        block_detected = false;
        wall_beside = false;
        difilute_position=false;
        map_tag_clear_pid=false;
        tag_kalman_filter=KalmanFilter2D(Eigen::Vector2d(0,0),0.01,0.02);
        return NodeStatus::RUNNING;
    }
    NodeStatus onRunning() override
    {
        std_msgs::Int32 t;
        t.data=1;
        no_move_pub.publish(t);
        std_msgs::Int32MultiArray target_cube_num_msg;
        target_cube_num_msg.data=target_cube_num;
        target_cube_num_pub.publish(target_cube_num_msg);
        if(tag_id==-1||block_detected==false)
        {
            no_need_block_time++;
        }
        if(no_need_block_time>30)
        {
            tag_detection_status_sub.shutdown();
            cout<<"No need block"<<endl;
            return NodeStatus::FAILURE;
        }
        
        if(!wall_beside && block_detected)
        {   
            tf::Transform transform;

            transform.setOrigin(tf::Vector3(tag_detection_pose.pose.pose.position.x, tag_detection_pose.pose.pose.position.y, tag_detection_pose.pose.pose.position.z));
            tf::Quaternion q(tag_detection_pose.pose.pose.orientation.x, tag_detection_pose.pose.pose.orientation.y, tag_detection_pose.pose.pose.orientation.z, tag_detection_pose.pose.pose.orientation.w);
            std::cout<<tag_detection_pose.pose.pose.orientation.x<<" "<<tag_detection_pose.pose.pose.orientation.y<<" "<<tag_detection_pose.pose.pose.orientation.z<<" "<<tag_detection_pose.pose.pose.orientation.w<<std::endl;
            transform.setRotation(q);

            tf_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_color_optical_frame", "target_tag"));

            tf::Stamped<tf::Pose> target_tag_map_pose_tf;
            tf::Stamped<tf::Pose> target_tag_pose_tf;
            target_tag_pose_tf.setIdentity();
            target_tag_pose_tf.frame_id_ = "target_tag";
            target_tag_pose_tf.stamp_ = ros::Time();
            try{
                tf_listener_->waitForTransform( "map", "target_tag", ros::Time(0), ros::Duration(0.5));
                tf_listener_->transformPose( "map", target_tag_pose_tf, target_tag_map_pose_tf);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("Failed to transform robot pose: %s", ex.what());
            }

            tf::poseStampedTFToMsg(target_tag_map_pose_tf, target_tag_map_pose);
            std::cout<<"map_x: "<<target_tag_map_pose.pose.position.x<<" map_y"<<target_tag_map_pose.pose.position.y<<std::endl;

            // if(1)
            // {
            //     ROS_INFO("The robot is in difilute position, please move the robot to the correct position!!");

            // }

            double y_output;
            double x_output;
            if(abs(tag_detection_pose.pose.pose.position.x-y_target)>0.005&&y_done==false)
            {
                y_output = y_pid.calculate(y_target, tag_detection_pose.pose.pose.position.x);
                //limit
                if(y_output>0.3)
                {
                    y_output=0.3;
                }
                else if(y_output<-0.3)
                {
                    y_output=-0.3;
                }

            }
            else{
                y_output=0;
                y_done=true;
            }
            if(abs(tag_detection_pose.pose.pose.position.z-x_target)>0.01&&y_done)
            {
                x_output = -x_pid.calculate(x_target, tag_detection_pose.pose.pose.position.z);
                // x_output=0;
            }
            else if(abs(tag_detection_pose.pose.pose.position.z-x_target)<=0.01&&y_done) 
            {
                geometry_msgs::Twist twist;
                x_output=0;
                twist.linear.x = 0;
                twist.linear.y = 0;
                cmd_pub.publish(twist);
                open_gripper();
                ros::Duration(0.7).sleep();
                // setOutput<int>("takeing_cube_num",tag_id);
                geometry_msgs::Pose arm_position;
                arm_position.position.x = 0.19;
                arm_position.position.y = -0.08;
                arm_position_pub.publish(arm_position);
                ros::Duration(2).sleep();
                // sendBaseVel(0.25,0,0);
                // ros::Duration(0.4).sleep();
                sendBaseVel(0,0,0);
                ros::Duration(1).sleep();
                close_gripper();
                ros::Duration(1).sleep();
                reset_arm();
                ros::Duration(1).sleep();
                tag_detection_status_sub.shutdown();
                return NodeStatus::SUCCESS;
            }


            // tf2::Quaternion quaternion(tag_detection_pose.pose.pose.orientation.x, 
            //                             tag_detection_pose.pose.pose.orientation.y, 
            //                             tag_detection_pose.pose.pose.orientation.z, 
            //                             tag_detection_pose.pose.pose.orientation.w); 
            // tf2::Matrix3x3 mat(quaternion);
            // double roll, pitch, yaw;
            // mat.getRPY(roll, pitch, yaw);
            // double z_output = z_pid.calculate(0, yaw);

            geometry_msgs::Twist twist;
            twist.linear.x = x_output;
            twist.linear.y = y_output;
            // twist.linear.z = 0;
            // twist.angular.x = 0;
            // twist.angular.y = 0;
            // twist.angular.z = z_output * 0.8;
            cmd_pub.publish(twist);
            //ros::Duration(0.05).sleep();
            std::cout << "x : " << tag_detection_pose.pose.pose.position.x << std::endl;
            // std::cout << "pid output in x : " << x_output << std::endl;
            // std::cout << "pid output in y : " << y_output << std::endl;
            // std::cout << "pid output in z : " << z_output << std::endl;


        }
        if(!block_detected&&target_tag_map_pose.header.frame_id!="map")
            {
                geometry_msgs::Twist twist;
                no_need_block_time++;
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.linear.z = 0;
                twist.angular.x = 0;
                twist.angular.y = 0;
                twist.angular.z = 0;
                cmd_pub.publish(twist);
                
            }
        if(!block_detected && target_tag_map_pose.header.frame_id=="map")
        {
            ROS_INFO("The target block was blocked by other tag!");
            tf::Transform transform;
            if(map_tag_clear_pid==false)
            {
                x_pid.clear_pid();
                y_pid.clear_pid();
                map_tag_clear_pid=true;
            }
            transform.setOrigin(tf::Vector3(target_tag_map_pose.pose.position.x, target_tag_map_pose.pose.position.y, target_tag_map_pose.pose.position.z));
            tf::Quaternion q(target_tag_map_pose.pose.orientation.x, target_tag_map_pose.pose.orientation.y, target_tag_map_pose.pose.orientation.z, target_tag_map_pose.pose.orientation.w);
            transform.setRotation(q);
            tf_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "target_tag"));

            tf::Stamped<tf::Pose> target_tag_pose_tf;
            target_tag_pose_tf.setIdentity();
            target_tag_pose_tf.frame_id_ = "target_tag";
            target_tag_pose_tf.stamp_ = ros::Time();
            tf::Stamped<tf::Pose> target_tag_camera_pose_tf;

            try{
                tf_listener_->waitForTransform( "camera_color_optical_frame", "target_tag", ros::Time(0), ros::Duration(0.5));
                tf_listener_->transformPose( "camera_color_optical_frame", target_tag_pose_tf, target_tag_camera_pose_tf);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("Failed to transform robot pose: %s", ex.what());
            }
            geometry_msgs::PoseStamped target_tag_camera_pose;
            tf::poseStampedTFToMsg(target_tag_camera_pose_tf, target_tag_camera_pose);

            std::cout<<"camera_x: "<<target_tag_camera_pose.pose.position.x<<" camera_y"<<target_tag_camera_pose.pose.position.y<<std::endl;
            double y_output;
            double x_output;
            if(abs(target_tag_camera_pose.pose.position.x-y_target)>0.01&&y_done==false)
            {
                y_output = y_pid.calculate(y_target, target_tag_camera_pose.pose.position.x);
            }
            else{
                y_output=0;
                y_done=true;
            }
            if(abs(target_tag_camera_pose.pose.position.z-x_target)>0.01&&y_done)
            {

                x_output = -x_pid.calculate(x_target, target_tag_camera_pose.pose.position.z);
                // x_output=0;
            }

            else if(abs(target_tag_camera_pose.pose.position.z-x_target)<=0.01&&y_done) 
            {
                geometry_msgs::Twist twist;
                x_output=0;
                twist.linear.x = 0;
                twist.linear.y = 0;
                cmd_pub.publish(twist);
                open_gripper();
                ros::Duration(0.7).sleep();
                // setOutput<int>("takeing_cube_num",tag_id);
                geometry_msgs::Pose arm_position;
                arm_position.position.x = 0.19;
                arm_position.position.y = -0.08;
                arm_position_pub.publish(arm_position);
                ros::Duration(2).sleep();

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
        }

        return NodeStatus::RUNNING;
    }
    void onHalted() override
    {
        tag_detection_status_sub.shutdown();
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
    ros::Publisher target_cube_num_pub;
    ros::Subscriber position_state_sub;
    // ros::Subscriber tar_tag_sub;
    geometry_msgs::PoseStamped target_tag_map_pose;
    int tag_id;
    int take_node_state;
    bool map_tag_clear_pid;
    const float x_target = 0.15;
    const float y_target = 0.045;
    int no_need_block_time;
    KalmanFilter2D tag_kalman_filter;
    PIDController x_pid, y_pid, z_pid;
    vector <int> target_cube_num;
    vector<double> x_param = {1.1, 0.01, 0.01};
    vector<double> y_param = {1, 0.01, 0.01};
    vector<double> z_param = {1, 0, 0};

};