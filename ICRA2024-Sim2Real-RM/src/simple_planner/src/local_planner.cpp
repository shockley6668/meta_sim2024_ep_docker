/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Eigen>
#include <chrono>

#include "cubic_spline/cubic_spline_ros.h"
#include "utility.h"


namespace robomaster{

 class PIDController {
    public:
        PIDController(){}
        PIDController(double kp, double ki, double kd)
            : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}

        double calculate(double setpoint, double now_measure) {
            double error = setpoint - now_measure;
            double p_out = kp_ * error;
            integral_ += error;
            double i_out = ki_ * integral_;
            double d_out = kd_ * (error - prev_error_);
            prev_error_ = error;
            double output=p_out + i_out + d_out;
            if(output<0.1&&output>0)
            {
                output = 0.1;
            }
            else if(output>-0.1&&output<0)
            {
                output = -0.1;
            }
            return output;
    }

    private:
        double kp_, ki_, kd_;
        double prev_error_;
        double integral_;
};
    class LocalPlanner{
    public:
        PIDController controller_x;
        PIDController controller_y;
        PIDController controller_w;
        LocalPlanner(ros::NodeHandle& given_nh):nh(given_nh),plan_(false), prune_index_(0),target_yaw(0),controller_x(6,0.01,0.0),controller_y(6,0.01,0.0),controller_w(3,0.03,0.0){

           
            nh.param<double>("max_speed", max_speed_, 2.0);
            double max_angle_diff;
            nh.param<double>("max_angle_diff", max_angle_diff, 60);
            max_angle_diff_ = max_angle_diff * M_PI / 180;
            nh.param<double>("p_coeff", p_coeff_, 10.0);
            nh.param<int>("plan_frequency", plan_freq_, 50);
            nh.param<double>("goal_tolerance", goal_tolerance_, 0.05);
            nh.param<double>("prune_ahead_distance", prune_ahead_dist_, 0.1);
            nh.param<std::string>("global_frame", global_frame_, "odom");
            diff_yaw=0;
    
            tf_listener_ = std::make_shared<tf::TransformListener>();

            local_path_pub_= nh.advertise<nav_msgs::Path>("path", 5);

            cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

            global_path_sub_ = nh.subscribe("/global_planner/path", 5, &LocalPlanner::GlobalPathCallback,this);
            target_yaw_sub_ = nh.subscribe("/target_yaw", 5, &LocalPlanner::TargetYawCallback,this);

            plan_timer_ = nh.createTimer(ros::Duration(1.0/plan_freq_),&LocalPlanner::Plan,this);
            


        }
        ~LocalPlanner()= default;
        void GlobalPathCallback(const nav_msgs::PathConstPtr & msg){
            if (!msg->poses.empty()){
                global_path_ = *msg;
                prune_index_ = 0;
                plan_ = true;
                xy_done = false;
                yaw_done = false;
            }
        }
    private:
        void TargetYawCallback(const std_msgs::Float64ConstPtr & msg){
            target_yaw = msg->data;
        }
        void velocitylimit(double &in,double limit){
            if(in>0)
            {
                in=std::min(in,limit);
            }
            else
            {
                in=std::max(in,-limit);
            }
        }
    
        void Plan(const ros::TimerEvent& event){

            if (plan_){

                auto begin = std::chrono::steady_clock::now();
                auto start = ros::Time::now();
                // 1. Update the transform from global path frame to local planner frame
                UpdateTransform(tf_listener_, global_frame_,
                                global_path_.header.frame_id, global_path_.header.stamp,
                                global2path_transform_);//source_time needs decided
                  //odom to base link 转换                
                UpdateTransform(tf_listener_, "base_link",
                                "odom", ros::Time(0),
                                odom2baselink_transform_);
                std::cout<<ros::Time::now()- start<<std::endl;

                // 2. Get current robot pose in global path frame
                geometry_msgs::PoseStamped robot_pose;
                GetGlobalRobotPose(tf_listener_, global_path_.header.frame_id, robot_pose);

                // 3. Check if robot has already arrived with given distance tolerance
                
                if (GetEuclideanDistance(robot_pose,global_path_.poses.back())<= goal_tolerance_
                    && prune_index_ == global_path_.poses.size() - 1){
                    xy_done=true;
                }
                if(xy_done&&yaw_done)
                {
                    plan_=false;
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0;
                    cmd_vel.linear.y = 0;
                    cmd_vel.angular.z = 0;
                    cmd_vel.linear.z = 1;   // bool success or not
                    cmd_vel_pub_.publish(cmd_vel);
                    ROS_INFO("Planning Success!");
                    return;
                }
                // 4. Get prune index from given global path
                FindNearstPose(robot_pose, global_path_, prune_index_, prune_ahead_dist_);// TODO: double direct prune index is needed later!

                // 5. Generate the prune path and transform it into local planner frame
                nav_msgs::Path prune_path, local_path;

                local_path.header.frame_id = global_frame_;
                prune_path.header.frame_id = global_frame_;

                geometry_msgs::PoseStamped tmp_pose;
                tmp_pose.header.frame_id = global_frame_;

                TransformPose(global2path_transform_, robot_pose, tmp_pose);
                prune_path.poses.push_back(tmp_pose);

                for (int i = prune_index_; i < global_path_.poses.size(); i++){
                    TransformPose(global2path_transform_, global_path_.poses[i], tmp_pose);
                    prune_path.poses.push_back(tmp_pose);

                }

                // 6. Generate the cubic spline trajectory from above prune path
                GenTraj(prune_path, local_path);
                local_path_pub_.publish(local_path);

                // 7. Follow the trajectory and calculate the velocity
                geometry_msgs::Twist cmd_vel;
                FollowTraj(prune_path.poses.front(), local_path, cmd_vel);
                cmd_vel_pub_.publish(cmd_vel);

                auto plan_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin);
                ROS_INFO("Planning takes %f ms and passed %d/%d.",
                         plan_time.count()/1000.,
                         prune_index_,
                         static_cast<int>(global_path_.poses.size()));
            }

        }

        void FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                        const nav_msgs::Path& traj,
                        geometry_msgs::Twist& cmd_vel){

            diff_yaw = target_yaw - GetYawFromOrientation(robot_pose.pose.orientation);

            bool test = std::isnan(diff_yaw);
            if(std::isnan(diff_yaw)){
                plan_ = false;
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = 0;
                cmd_vel_pub_.publish(cmd_vel);
                ROS_WARN("Planning is not feasible with diff_yaw : %f",diff_yaw);
                return;
            }


            // set it from -PI to PI
            if(diff_yaw > M_PI){
                diff_yaw -= 2*M_PI;
            } else if(diff_yaw < -M_PI){
                diff_yaw += 2*M_PI;
            }

            // // calculate velocity

            // if( diff_yaw > 0 ){
            //     cmd_vel.angular.z = std::min(p_coeff_*diff_yaw, 2.0);
            // }else{
            //     cmd_vel.angular.z = std::max(p_coeff_*diff_yaw, -2.0);
            // }


            // cmd_vel.linear.x = max_speed_*(1.0-std::abs(diff_yaw)/(max_angle_diff_));
            // cmd_vel.linear.y = 0;
            // if(std::abs(diff_yaw) > max_angle_diff_){
            //     cmd_vel.linear.x = 0;
            // }
            //转换pose到baselink
            geometry_msgs::PoseStamped base_link_traj_pose;
            int trajSize=traj.poses.size();
            //计算前瞻点
            int i;
            for(i=0;i<trajSize;i++)
            {
                if(GetEuclideanDistance(robot_pose,traj.poses[i])>0.2)
                {
                    break;
                }
            }
        
            if(i<trajSize)
            {
                TransformPose(odom2baselink_transform_,traj.poses[i],base_link_traj_pose);
            }
            else
            {
                TransformPose(odom2baselink_transform_,traj.poses[trajSize-1],base_link_traj_pose);
            }
           
            
            // if(traj.poses.size()>0)
            // {
                
            //     std::cout<<base_link_traj_pose.pose.position.x<<std::endl;
            //     std::cout<<base_link_traj_pose.pose.position.y<<std::endl;
            // }
            //baselink下目标点的坐标
            double diff_x = base_link_traj_pose.pose.position.x;
            double diff_y = base_link_traj_pose.pose.position.y;
            //0就是使得机器人坐标与目标坐标重合
            cmd_vel.linear.x=-controller_x.calculate(0,diff_x);
            cmd_vel.linear.y=-controller_y.calculate(0,diff_y);
            cmd_vel.angular.z=-controller_w.calculate(0,diff_yaw);
            if(abs(diff_yaw)<0.06)
            {
                cmd_vel.angular.z=0;
                yaw_done=true;    
            }
            if(xy_done)
            {
                cmd_vel.linear.x=0;
                cmd_vel.linear.y=0;
            }
            //std::cout<<"diff_yaw:"<<diff_yaw<<std::endl;
            //速度限幅
            velocitylimit(cmd_vel.linear.x,3);
            velocitylimit(cmd_vel.linear.y,3);
            velocitylimit(cmd_vel.angular.z,1.5);
        }

    private:

        void FindNearstPose(geometry_msgs::PoseStamped& robot_pose,nav_msgs::Path& path, int& prune_index, double prune_ahead_dist = 0.3){

            double dist_threshold = 10;// threshold is 10 meters (basically never over 10m i suppose)
            double sq_dist_threshold = dist_threshold * dist_threshold;
            double sq_dist;
            if(prune_index!=0){
                sq_dist = GetEuclideanDistance(robot_pose,path.poses[prune_index-1]);
            }else{
                sq_dist = 1e10;
            }

            double new_sq_dist = 0;
            while (prune_index < (int)path.poses.size()) {
                new_sq_dist = GetEuclideanDistance(robot_pose,path.poses[prune_index]);
                if (new_sq_dist > sq_dist && sq_dist < sq_dist_threshold) {

                    //Judge if it is in the same direction and sq_dist is further than 0.3 meters
                    if ((path.poses[prune_index].pose.position.x - robot_pose.pose.position.x) *
                        (path.poses[prune_index-1].pose.position.x - robot_pose.pose.position.x) +
                        (path.poses[prune_index].pose.position.y - robot_pose.pose.position.y) *
                        (path.poses[prune_index-1].pose.position.y - robot_pose.pose.position.y) > 0
                        && sq_dist > prune_ahead_dist) {
                        prune_index--;
                    }else{
                        sq_dist = new_sq_dist;
                    }

                    break;
                }
                sq_dist = new_sq_dist;
                ++prune_index;
            }

            prune_index = std::min(prune_index, (int)(path.poses.size()-1));

        }

        ros::NodeHandle nh;
        std::shared_ptr<tf::TransformListener> tf_listener_;
        tf::StampedTransform global2path_transform_;
        tf::StampedTransform odom2baselink_transform_;
        std::string global_frame_;
        ros::Timer plan_timer_;
        
        ros::Subscriber global_path_sub_;
        ros::Subscriber target_yaw_sub_;
        ros::Publisher local_path_pub_;
        ros::Publisher cmd_vel_pub_;


        bool plan_;
        bool xy_done;
        bool yaw_done;
        int prune_index_;
        nav_msgs::Path global_path_;

        double max_speed_;
        double max_angle_diff_;
        double p_coeff_;
        double goal_tolerance_;
        double prune_ahead_dist_;
        double target_yaw;
        double diff_yaw;
        int plan_freq_;

    };
}

using namespace robomaster;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle nh("~");
    LocalPlanner local_planner(nh);
    ros::spin();
    return 0;
}


