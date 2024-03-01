#include "std_msgs/Int32.h"
#include "ros/ros.h"
#include "bt_frame/ep_goal.h"
#include <queue>
#include <vector>
#include <cstdlib>
#include <climits>
#include "simple_planner/change_point_num.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "utility.h"
#include <stdexcept>
using namespace std;

bool goal_reached=false;
bool first_find=true;
int state;
bool send_flag=false;
bool detected = false;
vector <int> place1;
vector <int> place2;
vector <int> place3;
vector <int> finish;
bool no_move=false;
bool find_three_state=true;
bool switch_mode = true;
vector <int> target_cube_num;
apriltag_ros::AprilTagDetectionArray tag_msg;
geometry_msgs::PoseStamped robot_gobal_pose_;
std::shared_ptr<tf::TransformListener> tf_listener_;
tf::StampedTransform tag2map_transform_;
vector<geometry_msgs::PoseStamped> poses = vector<geometry_msgs::PoseStamped>(6);
double max_value = std::numeric_limits<double>::max();
vector<double> weight = {max_value, max_value, max_value, max_value, max_value, max_value};
vector<vector<int>> grid = {
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 1, 1, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 1, 1, 0, 1, 1, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    };
struct Node {
    int x, y, dist;
    vector<pair<int, int>> path;
    Node(int x, int y, int dist = 0, vector<pair<int, int>> path = {}): x(x), y(y), dist(dist), path(path) {}
    bool operator<(const Node& that) const {
        return dist > that.dist;
    }
};
const int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
const int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};



vector<pair<int, int>> dijkstra(vector<vector<int>>& grid, Node start, Node end) {
    int m = grid.size(), n = grid[0].size();
    vector<vector<int>> dist(m, vector<int>(n, INT_MAX));
    priority_queue<Node> pq;

    start.path.push_back({start.x, start.y});

    dist[start.x][start.y] = 0;
    pq.push(start);

    while (!pq.empty()) {
        Node node = pq.top(); pq.pop();
        if (node.x == end.x && node.y == end.y) return node.path;
        for (int i = 0; i < 8; ++i) {
            int x = node.x + dx[i], y = node.y + dy[i];
            if (x < 0 || x >= m || y < 0 || y >= n || grid[x][y] == 1) continue;
            int d = node.dist + (i < 4 ? 1 : sqrt(2)); // 斜着走的距离为sqrt(2)
            if (d < dist[x][y]) {
                dist[x][y] = d;
                vector<pair<int, int>> newPath = node.path;
                newPath.push_back({x, y});
                pq.push(Node(x, y, d, newPath));
            }
        }
    }

    return {};
}

void goal_status_callback(const std_msgs::Int32::ConstPtr &msg)
{
    if(msg->data==1)
    {
        goal_reached=true;
    }
} 
void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr & msg)
{
    tag_msg=*msg;
    if(msg->detections.size() > 0)
    {
        detected = true;
    }
    else
    {
        detected = false;
    }
    geometry_msgs::PoseStamped tag_map_pose;
    for(auto it : msg->detections)
    {
        if(it.id[0] >= 6)
            continue;
        bool fns = false;
        // for(int i = 0;i < place1.size();i++)
        //     if(place1[i] == it.id[0])
        //         fns = true;
        // for(int i = 0;i < place2.size();i++)
        //     if(place2[i] == it.id[0])
        //         fns = true;
        // for(int i = 0;i < place3.size();i++)
        //     if(place3[i] == it.id[0])
        //         fns = true;
        // if(fns)
        //     continue;
        geometry_msgs::PoseStamped it_pose;
        it_pose.header = it.pose.header;
        it_pose.pose.position.x = it.pose.pose.pose.position.x;
        it_pose.pose.position.y = it.pose.pose.pose.position.y;
        it_pose.pose.position.z = it.pose.pose.pose.position.z;
        it_pose.pose.orientation.x = it.pose.pose.pose.orientation.x;
        it_pose.pose.orientation.y = it.pose.pose.pose.orientation.y;
        it_pose.pose.orientation.z = it.pose.pose.pose.orientation.z;
        UpdateTransform(tf_listener_, "map", "camera_color_optical_frame", ros::Time::now(), tag2map_transform_);
        TransformPose(tag2map_transform_, it_pose, tag_map_pose);
        if(tag_map_pose.pose.position.z >= 0)
            continue;
        double dist_ = 0;
        GetGlobalRobotPose(tf_listener_, "map", robot_gobal_pose_);
        dist_ = sqrt(pow(tag_map_pose.pose.position.x - robot_gobal_pose_.pose.position.x, 2) + pow(tag_map_pose.pose.position.y - robot_gobal_pose_.pose.position.y, 2));
        double w = dist_ + tag_map_pose.pose.position.y;
        // if(poses[it.id[0]].pose.position.x == 0)
        {
            // if(dist <= 2.0f)
            //     poses[it.id[0]] = robot_gobal_pose_;
            if(w <= weight[it.id[0]] && dist_ >= 0.45 && dist_ <= 1.8)
            {
                weight[it.id[0]] = w;
                poses[it.id[0]] = robot_gobal_pose_;
            }
        }
        // std::cout << "tag_current_weight" << w << std::endl;
        // std::cout << "tag_weight: " << weight[it.id[0]] << std::endl;
        double dist[4] = {0};
        dist[0] = pow(tag_map_pose.pose.position.x - 0.8, 2) + pow(tag_map_pose.pose.position.y - 1.0, 2);
        dist[1] = pow(tag_map_pose.pose.position.x - 1.86, 2) + pow(tag_map_pose.pose.position.y + 0.1, 2);
        dist[2] = pow(tag_map_pose.pose.position.x - 1.3, 2) + pow(tag_map_pose.pose.position.y - 2.9, 2);
        dist[3] = pow(tag_map_pose.pose.position.x - 1.4, 2) + pow(tag_map_pose.pose.position.y - 1.7, 2);
        // std::cout << it.id[0] << " dist: " << dist[0] << " " << dist[1] << " " << dist[2] << " " << dist[3] << std::endl;

        double min = dist[0];
        int index = 0;
        for(int i = 0; i < 4; i++)
        {
            if(dist[i] < min)
            {
                min = dist[i];
                index = i;
            }
        }
        if(tag_map_pose.pose.position.y >= 2.75)
            index = 2;
        else if(tag_map_pose.pose.position.x <= 0.62)
            index = 0;
        else if(tag_map_pose.pose.position.x >= 1.70)
            index = 1;
        // std::cout << "index: " << index << std::endl;
        switch (index)
        {
        case 0:
        {
            if(std::find(place1.begin(), place1.end(), it.id[0]) == place1.end())
            {
                if(std::find(finish.begin(), finish.end(), it.id[0]) == finish.end())
                    place1.push_back(it.id[0]);
            }
        }
            break;
        case 1:
        {
            if(std::find(place2.begin(), place2.end(), it.id[0]) == place2.end())
            {
                if(std::find(finish.begin(), finish.end(), it.id[0]) == finish.end())
                    place2.push_back(it.id[0]);
            }
        }
            break;
        case 2:
        {
            if(std::find(place3.begin(), place3.end(), it.id[0]) == place3.end())
            {
                if(std::find(finish.begin(), finish.end(), it.id[0]) == finish.end())
                    place3.push_back(it.id[0]);
            }
        }
            break;
        default:
            break;
        }
    }
    // for(int i = 0;i < 6;i++)
    // {
    //     if(poses[i].pose.position.x != 0)
    //         std::cout << "id = " << i << ": " << poses[i].pose.position.x << " " << poses[i].pose.position.y << std::endl;
    // }
}
void no_move_callback(const std_msgs::Int32::ConstPtr &msg)
{
    // if(msg->data==1)
    // {
        // if(tag_msg.detections.size()>0)
        // {
        //     switch (state)
        //     {
        //     case 1:
        //         // place1.clear();
        //         for(int i=0;i<tag_msg.detections.size();i++)
        //         {
        //             if (std::find(place1.begin(), place1.end(), tag_msg.detections[i].id[0]) == place1.end()) {
        //                 place1.push_back(tag_msg.detections[i].id[0]);  
        //             }
        //         }

        //         break;
        //     case 2:
        //         // place2.clear();
        //         for(int i=0;i<tag_msg.detections.size();i++)
        //         {
        //             if (std::find(place2.begin(), place2.end(), tag_msg.detections[i].id[0]) == place2.end()) {
        //                 place2.push_back(tag_msg.detections[i].id[0]);  
        //             }
        //         }
        //         break;
        //     case 3:
        //         // place3.clear();
        //         for(int i=0;i<tag_msg.detections.size();i++)
        //         {
        //             if(place3.empty())
        //             {
        //                 place3.push_back(tag_msg.detections[i].id[0]);
        //             }
        //             if (std::find(place3.begin(), place3.end(), tag_msg.detections[i].id[0]) == place3.end()) {
        //                 place3.push_back(tag_msg.detections[i].id[0]);  
        //             }
        //         }
        //         break;
        //     default:
        //         break;
        //     }
        // }
    // }
}
void target_num_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    if(msg->data.size()==3)
    {
        target_cube_num=msg->data;
    }
}
template <typename T>
void printVector(const vector<T> v)
{
    if(v.empty())
    {
        std::cout<<"empty"<<std::endl;
    } 
    for (int i = 0; i < v.size(); i++)
    {
        cout << v[i] << " ";
    }
    cout << endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_decition_node");
    ros::NodeHandle nh;

    ros::Publisher goal_pub = nh.advertise<bt_frame::ep_goal>("ep_next_goal", 10);
    ros::Publisher test = nh.advertise<std_msgs::Int32>("test", 10);
    ros::Subscriber goal_status_sub=nh.subscribe<std_msgs::Int32>("ep_goal_status",10,goal_status_callback);
    ros::Rate loop_rate(100);
    ros::ServiceClient change_point_client = nh.serviceClient<simple_planner::change_point_num>("/change_point_num");
    ros::Subscriber tag_sub = nh.subscribe("/tag_detections", 10, &tagCallback);
    ros::Subscriber no_move_sub=nh.subscribe("no_move",10,no_move_callback);
    ros::Subscriber target_num_sub=nh.subscribe<std_msgs::Int32MultiArray>("target_cube_num",10,target_num_callback);
    ros::Publisher position_set_pub = nh.advertise<std_msgs::Int32>("position_state", 10);
    ros::Publisher tar_tag = nh.advertise<std_msgs::Int32>("tar_tag", 10);
    ros::Subscriber taking_tag_sub = nh.subscribe<std_msgs::Int32>("taking_tag_id", 10, [&](const std_msgs::Int32::ConstPtr &msg){
        // std::cout << "now taking tag: " << msg->data << std::endl;
        for(int i = 0;i < place1.size();i++)
            if(place1[i] == msg->data)
            {
                // place1.erase(place1.begin() + i);
                place1[i] = -1;
                if(std::find(finish.begin(), finish.end(), msg->data) == finish.end())
                    finish.push_back(msg->data);
                return;
            }   
        for(int i = 0;i < place2.size();i++)
            if(place2[i] == msg->data)
            {
                // place2.erase(place2.begin() + i);
                place2[i] = -1;
                if(std::find(finish.begin(), finish.end(), msg->data) == finish.end())
                    finish.push_back(msg->data);
                return;
            }
        for(int i = 0;i < place3.size();i++)
            if(place3[i] == msg->data)
            {
                // place3.erase(place3.begin() + i);
                place3[i] = -1;
                if(std::find(finish.begin(), finish.end(), msg->data) == finish.end())
                    finish.push_back(msg->data);
                return;
            }
    });
    ros::Subscriber position_state_sub = nh.subscribe<std_msgs::Int32>("check_done", 10, [&](const std_msgs::Int32::ConstPtr &msg){
        if(msg->data == 1)
        {
            find_three_state = false;
        }
        else{
            find_three_state = true;
        }
    });

    tf_listener_ = std::make_shared<tf::TransformListener>();
    state=0;
    loop_rate.sleep();
    while (ros::ok())
    {
        GetGlobalRobotPose(tf_listener_, "map", robot_gobal_pose_);
        if(goal_reached && !detected)
        {
            double base_dist[4];
            int base_index = -1;
            base_dist[0] = pow(robot_gobal_pose_.pose.position.x - 0.8, 2) + pow(robot_gobal_pose_.pose.position.y - 1.0, 2);
            base_dist[1] = pow(robot_gobal_pose_.pose.position.x - 1.86, 2) + pow(robot_gobal_pose_.pose.position.y + 0.1, 2);
            base_dist[2] = pow(robot_gobal_pose_.pose.position.x - 1.3, 2) + pow(robot_gobal_pose_.pose.position.y - 2.9, 2);
            base_dist[3] = pow(robot_gobal_pose_.pose.position.x - 1.4, 2) + pow(robot_gobal_pose_.pose.position.y - 1.7, 2);

            double min = base_dist[0];
            for(int k = 0;k < 4;k++)
            {
                if(base_dist[k] < min)
                {
                    min = base_dist[k];
                    base_index = k;
                }
            }

            switch (base_index)
            {
                case 0:
                    place1.clear();
                    break;
                case 1:
                    place2.clear();
                    break;
                case 2:
                    place3.clear();
                    break;
                default:
                    break;
            }
        }
        if(find_three_state && first_find)
        {
            //三点巡航
            std::cout << "find_three_state and first_find" << std::endl;
            switch_mode = true;
        }
        else if(!first_find && find_three_state)
        {
            //according to placex and poses to search target block
            std::cout << "not first_find and find_three_state" << std::endl;
            int i = 0;
            for(i = 0;i < 3;i++)
            {
                if(std::find(finish.begin(), finish.end(), target_cube_num[i]) != finish.end())
                    continue;
                else
                    break;
            }
            bool found[3] = {false, false, false};
            found[0] = std::find(place1.begin(), place1.end(), target_cube_num[i]) != place1.end();
            found[1] = std::find(place2.begin(), place2.end(), target_cube_num[i]) != place2.end();
            found[2] = std::find(place3.begin(), place3.end(), target_cube_num[i]) != place3.end();
            bool fns_found = false;
            if(found[0] && !fns_found)
            {
                std::cout << "target block id : " << target_cube_num[i] << " found in place1" << std::endl;
                fns_found = true;
                if(poses[target_cube_num[i]].pose.position.x == 0 && poses[target_cube_num[i]].pose.position.y == 0)
                {
                    state = 0;
                    switch_mode = true;
                }
                else
                {
                    if(goal_reached && !detected)
                    {
                        if(found[1])
                            state = 1;
                        else if(found[2])
                            state = 2;
                        else
                        {
                            if(state == 0)
                                state = 1;
                            else if(state == 1)
                                state = 2;
                            else if(state == 2)
                                state = 0;
                            else if(state == 3)
                                state = 0;
                            else if(state == -1)
                                state = 0;
                        }
                        std::cout << "state : " << state << std::endl;
                        goal_reached = false;
                        switch_mode = true;
                        found[0] = false;
                        place1.clear();
                    }
                    if(switch_mode == false)
                    {
                        if(goal_reached)
                        {
                            static int cnt = 0;
                            cnt++;
                            if(cnt > 15)
                            {
                                int k = 0;
                                for(k = 0;k < place1.size();k++)
                                    if(place1[k] == target_cube_num[i])
                                    {
                                        place1[k] = -1;
                                        break;
                                    }
                                cnt = 0;
                            }
                        }
                        bt_frame::ep_goal goal;
                        goal.type = 0;
                        goal.x = poses[target_cube_num[i]].pose.position.x;
                        goal.y = poses[target_cube_num[i]].pose.position.y;
                        tf::Quaternion q;
                        tf::quaternionMsgToTF(poses[target_cube_num[i]].pose.orientation, q);
                        double roll, pitch, yaw;
                        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                        goal.yaw = yaw;
                        goal_pub.publish(goal);
                        state = -1;
                    }
                }
            }
            if(found[1] && !fns_found)
            {
                // found = true;
                std::cout << "target block id : " << target_cube_num[i] << " found in place2" << std::endl;
                fns_found = true;
                if(poses[target_cube_num[i]].pose.position.x == 0 && poses[target_cube_num[i]].pose.position.y == 0)
                {
                    state = 1;
                    switch_mode = true;
                }
                else
                {
                    if(goal_reached && !detected)
                    {
                        if(found[0])
                            state = 0;
                        else if(found[2])
                            state = 2;
                        else
                        {
                            if(state == 0)
                                state = 1;
                            else if(state == 1)
                                state = 2;
                            else if(state == 2)
                                state = 0;
                            else if(state == 3)
                                state = 0;
                            else if(state == -1)
                                state = 0;
                        }
                        std::cout << "state : " << state << std::endl;
                        goal_reached = false;
                        found[1] = false;
                        switch_mode = true;
                        // place2.resize(0); 
                    }
                    if(switch_mode==false)
                    {
                        if(goal_reached)
                        {
                            static int cnt = 0;
                            cnt++;
                            if(cnt > 15)
                            {
                                int k = 0;
                                for(k = 0;k < place2.size();k++)
                                    if(place2[k] == target_cube_num[i])
                                    {
                                        place2[k] = -1;
                                        break;
                                    }
                                cnt = 0;
                            }
                        }
                        bt_frame::ep_goal goal;
                        goal.type = 0;
                        goal.x = poses[target_cube_num[i]].pose.position.x;
                        goal.y = poses[target_cube_num[i]].pose.position.y;
                        tf::Quaternion q;
                        tf::quaternionMsgToTF(poses[target_cube_num[i]].pose.orientation, q);
                        double roll, pitch, yaw;
                        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                        goal.yaw = yaw;
                        goal_pub.publish(goal);
                        state = -1;
                    }
                }
            }
            if(found[2] && !fns_found)
            {
                std::cout << "target block id : " << target_cube_num[i] << " found in place3" << std::endl;
                fns_found = true;
                if(poses[target_cube_num[i]].pose.position.x == 0 && poses[target_cube_num[i]].pose.position.y == 0)
                {
                    state = 2;
                    switch_mode = true;
                }
                else
                {
                    if(goal_reached && !detected)
                    {
                        if(found[0])
                            state = 0;
                        else if(found[1])
                            state = 1;
                        else
                        {
                            if(state == 0)
                                state = 1;
                            else if(state == 1)
                                state = 2;
                            else if(state == 2)
                                state = 0;
                            else if(state == 3)
                                state = 0;
                            else if(state == -1)
                                state = 0;
                        }
                        std::cout << "state : " << state << std::endl;
                        goal_reached = false;
                        switch_mode = true;
                        // place3.resize(0);
                    }
                    if(switch_mode==false)
                    {
                        if(goal_reached)
                        {
                            static int cnt = 0;
                            cnt++;
                            if(cnt > 15)
                            {
                                int k = 0;
                                for(k = 0;k < place3.size();k++)
                                    if(place3[k] == target_cube_num[i])
                                    {
                                        place3[k] = -1;
                                        break;
                                    }
                                cnt = 0;
                            }
                        }
                        bt_frame::ep_goal goal;
                        goal.type = 0;
                        goal.x = poses[target_cube_num[i]].pose.position.x;
                        goal.y = poses[target_cube_num[i]].pose.position.y;
                        tf::Quaternion q;
                        tf::quaternionMsgToTF(poses[target_cube_num[i]].pose.orientation, q);
                        double roll, pitch, yaw;
                        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                        goal.yaw = yaw;
                        goal_pub.publish(goal);
                        state = -1;
                    }
                }
            }
            if(!found[0] && !found[1] && !found[2])
            {
                if(state == 0)
                    state = 1;
                else if(state == 1)
                    state = 2;
                else if(state == 2)
                    state = 0;
                else if(state == 3)
                    state = 0;
                else if(state == -1)
                    state = 0;
                // place1.resize(0);
                // place2.resize(0);
                // place3.resize(0);
                switch_mode = true;
                std::cout << "state : " << state << std::endl;
            }
        }
        else if(!first_find && !find_three_state)
        {
            //according to placex and poses to arrange path
            std::cout << "not first_find and not find_three_state" << std::endl;
            // bool found = false;
            bool found[3] = {false, false, false};
            int j = 0;
            for(j = 0;j < place1.size();j++)
                if(place1[j] != -1)
                {
                    found[0] = true;
                    break;
                }
            if(j == place1.size())
                found[0] = false;
            for(j = 0;j < place2.size();j++)
                if(place2[j] != -1)
                {
                    found[1] = true;
                    break;
                }
            if(j == place2.size())
                found[1] = false;
            for(j = 0;j < place3.size();j++)
                if(place3[j] != -1)
                {
                    found[2] = true;
                    break;
                }
            if(j == place3.size())  
                found[2] = false;
            std::cout << "status in places : " << found[0] << " " << found[1] << " " << found[2] << std::endl;
            if(found[0])
            {
                if(goal_reached && !detected)
                {
                    if(found[1])
                        state = 1;
                    else if(found[2])
                        state = 2;
                    else
                    {
                        if(state == 0)
                            state = 1;
                        else if(state == 1)
                            state = 2;
                        else if(state == 2)
                            state = 0;
                        else if(state == 3)
                            state = 0;
                        else if(state == -1)
                            state = 0;
                    }
                    // place1.clear();
                    std::cout << "state : " << state << std::endl;
                    std::cout << "loop to place" << state + 1 << std::endl;
                    goal_reached = false;
                    switch_mode = true;
                    found[0] = false;
                }
                int tag_id = -1;
                for(int k = 0;k < place1.size();k++)
                    if(place1[k] != -1)
                    {
                        tag_id = k;
                        break;
                    }
                try
                {
                    if(poses[tag_id].pose.position.x == 0 && poses[tag_id].pose.position.y == 0)
                    {
                        state = 0;
                        std::cout << "loop to place" << state + 1 << std::endl;
                        switch_mode = true;
                    }
                    else
                    {   
                        if(switch_mode==false)
                        {
                            bt_frame::ep_goal goal;
                            goal.type = 0;
                            goal.x = poses[tag_id].pose.position.x;
                            goal.y = poses[tag_id].pose.position.y;
                            tf::Quaternion q;
                            tf::quaternionMsgToTF(poses[tag_id].pose.orientation, q);
                            double roll, pitch, yaw;
                            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                            goal.yaw = yaw;
                            goal_pub.publish(goal);
                            state = -1;
                            std::cout << "target position : " << goal.x << " " << goal.y << std::endl;
                        }
                    }  
                }     
                catch(const std::exception& e)
                {
                    ROS_ERROR("out of range: %s", e.what());
                    state = 0;
                    switch_mode = true;
                }
            }
            if(found[1])
            {
                if(goal_reached && !detected)
                {
                    if(found[0])
                        state = 0;
                    else if(found[2])
                        state = 2;
                    else
                    {
                        if(state == 0)
                            state = 1;
                        else if(state == 1)
                            state = 2;
                        else if(state == 2)
                            state = 0;
                        else if(state == 3)
                            state = 0;
                        else if(state == -1)
                            state = 0;
                    }
                    // place2.clear();
                    std::cout << "state : " << state << std::endl;
                    std::cout << "loop to place" << state + 1 << std::endl;
                    goal_reached = false;
                    found[1] = false;
                    switch_mode = true;
                }
                int tag_id = -1;
                for(int k = 0;k < place2.size();k++)
                    if(place2[k] != -1)
                    {
                        tag_id = k;
                        break;
                    }
                try
                {
                    if(poses[tag_id].pose.position.x == 0 && poses[tag_id].pose.position.y == 0)
                    {
                        state = 1;
                        std::cout << "loop to place" << state + 1 << std::endl;
                        switch_mode = true;
                    }
                    else
                    {
                        if(switch_mode==false)
                        {
                            bt_frame::ep_goal goal;
                            goal.type = 0;
                            goal.x = poses[tag_id].pose.position.x;
                            goal.y = poses[tag_id].pose.position.y;
                            tf::Quaternion q;
                            tf::quaternionMsgToTF(poses[tag_id].pose.orientation, q);
                            double roll, pitch, yaw;
                            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                            goal.yaw = yaw;
                            goal_pub.publish(goal);
                            std::cout << "target position : " << goal.x << " " << goal.y << std::endl;
                            state = -1;
                        }
                    }
                }
                catch(const std::exception& e)
                {
                    ROS_ERROR("out of range: %s", e.what());
                    state = 1;
                    switch_mode = true;
                }
            }
            if(found[2])     
            {
                if(goal_reached && !detected)
                {
                    if(found[0])
                        state = 0;
                    else if(found[1])
                        state = 1;
                    else
                    {
                        if(state == 0)
                            state = 1;
                        else if(state == 1)
                            state = 2;
                        else if(state == 2)
                            state = 0;
                        else if(state == 3)
                            state = 0;
                        else if(state == -1)
                            state = 0;
                    }
                    // place3.clear();
                    std::cout << "state : " << state << std::endl;
                    std::cout << "loop to place" << state + 1 << std::endl;
                    goal_reached = false;
                    found[2] = false;
                    switch_mode = true;
                }
                int tag_id = -1;
                for(int k = 0;k < place1.size();k++)
                    if(place1[k] != -1)
                    {
                        tag_id = k;
                        break;
                    }
                try
                {
                    if(poses[tag_id].pose.position.x == 0 && poses[tag_id].pose.position.y == 0)
                    {
                        state = 2;
                        std::cout << "loop to place" << state + 1 << std::endl;
                        switch_mode = true;
                    }
                    else
                    {
                        if(switch_mode==false)
                        {
                            bt_frame::ep_goal goal;
                            goal.type = 0;
                            goal.x = poses[tag_id].pose.position.x;
                            goal.y = poses[tag_id].pose.position.y;
                            tf::Quaternion q;
                            tf::quaternionMsgToTF(poses[tag_id].pose.orientation, q);
                            double roll, pitch, yaw;
                            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                            goal.yaw = yaw;
                            goal_pub.publish(goal);
                            std::cout << "target position : " << goal.x << " " << goal.y << std::endl;
                            state = -1;
                        }
                    }
                }
                catch(const std::exception& e)
                {
                    ROS_ERROR("out of range: %s", e.what());
                    state = 2;
                    switch_mode = true;
                }
            }
            if(!found[0] && !found[1] && !found[2])
            {
                if(state == 0 && goal_reached)
                    state = 1;
                else if(state == 1 && goal_reached)
                    state = 2;
                else if(state == 2 && goal_reached)
                    state = 0;
                else if(state == 3 && goal_reached)
                    state = 0;
                else if(state == -1 && goal_reached)
                    state = 0;
                switch_mode = true;
                std::cout << "state : " << state << std::endl;
                std::cout << "loop to place" << state + 1 << std::endl;
            }
            
        }
        //just for first_find and collecting the position of the blocks
        std::cout << "switch mode : " << switch_mode << std::endl;
        switch (state)
        {
        case 0:
            //第一阶段 用movebase
            {
                if(!goal_reached && switch_mode)
                {
                    bt_frame::ep_goal goal;
                    goal.type=0;
                    goal.x=0.8;
                    goal.y=1.0;
                    goal.yaw=3.14;
                    goal_pub.publish(goal);
                    std::cout << "state: " << state << std::endl;
                    std::cout << "goal_reached: " << goal_reached << std::endl;
                }
                if(goal_reached)
                {
                    if(first_find || find_three_state)
                        state++;
                    goal_reached=false;
                    switch_mode = false;
                }   
            }
            break;
        case 1:
            {
                if(!goal_reached && switch_mode)
                {
                    bt_frame::ep_goal goal;
                    goal.type=0;
                    goal.x=1.88;
                    goal.y=-0.05;
                    goal.yaw=0;
                    goal_pub.publish(goal);
                    std::cout << "state: " << state << std::endl;
                    std::cout << "goal_reached: " << goal_reached << std::endl;
                    goal_pub.publish(goal);
                }
                
                if(goal_reached)
                {
                    if(first_find || find_three_state)
                        state++;
                    goal_reached=false;
                    switch_mode = false;
                }
            }
            break;
        case 2:
            {
                if(!goal_reached && switch_mode)
                {
                    bt_frame::ep_goal goal;
                    goal.type=0;
                    goal.type=0;
                    goal.x=1.3;
                    goal.y=2.9;
                    goal.yaw= 2.392;
                    goal_pub.publish(goal);

                    std::cout << "state: " << state << std::endl;
                    std::cout << "goal_reached: " << goal_reached << std::endl;

                    if(!first_find && !find_three_state)
                        state = 0;
                }
                if(goal_reached)
                {
                    if(first_find || find_three_state)
                        state++;
                    goal_reached=false;
                    switch_mode = false;
                    first_find=false;
                }
            }
            break;
        case 3:
            {
                std::cout << "state: " << state << std::endl;
                // if(find_three_state==true)
                {
                    state=0;
                }
            }
            break;
        default:
            break;
        }
        std_msgs::Int32 position_state;
        position_state.data=state;
        std::cout << "place1: ";
        printVector<int>(place1);
        std::cout << "place2: ";
        printVector<int>(place2);
        std::cout << "place3: ";
        printVector<int>(place3);
        std::cout << "finish: ";
        printVector<int>(finish);
        std::cout << "weight :";
        printVector<double>(weight);
        for(int i = 0;i < place1.size();i++)
        {
            for(int j = 0;j < finish.size();j++)
                if(place1[i] == finish[j])
                {
                    place1[i] = -1;
                    break;
                }
        }
        for(int i = 0;i < place2.size();i++)
        {
            for(int j = 0;j < finish.size();j++)
                if(place2[i] == finish[j])
                {
                    place2[i] = -1;
                    break;
                }
        }
        for(int i = 0;i < place3.size();i++)
        {
            for(int j = 0;j < finish.size();j++)
                if(place3[i] == finish[j])
                {
                    place3[i] = -1;
                    break;
                }
        }
        
        position_set_pub.publish(position_state);
        ros::spinOnce();
        loop_rate.sleep();        
    }

    return 0;
}
