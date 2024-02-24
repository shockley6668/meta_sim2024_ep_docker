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
using namespace std;

bool goal_reached=false;
bool first_find=true;
int state;
bool send_flag=false;
vector <int> place1;
vector <int> place2;
vector <int> place3;
bool no_move=false;
bool find_three_state=true;
vector <int> target_cube_num;
apriltag_ros::AprilTagDetectionArray tag_msg;
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
}
void no_move_callback(const std_msgs::Int32::ConstPtr &msg)
{
    if(msg->data==1)
    {
        if(tag_msg.detections.size()>0)
        {
            switch (state)
            {
            case 1:
                for(int i=0;i<tag_msg.detections.size();i++)
                {
                    if (std::find(place1.begin(), place1.end(), tag_msg.detections[i].id[0]) == place1.end()) {
                        place1.push_back(tag_msg.detections[i].id[0]);  
                    }
                }
                break;
            case 2:
                for(int i=0;i<tag_msg.detections.size();i++)
                {
                    if (std::find(place2.begin(), place2.end(), tag_msg.detections[i].id[0]) == place2.end()) {
                        place2.push_back(tag_msg.detections[i].id[0]);  
                    }
                }
                break;
            case 3:
                for(int i=0;i<tag_msg.detections.size();i++)
                {
                    if (std::find(place3.begin(), place3.end(), tag_msg.detections[i].id[0]) == place3.end()) {
                        place3.push_back(tag_msg.detections[i].id[0]);  
                    }
                }
                break;
            default:
                break;
            }
        }
    }
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
    ros::Subscriber goal_status_sub=nh.subscribe<std_msgs::Int32>("ep_goal_status",10,goal_status_callback);
    ros::Rate loop_rate(100);
    ros::ServiceClient change_point_client = nh.serviceClient<simple_planner::change_point_num>("/change_point_num");
    ros::Subscriber tag_sub = nh.subscribe("/tag_detections", 10, &tagCallback);
    ros::Subscriber no_move_sub=nh.subscribe("no_move",10,no_move_callback);
    ros::Subscriber target_num_sub=nh.subscribe<std_msgs::Int32MultiArray>("target_cube_num",10,target_num_callback);
    ros::Publisher position_set_pub = nh.advertise<std_msgs::Int32>("position_state", 10);
    ros::Publisher tar_tag = nh.advertise<std_msgs::Int32>("tar_tag", 10);
    ros::Subscriber position_state_sub = nh.subscribe<std_msgs::Int32>("check_done", 10, [&](const std_msgs::Int32::ConstPtr &msg){
        if(msg->data == 1)
        {
            find_three_state = false;
        }
        else{
            find_three_state = true;
        }
    });
    state=0;
    loop_rate.sleep();
    while (ros::ok())
    {
    
        
        if(find_three_state==false&&first_find==false)
        {

            std::cout<<"find_three_state"<<find_three_state<<std::endl;
            if(!place1.empty())
            {
                state = 0;
                std_msgs::Int32 tar;
                tar.data = place1[0];
                tar_tag.publish(tar);
                std::cout<<"place1"<<std::endl;
                printVector<int>(place1);

            }
            else if(!place2.empty())
            {
                state = 1;
                std_msgs::Int32 tar;
                tar.data = place2[0];
                tar_tag.publish(tar);
                std::cout<<"place2"<<std::endl;
                printVector<int>(place2);

            }
            else if(!place3.empty())
            {
                state = 2;
                std_msgs::Int32 tar;
                tar.data = place3[0];
                tar_tag.publish(tar);
                std::cout<<"place3"<<std::endl;
                printVector<int>(place3);
   
            }
            else{
                state=0;
            }
            
            
            
        }
        else if(first_find==false&&find_three_state==true){
            int i=0;
            for( i=0;i<3;i++)
            {
                if(std::find(place1.begin(), place1.end(), target_cube_num[i]) != place1.end())
                {
                    place1.erase(std::remove(place1.begin(), place1.end(), target_cube_num[i]), place1.end()); //删除place1中的目标
                    state=0;
                    printVector<int>(place1);
                    break;
                }
                else if(std::find(place2.begin(), place2.end(), target_cube_num[i]) != place2.end())
                {
                    place2.erase(std::remove(place2.begin(), place2.end(), target_cube_num[i]), place2.end()); //删除place2中的目标
                    state=1;
                    printVector<int>(place2);
                    break;
                }
                else if(std::find(place3.begin(), place3.end(), target_cube_num[i]) != place3.end())
                {
                    place3.erase(std::remove(place3.begin(), place3.end(), target_cube_num[i]), place3.end()); //删除place3中的目标
                    state=2;
                    printVector<int>(place3);
                    break;
                }
            }
                if(i>=3)
                {
                    state=0;
                    first_find=true;
                }
        }
        
        switch (state)
        {
        case 0:
            //第一阶段 用movebase
            {
               
                bt_frame::ep_goal goal;
                goal.type=0;
                goal.x=0.8;
                goal.y=1.0;
                goal.yaw=3.14;
                goal_pub.publish(goal);
                
                if(goal_reached)
                {
                    state++;
                    goal_reached=false;
                }   
            }
            break;
        case 1:
            {
                // Node start(2 ,1), end(4,1);
                // vector<pair<int, int>> path = dijkstra(grid, start, end);
                bt_frame::ep_goal goal;
                simple_planner::change_point_num ch_t;
                ch_t.request.point_num=5;
                float arrx[5]={1.0,1.55,2.0,2.0,1.86};
                float arry[5]={1.0,1.0,1.0,0.6,-0.1};
                goal.type=1;
                // if(change_point_client.call(ch_t))
                // {
                //     std::cout<<"change point num to"<<ch_t.request.point_num<<std::endl;
                //     for (int i=0;i<5;i++) {
                //     geometry_msgs::PointStamped point_t;
                //     point_t.point.x=arrx[i];
                //     point_t.point.y=arry[i];
                //     goal.points.push_back(point_t);    
                //     }
                //     goal_pub.publish(goal);
                // }
                
                for (int i=0;i<5;i++) {
                    geometry_msgs::PointStamped point_t;
                    point_t.point.x=arrx[i];
                    point_t.point.y=arry[i];
                    goal.points.push_back(point_t);    
                }
                goal_pub.publish(goal);
                if(goal_reached)
                {
                    state++;
                    goal_reached=false;
                }
            }
            break;
        case 2:
            {
                bt_frame::ep_goal goal;
                goal.type=0;
                goal.type=0;
                goal.x=1.3;
                goal.y=2.9;
                goal.yaw= 2.392;
                goal_pub.publish(goal);
                
                if(goal_reached)
                {
                    state++;
                    goal_reached=false;
                    first_find=false;
                }
            }
            break;
        case 3:
            {
                if(find_three_state==true)
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

        position_set_pub.publish(position_state);
        ros::spinOnce();
        loop_rate.sleep();        
    }
 
    return 0;
}
