#include "std_msgs/Int32.h"
#include "ros/ros.h"
#include "bt_frame/ep_goal.h"
#include <queue>
#include <vector>
#include <cstdlib>
#include <climits>
#include "simple_planner/change_point_num.h"
using namespace std;

bool goal_reached=false;
int state;
bool send_flag=false;
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
int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_decition_node");
    ros::NodeHandle nh;
    
    ros::Publisher goal_pub = nh.advertise<bt_frame::ep_goal>("ep_next_goal", 10);
    ros::Subscriber goal_status_sub=nh.subscribe<std_msgs::Int32>("ep_goal_status",10,goal_status_callback);
    ros::Rate loop_rate(100);
    ros::ServiceClient change_point_client = nh.serviceClient<simple_planner::change_point_num>("/change_point_num");

    
    state=0;
    loop_rate.sleep();
    while (ros::ok())
    {
        std::cout<<state<<std::endl;
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
                float arrx[5]={1.0,1.55,2.0,2.0,1.92};
                float arry[5]={1.0,1.0,1.0,0.6,0.18};
                goal.type=1;
                if(change_point_client.call(ch_t))
                {
                    std::cout<<"change point num to"<<ch_t.request.point_num<<std::endl;
                    for (int i=0;i<5;i++) {
                    geometry_msgs::PointStamped point_t;
                    point_t.point.x=arrx[i];
                    point_t.point.y=arry[i];
                    goal.points.push_back(point_t);    
                    }
                    goal_pub.publish(goal);
                }
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
                goal.x=1.53;
                goal.y=3.22;
                goal.yaw=3.14;
                goal_pub.publish(goal);
                
                if(goal_reached)
                {
                    state++;
                    goal_reached=false;
                }
            }
            break;
        default:
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();        
    }
    
    return 0;
}
