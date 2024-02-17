#include <ros/ros.h>
#include <actionlib_msgs/GoalStatus.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
// #include <rmus_solution/switch.h>
// #include <rmus_solution/graspsignal.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <thread>
#include <cmath>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <deque>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/loggers/bt_sqlite_logger.h"
#include "behaviortree_cpp/json_export.h"

// #include "bt_factory.h"
// #include "xml_parsing.h"
// #include "dummy_nodes.h"
// #include "loggers/bt_file_logger_v2.h"
// #include "loggers/groot2_publisher.h"
// #include "loggers/bt_sqlite_logger.h"
// #include "movebase_node.h"