#include "basic_frame.h"
#include "simple_planner_node.h"
#include "goal_node.h"
#include "observe_node.h"
#include "take_node.h"
#include "goto_watch_board.h"
#include <ros/ros.h>
#include <bt_action_node.h>
#include <bt_service_node.h>
#include <check_done.h>
#include "place_node.h"
#include "take_up_node.h"
#include "place_down_node.h"
#include "check_up_done.h"
#include "go_to_stop_place.h"
#include "time_out.h"
using namespace BT;
using namespace std;
namespace chr = std::chrono;

class Stop:public StatefulActionNode
{
public:
    Stop(const std::string& name, const NodeConfig& config):StatefulActionNode(name, config){}
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
        return BT::NodeStatus::RUNNING;
    }
    NodeStatus onRunning() override
    {
        auto res1=getInput<int>("target_cube_num1");
        if( !res1 )
        {
            throw RuntimeError("error reading port [target]:", res1.error());
        }
        cout<<res1.value();
        auto res2=getInput<int>("target_cube_num2");
        if( !res2 )
        {
            throw RuntimeError("error reading port [target]:", res2.error());
        }
        cout<<res2.value();
        auto res3=getInput<int>("target_cube_num3");
        if( !res3 )
        {
            throw RuntimeError("error reading port [target]:", res3.error());
        }
        cout<<res3.value();
        
        cout<<endl;
        std::cout << "Stop running" << std::endl;
        return BT::NodeStatus::RUNNING;
    }
    void onHalted() override
    {
      // nothing to do here...
      std::cout << "Stop interrupted" << std::endl;
    }
};


// static const char* xml_text = R"(
// <root BTCPP_format="4">
//     <BehaviorTree ID="MainTree" _fullpath="">
//         <Sequence name="Sequence">
//             <GotoWatchBoard name="goto_watch_board"/>
//             <Sequence name="find_there_and_back">
//                 <Sequence name="block_place">
//                     <SimplePlanner name="moveto_block"/>
//                 </Sequence>
//                 <Take name="take_block"/>
//                 <Sequence name="aim_to_board">
//                     <SimplePlanner name="moveto_board"/>
//                 </Sequence>
//                 <Place name="place_block"/>
//             </Sequence>
//             <Stop name="stop"/>
//         </Sequence>
//     </BehaviorTree>
// </root>
// )";

static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree" _fullpath="">
    <Sequence name="Sequence">
        <Parallel success_count="1">
            <Sequence name="Sequence">
                <GotoWatchBoard name="goto_watch_board" target_cube_num1="{target_cube_num1}" target_cube_num2="{target_cube_num2}" target_cube_num3="{target_cube_num3}"/>
                <RetryUntilSuccessful num_attempts="100">
                    <Sequence>
                        <SimplePlanner name="simple_planner"/>
                        <Take name="take" target_cube_num1="{target_cube_num1}" target_cube_num2="{target_cube_num2}" target_cube_num3="{target_cube_num3}" takeing_cube_num="{takeing_cube_num}"/>
                        <Place name="place" target_cube_num1="{target_cube_num1}" target_cube_num2="{target_cube_num2}" target_cube_num3="{target_cube_num3}" takeing_cube_num="{takeing_cube_num}"/>
                        <CheckDone name="check_done" target_cube_num1="{target_cube_num1}" target_cube_num2="{target_cube_num2}" target_cube_num3="{target_cube_num3}"/>
                    </Sequence>
                </RetryUntilSuccessful>
                <RetryUntilSuccessful num_attempts="100">
                    <Sequence>
                        <SimplePlanner name="simple_planner"/>
                        <Take_Up name="take up"/>
                        <Place_Down name="Place down" target_cube_num2="{target_cube_num2}" arm_high="{arm_high}"/>
                        <Check_up_done name="Check_up_done" arm_high="{arm_high}"/>
                    </Sequence>
                </RetryUntilSuccessful>
            </Sequence>
            <TimeOut name="timeout"/>
        </Parallel>
        <GotoStop name="goto_stop"/>
        <Stop name="stop" target_cube_num1="{target_cube_num1}" target_cube_num2="{target_cube_num2}" target_cube_num3="{target_cube_num3}"/>
    </Sequence>
   
    </BehaviorTree>
</root>
)";

            // <RetryUntilSuccessful num_attempts="100">
            //     <Sequence>
            //         <SimplePlanner name="simple_planner"/>
            //         <Take_Up name="take up"/>
            //         <Place_Down name="Place down"/>
            //         <CheckDone name="check_done"/>
            //     </Sequence>
            // </RetryUntilSuccessful>

// static const char* xml_text = R"(
// <root BTCPP_format="4">
//     <BehaviorTree ID="MainTree" _fullpath="">
//         <Sequence name="Sequence">
//             <Take name="take" target_cube_num1="{target_cube_num1}" target_cube_num2="{target_cube_num2}" target_cube_num3="{target_cube_num3}" takeing_cube_num="{takeing_cube_num}"/>
//             <Place name="place" target_cube_num1="{target_cube_num1}" target_cube_num2="{target_cube_num2}" target_cube_num3="{target_cube_num3}" takeing_cube_num="{takeing_cube_num}"/>
//             <Stop name="stop" target_cube_num1="{target_cube_num1}" target_cube_num2="{target_cube_num2}" target_cube_num3="{target_cube_num3}"/>
//         </Sequence>
//     </BehaviorTree>
// </root>
// )";
// A custom structuree that I want to visualize in Groot2
struct Position2D {
  double x;
  double y;
};

// Allows Position2D to be visualized in Groot2
// You still need BT::RegisterJsonDefinition<Position2D>(PositionToJson)
void PositionToJson(nlohmann::json& j, const Position2D& p)
{
  j["x"] = p.x;
  j["y"] = p.y;
}


template <class T> static void RosBuilder(BehaviorTreeFactory& factory, const std::string& ID, ros::NodeHandle& nh)
{
    NodeBuilder builder = [&nh](const std::string& name, const NodeConfiguration& config) {
        return std::make_unique<T>(nh, name, config);
    };
    TreeNodeManifest manifest;
    manifest.type = getType<T>();
    manifest.ports = T::providedPorts();
    manifest.registration_ID = ID;
    factory.registerBuilder(manifest, builder);
}

/**
 * @brief The entry point of the program.
 * 
 * This function initializes the BehaviorTreeFactory, registers node types,
 * creates a behavior tree, and starts the main loop that ticks the tree.
 * 
 * @return int The exit status of the program.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "bt_node");
    ros::NodeHandle nh;

    BehaviorTreeFactory factory;

    
    RosBuilder<GotoWatchBoard>(factory, "GotoWatchBoard", nh);
    RosBuilder<SimplePlanner>(factory,"SimplePlanner",nh);
    RosBuilder<Take>(factory,"Take",nh);
    RosBuilder<Take_Up>(factory, "Take_Up",nh);
    RosBuilder<Place>(factory,"Place",nh);
    RosBuilder<Place_Down>(factory,"Place_Down",nh);
    RosBuilder<Check_done>(factory,"CheckDone",nh);
    RosBuilder<Check_up_done>(factory,"Check_up_done",nh);
    RosBuilder<GotoStop>(factory,"GotoStop",nh);
    RosBuilder<Timeout>(factory,"TimeOut",nh);
    factory.registerNodeType<Stop>("Stop");
    // factory.registerNodeType<Check_done>("CheckDone");
    //factory.registerNodeType<Goal>("Goal");
    // factory.registerNodeType<SimplePlanner>("SimplePlanner");
    //factory.registerNodeType<Observe>("Observe");
    // factory.registerNodeType<GotoWatchBoard>("GotoWatchBoard");
    
    //factory.registerNodeType<Take>("Take");
    //factory.registerNodeType<Place>("Place");

    std::string xml_models = BT::writeTreeNodesModelXML(factory); 

    factory.registerBehaviorTreeFromText(xml_text);

    BT::RegisterJsonDefinition<Position2D>(PositionToJson);
    auto tree = factory.createTree("MainTree");

    cout << "----------- XML file  ----------\n"
        << BT::WriteTreeToXML(tree, false, false)
        << "--------------------------------\n";

    const unsigned port = 1667;
    Groot2Publisher publisher(tree, port);

    // Add two more loggers, to save the transitions into a file.
    // Both formats are compatible with Groot2

    // Lightweight serialization
    BT::FileLogger2 logger2(tree, "t12_logger2.btlog");
    // SQLite logger can save multiple sessions into the same database
    bool append_to_database = true;
    BT::SqliteLogger sqlite_logger(tree, "t12_sqlitelog.db3", append_to_database);
    NodeStatus status = NodeStatus::IDLE;
    ros::Rate loop_rate(10); // 10 Hz
    while( ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
    {
        ros::spinOnce();
        // std::cout << "Start" << std::endl;
        tree.tickOnce();
        //tree.sleep(std::chrono::milliseconds(100));
        loop_rate.sleep();
    }
    return 0;
}