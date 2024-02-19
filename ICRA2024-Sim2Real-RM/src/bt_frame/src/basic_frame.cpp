#include "basic_frame.h"
#include "simple_planner_node.h"
#include "goal_node.h"
#include "observe_node.h"
#include "take_node.h"
#include "goto_watch_board.h"
#include <ros/ros.h>
#include <bt_action_node.h>
#include <bt_service_node.h>

using namespace BT;
using namespace std;
namespace chr = std::chrono;

class Stop:public SyncActionNode
{
public:
    Stop(const std::string& name, const NodeConfig& config):SyncActionNode(name, config){}
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override
    {
        cout << "------     Robot stopping      ------\n";

        cout << "------     Robot stopped       ------\n";
        return NodeStatus::SUCCESS;
    }
};

class Place:public SyncActionNode
{
public:
    Place(const std::string& name, const NodeConfig& config):SyncActionNode(name, config){}
    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override
    {
        cout << "------     Robot placing block      ------\n";

        cout << "------     Robot placed block       ------\n";
        return NodeStatus::SUCCESS;
    }
};

static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree" _fullpath="">
        <Sequence name="Sequence">
            <GotoWatchBoard name="goto_watch_board"/>
            <Sequence name="find_there_and_back">
                <Sequence name="block_place">
                    <Goal name="next_block"/>
                    <SimplePlanner name="moveto_block"/>
                </Sequence>
                <Take name="take_block"/>
                <Sequence name="aim_to_board">
                    <Goal name="board"/>
                    <SimplePlanner name="moveto_board"/>
                </Sequence>
                <Place name="place_block"/>
            </Sequence>
            <Stop name="stop"/>
        </Sequence>
    </BehaviorTree>
</root>
)";

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

    NodeBuilder builder = [&nh](const std::string& name, const NodeConfiguration& config) {
        return std::make_unique<GotoWatchBoard>(nh,name, config);
    };
    TreeNodeManifest manifest;
    manifest.type = getType<GotoWatchBoard>();
    manifest.ports = GotoWatchBoard::providedPorts();
    manifest.registration_ID = "GotoWatchBoard";
    factory.registerBuilder(manifest, builder);
    factory.registerNodeType<Stop>("Stop");
    factory.registerNodeType<Goal>("Goal");
    factory.registerNodeType<SimplePlanner>("SimplePlanner");
    //factory.registerNodeType<Observe>("Observe");
    // factory.registerNodeType<GotoWatchBoard>("GotoWatchBoard");
    
    factory.registerNodeType<Take>("Take");
    factory.registerNodeType<Place>("Place");

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
    ros::Rate loop_rate(100); // 100 Hz
    while( ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
    {
        std::cout << "Start" << std::endl;
        tree.tickWhileRunning();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}