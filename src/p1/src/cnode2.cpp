#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include "header_nodes.h"

using namespace BT;
using std::string;
using std::cout;
using std::endl;
using std::chrono::milliseconds;

int main(int argc, char** argv){
    ros::init(argc,argv, "move_robot_client_node");
    cout<<">>> Init Client."<<endl;

    char tmp[256];
    string path;
    getcwd(tmp, 256);
    path = string(tmp) + "/src/p1/src/xml/tree10.xml";

    /*for (int i = 0; i < argc; i++)
    {
        cout<<argv[i]<<endl;
    }*/

    if (argc > 1)
    {
        path = string(argv[1]) + "/src/xml/tree10.xml";
    }

    cout<<path<<endl;
    
    BehaviorTreeFactory factory;

    factory.registerNodeType<MoveRobot>("MoveRobot");

    auto tree = factory.createTreeFromFile(path);
    
    NodeStatus status = NodeStatus::RUNNING;
    cout<<">>> Starting Tree."<<endl;
    
    while (status == NodeStatus::RUNNING)
        {
            status = tree.tickRoot();
            /*
            It is important to always add some sleep if you call tickRoot() in a loop to avoid using 100% of your CPU.
            Again Tree.sleep() is recommended. See online tutorials for details.
            */
            tree.sleep(milliseconds(10));
        }
    cout<<">>> Finished Tree Execution!"<<endl;
    return 0;
}