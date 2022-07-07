#include <p1/MoveRobotAction.h>
#include <p1/MoveRobotFeedback.h>
#include <actionlib/client/simple_action_client.h>
#include "behaviortree_cpp_v3/action_node.h"

typedef actionlib::SimpleActionClient<p1::MoveRobotAction> Client;

using namespace BT;
using std::cout;
using std::endl;
using namespace p1;

struct GoalID
{
    int goal_id;
};

namespace BT
{
    template <> inline GoalID convertFromString(StringView str){
        GoalID goal;
        goal.goal_id = convertFromString<int>(str);
        return goal;
    }
} // namespace BT


class MoveRobot : public BT::StatefulActionNode
{
    private:
        Client client;
        p1::MoveRobotGoal goal;

    public:
        MoveRobot(const std::string& name, const BT::NodeConfiguration& config):
                    BT::StatefulActionNode(name, config), client("move_robot", true){}
        
        static BT::PortsList providedPorts(){
            return {BT::InputPort<int>("goal")};
        }

        NodeStatus onStart() override {
            int g = 0;
            getInput("goal", g);
            goal.goal_id = g;
            cout<<">>> Got goal: "<<g<<endl;
            initClient();
            client.sendGoal(goal,
                            Client::SimpleDoneCallback(),
                            Client::SimpleActiveCallback(),
                            boost::bind(&MoveRobot::onFeedback, this, _1));
            cout<<">>> Finished starting."<<endl;
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override {
            if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                cout<<">>> Robot has gotten to it's destination!\n"<<endl;
                return NodeStatus::SUCCESS;
            }else{
//                printf("Current State: %s\n", client.getState().toString().c_str());
                return NodeStatus::RUNNING;
            }
        }

        void onHalted() override {
            // notify the server that the operation have been aborted. NOT_IMPLEMENTED FOR NOW
            cout<<">>> MoveRobot was Halted"<<endl;
        }

        void initClient() {
            cout<<">>> Waiting for server."<<endl;
            client.waitForServer(ros::Duration(5.0));
        }

        void onFeedback(const MoveRobotFeedbackConstPtr& feedback) {
//            cout<<feedback->status<<endl;
        }
};
