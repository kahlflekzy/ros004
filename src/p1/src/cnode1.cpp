#include "ros/ros.h"
#include <p1/MoveRobotAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<p1::MoveRobotAction> Client;

int main(int argc, char** argv){
    ros::init(argc,argv, "move_robot_client_node_test");

    Client client("move_robot", true); // true -> don't need ros::spin()
    client.waitForServer();
    p1::MoveRobotGoal goal;
    goal.goal_id = 1; // build goal here
    client.sendGoal(goal); 
    client.waitForResult(ros::Duration(5.0));
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        printf("Yay! Robot has moved");
    }
    printf("Current State: %s\n", client.getState().toString().c_str());
    return 0;
}