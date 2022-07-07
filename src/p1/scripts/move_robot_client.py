#! /usr/bin/python3

import roslib
roslib.load_manifest('p1') # maybe not needed
import rospy
import actionlib

from p1.msg import MoveRobotAction, MoveRobotGoal

if __name__ == '__main__':
    rospy.init_node('move_robot_client')
    client = actionlib.SimpleActionClient('move_robot', MoveRobotAction)
    client.wait_for_server()

    goal = MoveRobotGoal()
    goal.goal_id = 1
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
