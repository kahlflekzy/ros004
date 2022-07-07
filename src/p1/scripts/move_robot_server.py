#! /usr/bin/python3

import actionlib
import roslib
import rospy
roslib.load_manifest("p1")

from p1.msg import MoveRobotAction

class MoveRobotServer:
    def __init__(self) -> None:
        self.server = actionlib.SimpleActionServer("move_robot", MoveRobotAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        """
        Do alot of awesome groundbreaking robot stuff here
        :param goal:
        :return:
        """
        rospy.loginfo(f"executing task: {goal}")
        rospy.loginfo(f"{dir(self.server)}")
        rospy.loginfo(f"{help(self.server.publish_feedback)}")
        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node("p1_python_move_robot_server")
    rospy.loginfo("initialized node")
    s = MoveRobotServer()
    rospy.spin()
