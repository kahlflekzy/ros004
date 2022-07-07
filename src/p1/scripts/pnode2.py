#! /usr/bin/python3

from p1.msg import MoveRobotAction, MoveRobotFeedback
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

import actionlib
import glob
import os
import rospy
import sys


class MoveRobotServer:
    WAITING = 1
    RUNNING = 2
    SUCCEEDED = 3

    def __init__(self, path=None, file=None, publish_initial_pose=True) -> None:
        if not file:
            self.file = glob.glob(path + "/goals_*.dat")[0]
        else:
            self.file = f"{path}/{file}"
        self.goals = None
        self.index = 0
        self.pose = None
        self.base_pose_stamped = None
        self.moved_from_base = False
        self.publish_initial_pose = publish_initial_pose
        self.status = self.WAITING
        self.current_goal = Goal()
        self.get_goals()
        rospy.loginfo("Starting publisher.")
        self.publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.init_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=5)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.worker)
        rospy.sleep(2)
        # rospy.loginfo(f"Goals File: {self.file}")
        self.init_pose(path)
        self.server = actionlib.SimpleActionServer("move_robot", MoveRobotAction, self.execute, False)
        rospy.loginfo("Starting server.")
        self.server.start()

    def execute(self, goal):
        """
        We get a goal from client, the use this to obtain a pose from a list of stored goals.
        Then we publish this goal to the topic '/move_base_simple/goal'.

        Next we should subscribe to a topic that tells us when this task is complete.

        :param goal:
        :return:
        """
        rospy.loginfo(f"Executing task: {goal}")
        if goal.goal_id == 0:
            pose = self.base_pose_stamped
            if not self.moved_from_base:
                self.server.set_succeeded()
                return
        else:
            pose = self.goal_to_pose(goal.goal_id - 1)
        #############################
        self.publisher.publish(pose)
        #############################
        feedback = MoveRobotFeedback()

        while self.current_goal.status != GoalStatus.SUCCEEDED:
            feedback.status = self.current_goal.status
            # for the feedback, maybe calculate euclidean distance to goal and represent it in %
            self.server.publish_feedback(feedback)
        # rospy.loginfo(f"Finished Task")
        self.current_goal = Goal()
        if goal.goal_id == 0:
            self.moved_from_base = False
        else:
            self.moved_from_base = True
        self.server.set_succeeded()

    def get_goals(self) -> bool:
        """"""
        with open(self.file) as file:
            self.goals = file.readlines()
            return True

    def goal_to_pose(self, index: int) -> PoseStamped:
        """
        Gets the index for a goal from a client and with it obtains a goal from a list of saved goals, then converts
        this to generate a PoseStamp.

        :param index:
        :return:
        """
        goal = self.goals[index].split(";")
        if len(goal) != 8:
            rospy.logerror(f"Invalid goal from goal file: '{self.file}'")
            raise Exception
        goals = list(map(float, goal[1:]))
        return self.create_pose_stamped(goal[0], goals)

    @staticmethod
    def create_pose_stamped(frame_id, goals):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = goals[0]
        pose_stamped.pose.position.y = goals[1]
        pose_stamped.pose.position.z = goals[2]
        pose_stamped.pose.orientation.x = goals[3]
        pose_stamped.pose.orientation.y = goals[4]
        pose_stamped.pose.orientation.z = goals[5]
        pose_stamped.pose.orientation.w = goals[6]
        return pose_stamped

    def init_pose(self, path) -> bool:
        rospy.loginfo("Getting initial pose")
        file = glob.glob(f"{path}/init_pose_*.dat")[0]
        with open(file) as file_obj:
            poses = file_obj.readlines()
            pose_str = poses[-1]
            pose = self.pose_to_pose_wc_stamped(pose_str, path)
            self.pose = pose
        if self.pose is not None and self.publish_initial_pose:
            rospy.loginfo("Publishing initial pose")
            self.init_pose_pub.publish(self.pose)
            # ensures we publish the initial pose estimate just once. And we might choose to skip this step.
            self.publish_initial_pose = False
        return True

    def pose_to_pose_wc_stamped(self, pose_str: str, path: str = "") -> PoseWithCovarianceStamped:
        """
        Converts our str representation of a PoseWithCovarianceStamped into an actual PoseWithCovarianceStamped msg

        :param pose_str:
        :param path:
        :return:
        """
        data = pose_str.split(';')
        if len(data) != 9:
            rospy.logerror(f"Invalid pose from pose file: '{path}'")
            raise Exception
        covariance = data.pop()
        frame_id = data.pop(0)
        data = list(map(float, data))
        pose_c_stamped = PoseWithCovarianceStamped()
        pose_c_stamped.header.frame_id = frame_id
        pose_c_stamped.header.stamp = rospy.Time.now()
        pose_c_stamped.pose.pose.position.x = data[0]
        pose_c_stamped.pose.pose.position.y = data[1]
        pose_c_stamped.pose.pose.position.z = data[2]
        pose_c_stamped.pose.pose.orientation.x = data[3]
        pose_c_stamped.pose.pose.orientation.y = data[4]
        pose_c_stamped.pose.pose.orientation.z = data[5]
        pose_c_stamped.pose.pose.orientation.w = data[6]
        pose_c_stamped.pose.covariance = list(map(float, covariance.split(',')))
        self.base_pose_stamped = self.create_pose_stamped(frame_id, data)
        return pose_c_stamped

    def worker(self, data):
        """
        Subscribe to
        - /move_base/current_goal,
        - /move_base/result, and
        - /move_base/status

        I discovered status works best.

        :return:
        """
        if len(data.status_list) != 0:
            _goal = data.status_list[0]
            if self.current_goal.watch_out_for_goal and _goal.status != _goal.SUCCEEDED:
                # elif self.current_goal.id == _goal.SUCCEEDED and _goal.goal_id.status != _goal.SUCCEEDED:
                # If current goal hasn't been set or the status of published goal is 1 (ACTIVE)
                self.current_goal.id = _goal.goal_id.id
                self.current_goal.status = _goal.status
                self.current_goal.watch_out_for_goal = False
            elif self.current_goal.id == _goal.goal_id.id:
                self.current_goal.status = _goal.status
        # if self.index % 50 == 0: rospy.loginfo(f"\n{data.status_list[0]}")
        self.index += 1


class Goal:
    id = None
    status = MoveRobotServer.WAITING
    watch_out_for_goal = True


if __name__ == '__main__':
    rospy.init_node("p1_python_move_robot_server")
    rospy.loginfo("Initialized Node.")
    if len(sys.argv) > 1:
        # Ran from launch file
        init_base_pose = True if sys.argv[2] == "1" else False
        _path = sys.argv[1].rstrip('p1') + "data"
    else:
        _path = os.getcwd() + "/src/data"
        init_base_pose = True
    s = MoveRobotServer(_path, publish_initial_pose=init_base_pose)
    rospy.spin()
