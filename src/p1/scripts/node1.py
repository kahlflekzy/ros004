#! /usr/bin/python3
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import os
import pathlib
import datetime


class Subscriber:
    """
    This class subscribes to the topic `/move_base_simple/goal` and prints it to console.
    """

    def __init__(self) -> None:
        rospy.init_node("p1_node1")
        rospy.loginfo("Initialize Node")
        path = pathlib.Path(f"{os.getcwd()}/src/data/")
        if not os.path.exists(path):
            os.mkdir(path)
        d = datetime.datetime.now().strftime("%y_%m_%d_%H_%M_%S")
        self.goals_file = path / f"goals_{d}.dat"
        self.init_pose_file = path / f"init_pose_{d}.dat"
        rospy.loginfo(f"Goals File: {self.goals_file}")
        rospy.loginfo(f"Initial Pose File: {self.init_pose_file}")
        rospy.Subscriber("/move_base_simple/goal2", PoseStamped, self.handle_goal)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.handle_pose)

    def __call__(self, *args, **kwargs):
        rospy.spin()

    def handle_goal(self, data: PoseStamped):
        """
        Store data in the format [frame_id;px;py;pz;qx;qy;qz;qw]

        :param data:
        :return:
        """
        with open(self.goals_file, mode='a') as file:
            pose = data.pose
            msg = [data.header.frame_id,
                   pose.position.x, pose.position.y, pose.position.z,
                   pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            msg = ";".join(map(str, msg))
            msg += "\n"
            file.write(msg)
        # rospy.loginfo(f'{rospy.get_caller_id()} ->: Goal \n{data}')

    def handle_pose(self, data):
        with open(self.init_pose_file, mode='a') as file:
            pose = data.pose.pose
            msg = [data.header.frame_id,
                   pose.position.x, pose.position.y, pose.position.z,
                   pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                   ','.join(map(str, data.pose.covariance))
                   ]
            msg = ";".join(map(str, msg))
            msg += "\n"
            file.write(msg)
        rospy.loginfo(f'{rospy.get_caller_id()} -> Initial Pose: \n{data}')


if __name__ == '__main__':
    s = Subscriber()
    s()

###
# map;-1.990435230601741;-0.5068233524133372;0.0;0.0;0.0;-0.008290220848190994;0.9999656355286857;0.17851186247617834,-0.023272976025515257,0.0,0.0,0.0,0.0,-0.023272976025515257,0.2053658417787348,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.06211765171450785
###
