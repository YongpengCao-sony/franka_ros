#!/usr/bin/env python

import argparse
import math
import os
import sys
from copy import deepcopy

import moveit_commander
import rospy
import yaml
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from rospy_message_converter import message_converter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
]

EEF_LINK_NAME = "panda_hand"


class MoveGroupPythonInterface(object):
    """MoveGroupPythonInterface"""

    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface")

        robot = moveit_commander.RobotCommander()
        group_name = "penguin_robot"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())

    def get_current_link_state(self):
        pass


class TrajectoryClient:
    """for trajectory following"""

    def __init__(self):
        rospy.init_node("state_print")
        rospy.loginfo("Started the simple eef state retriever")

    def send_recorded_joint_trajectory(self):
        # make sure the correct controller is loaded and activated
        trajectory_client = SimpleActionClient(
            rospy.resolve_name('~follow_joint_trajectory'),
            FollowJointTrajectoryAction,
        )

        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        goal = FollowJointTrajectoryGoal()
        # go to the initial position at the beginning
        goal.trajectory.joint_names = JOINT_NAMES
        # position_init = [0.0, -1.17, 0.97, -1.39, -1.59, -3.14]

        point = JointTrajectoryPoint()
        point.positions = self.start_p
        point.time_from_start = rospy.Duration(5.0)
        point.velocities = [0] * len(self.start_p)
        goal.trajectory.points.append(point)
        goal.goal_time_tolerance = rospy.Duration.from_sec(0.5)

        input("press enter to move the robot to the start position")
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        # then follow the trajactory
        goal.trajectory = self.traj
        input("press enter to let the robot follow the trajectory")
        now = rospy.get_time()
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()
        end = rospy.get_time()
        time_spent_sec = end - now
        rospy.loginfo("spend {} seconds to execute the trajectory by joint position control".format(time_spent_sec))
        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))


if __name__ == "__main__":

    tutorial = MoveGroupPythonInterface()
