#!/usr/bin/env python

import os, sys
import rospy 
import yaml
import argparse

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from rospy_message_converter import message_converter
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult

JOINT_NAMES = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]

class TrajectoryClient:
    """for trajectory following"""
    
    def __init__(self):
        rospy.init_node("traj_follow")
        self.traj_date = rospy.get_param(rospy.resolve_name('~traj_date'))
        self.traj_robot = rospy.get_param(rospy.resolve_name('~traj_robot'))
        self.traj_name = rospy.get_param(rospy.resolve_name('~traj_name'))
        self.traj_speed = rospy.get_param(rospy.resolve_name('~traj_speed'))
        self.traj_path = rospy.get_param(rospy.resolve_name('~traj_path'))

        rospy.loginfo("laod the trajectory file: "+ self.traj_path + '{}_{}_{}_{}_trajectory.yaml'.format(self.traj_date, self.traj_robot, self.traj_name, self.traj_speed))
        with open(
                self.traj_path + '{}_{}_{}_{}_trajectory.yaml'.format(self.traj_date, self.traj_robot, self.traj_name, self.traj_speed),
                'r') as file:
            prime_service = yaml.safe_load(file)
    
        self.traj = message_converter.convert_dictionary_to_ros_message(
            'trajectory_msgs/JointTrajectory', prime_service)
        self.start_p = prime_service['points'][0]['positions']
        # print("the start position of the loaded yaml file is ", start_p)
            
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
        point.velocities = [0]*len(self.start_p)
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
    
    client = TrajectoryClient()
    client.send_recorded_joint_trajectory()


def original_follow_func():
    max_movement = max(abs(pose[joint] - initial_pose[joint]) for joint in pose)

    point = JointTrajectoryPoint()
    point.time_from_start = ros.Duration.from_sec(
        # Use either the time to move the furthest joint with 'max_dq' or 500ms,
        # whatever is greater
        max(max_movement / ros.get_param('~max_dq', 0.5), 0.5)
    )

    current_path = os.getcwd()
    with open(current_path +
            '/src/test_scripts/src/20230418_155932_trajectory.yaml',
            'r') as file:
        prime_service = yaml.safe_load(file)
    traj = message_converter.convert_dictionary_to_ros_message(
        'trajectory_msgs/JointTrajectory', prime_service)

    start_p = prime_service['points'][0]['positions']
    goal = FollowJointTrajectoryGoal()

    goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*pose.items())]
    point.velocities = [0] * len(pose)

    goal.trajectory.points.append(point)
    goal.goal_time_tolerance = ros.Duration.from_sec(0.5)

    ros.loginfo('Sending trajectory Goal to move into initial config')
    client.send_goal_and_wait(goal)

    result = client.get_result()
    if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
        ros.logerr('move_to_start: Movement was not successful: ' + {
            FollowJointTrajectoryResult.INVALID_GOAL:
            """
            The joint pose you want to move to is invalid (e.g. unreachable, singularity...).
            Is the 'joint_pose' reachable?
            """,

            FollowJointTrajectoryResult.INVALID_JOINTS:
            """
            The joint pose you specified is for different joints than the joint trajectory controller
            is claiming. Does you 'joint_pose' include all 7 joints of the robot?
            """,

            FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
            """
            During the motion the robot deviated from the planned path too much. Is something blocking
            the robot?
            """,

            FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
            """
            After the motion the robot deviated from the desired goal pose too much. Probably the robot
            didn't reach the joint_pose properly
            """,
        }[result.error_code])

    else:
        ros.loginfo('move_to_start: Successfully moved into start pose')


