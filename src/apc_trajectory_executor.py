#!/usr/bin/env python

from __future__ import division

import roslib
roslib.load_manifest('apc')
import rospy
from trajectory_msgs.msg import JointTrajectory

from ros_utils import ROSNode
from apc.msg import ExecStatus


__author__ = 'dibyo'


class APCTrajectoryExecutor(ROSNode):
    def __init__(self, joint_trajectories_topic, exec_status_topic):
        super(APCTrajectoryExecutor, self).__init__('execute')

        self.joint_trajectories_topic = joint_trajectories_topic
        self.exec_status_topic = exec_status_topic

        self.joint_trajectories_subscriber = rospy.Subscriber(self.joint_trajectories_topic,
                                                              JointTrajectory,
                                                              self.execute_joint_trajectory)
        self.exec_status_publisher = rospy.Publisher(self.exec_status_topic, ExecStatus)

    def execute_joint_trajectory(self, trajectory):
        pass