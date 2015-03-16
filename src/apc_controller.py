#!/usr/bin/env python

from __future__ import division

import roslib
roslib.load_manifest('apc')

import rospy
from ros_utils import ROSNode
from trajectory_msgs.msg import JointTrajectory

from apc.msg import BinWorkOrder, ExecStatus
from apc.srv import GetMotionPlan

__author__ = 'dibyo'


class APCController(ROSNode):
    def __init__(self, joint_trajectories_topic, exec_status_topic):
        super(APCController, self).__init__('apc_controller')

        self.joint_trajectories_topic = joint_trajectories_topic
        self.exec_status_topic = exec_status_topic

        self.robot_exec_status = None

        self.get_motion_plan_client = rospy.ServiceProxy('get_motion_plan',
                                                         GetMotionPlan)
        self.joint_trajectories_publisher = rospy.Publisher(self.joint_trajectories_topic,
                                                            JointTrajectory)
        self.exec_status_subscriber = rospy.Subscriber(self.exec_status_topic,
                                                       ExecStatus,
                                                       self.record_exec_status)

    def record_exec_status(self, status):
        self.robot_exec_status = status

    def execute_strategy(self, strategy):
        pass