#!/usr/bin/env python

from __future__ import division

import roslib
roslib.load_manifest('apc')
import rospy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from pr2.arm import Arm

from ros_utils import ROSNode
from apc.msg import ExecStatus
from utils import LoopingThread


__author__ = 'dibyo'


class APCTrajectoryExecutor(ROSNode):
    def __init__(self, joint_trajectories_topic, exec_status_topic):
        super(APCTrajectoryExecutor, self).__init__('execute')

        self.joint_trajectories_topic = joint_trajectories_topic
        self.exec_status_topic = exec_status_topic
        self.exec_status = ExecStatus.IDLE

        self.larm = Arm('left')
        self.larm_gripper_name = 'l_gripper_l_finger_joint'
        self.rarm = Arm('right')
        self.rarm_gripper_name = 'r_gripper_l_finger_joint'

        self.joint_trajectories_subscriber = rospy.Subscriber(self.joint_trajectories_topic,
                                                              JointTrajectory,
                                                              self.execute_joint_trajectory)

        self.exec_status_publisher = rospy.Publisher(self.exec_status_topic, ExecStatus)
        self.exec_status_publisher_thread = LoopingThread(target=self.publish_exec_status,
                                                          rate=50)
        self.exec_status_publisher_thread.start()

    def publish_exec_status(self):
        header = Header()
        header.stamp(rospy.Time.now())
        self.exec_status_publisher.publish(ExecStatus(header, self.exec_status))

    def execute_joint_trajectory(self, trajectory):
        # TODO: Add handling for speed

        try:
            joint_names = trajectory.joint_names
            if joint_names[0][0] == 'l':
                arm = self.larm
            elif joint_names[0][0] == 'r':
                arm = self.rarm
            else:  # TODO: Add handling for torso joint
                return

            if len(joint_names) == 1:
                if trajectory.points[0][0] == 0:
                    arm.close_gripper(block=False)
                else:
                    arm.open_gripper(block=False)
            elif len(joint_names) > 1:
                self.exec_status = ExecStatus.BUSY
                arm.execute_joint_trajectory(trajectory.points)
                self.exec_status = ExecStatus.IDLE
        except rospy.ROSException:
            self.exec_status = ExecStatus.ERROR