#!/usr/bin/env python

from __future__ import division

import roslib
roslib.load_manifest('apc')
import rospy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from pr2.arm import Arm

from apc.msg import ExecStatus
from ros_utils import ROSNode
from utils import LoopingThread


__author__ = 'dibyo'


class APCTrajectoryExecutor(ROSNode):
    class ExecStatusContextManager(object):
        def __init__(self, topic):
            self.topic = topic

            self.status = ExecStatus.IDLE

            self.publisher = rospy.Publisher(topic,
                                             ExecStatus)
            self.publisher_thread = LoopingThread(target=self.publish,
                                                  rate=50)

        def __enter__(self):
            self.status = ExecStatus.BUSY
            return self

        def __exit__(self, exc_type, exc_val, exc_tb):
            self.status = ExecStatus.IDLE

        def publish(self):
            header = Header()
            header.stamp = rospy.Time.now()
            self.publisher.publish(ExecStatus(header, self.status))

    def __init__(self, joint_trajectories_topic, exec_status_topic):
        super(APCTrajectoryExecutor, self).__init__('execute')

        self.joint_trajectories_topic = joint_trajectories_topic
        self.exec_status_topic = exec_status_topic
        self.exec_status = ExecStatus.IDLE

        self.larm = Arm('left', default_speed=0.3)
        self.larm_gripper_name = 'l_gripper_l_finger_joint'
        # self.rarm = Arm('right')
        # self.rarm_gripper_name = 'r_gripper_l_finger_joint'

        self.joint_trajectories_subscriber = rospy.Subscriber(self.joint_trajectories_topic,
                                                              JointTrajectory,
                                                              self.execute_joint_trajectory)

        self.exec_status_publisher = rospy.Publisher(self.exec_status_topic, ExecStatus)
        self.exec_status_publisher_thread = LoopingThread(target=self.publish_exec_status,
                                                          rate=50)
        self.exec_status_publisher_thread.start()

    def publish_exec_status(self):
        header = Header()
        header.stamp = rospy.Time.now()
        self.exec_status_publisher.publish(ExecStatus(header, self.exec_status))

    def execute_joint_trajectory(self, trajectory):
        # TODO: Add handling for speed
        try:
            joint_names = trajectory.joint_names
            points = [point.positions for point in trajectory.points]
            if joint_names[0][0] == 'l':
                arm = self.larm
            # elif joint_names[0][0] == 'r':
            #     arm = self.rarm
            else:  # TODO: Add handling for torso joint
                return

            if len(joint_names) == 1:
                if points[0][0] == 0:
                    arm.close_gripper()
                else:
                    arm.open_gripper()
            elif len(joint_names) > 1:
                self.exec_status = ExecStatus.BUSY
                arm.execute_joint_trajectory(points)
                self.exec_status = ExecStatus.IDLE
        except rospy.ROSException:
            self.exec_status = ExecStatus.ERROR

"""
if __name__ == '__main__':
    executor = APCTrajectoryExecutor('joint_trajectories', 'exec_status')
    print "ready for execution"
    executor.spin()
"""
