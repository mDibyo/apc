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

        self._robot_exec_status = None

        self.get_motion_plan_client = rospy.ServiceProxy('get_motion_plan',
                                                         GetMotionPlan)

        self.joint_trajectories_publisher = rospy.Publisher(self.joint_trajectories_topic,
                                                            JointTrajectory)
        self.exec_status_subscriber = rospy.Subscriber(self.exec_status_topic,
                                                       ExecStatus,
                                                       self.record_exec_status)

    @property
    def robot_exec_status(self):
        return self._robot_exec_status

    @robot_exec_status.setter
    def robot_exec_status(self, exec_status):
        if exec_status == ExecStatus.ERROR:
            rospy.logerr("Error in robot execution status")
        self._robot_exec_status = exec_status

    def record_exec_status(self, status):
        self.robot_exec_status = status

    def wait_for_busy_exec_status(self):
        while self.robot_exec_status != ExecStatus.BUSY:
            rospy.sleep(0.1)
        rospy.loginfo("robot execution status busy")

    def wait_for_idle_exec_status(self):
        while self.robot_exec_status != ExecStatus.IDLE:
            rospy.sleep(0.1)
        rospy.loginfo("robot execution status idle")

    def execute_motion_plan(self, motion_plan):
        for joint_trajectory in motion_plan.trajectories:
            self.joint_trajectories_publisher.publish(joint_trajectory)
            # self.wait_for_busy_exec_status()
            rospy.sleep(0.1)
            self.wait_for_idle_exec_status()

    def execute_work_order(self, work_order):
        try:
            res = self.get_motion_plan_client(work_order)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))


if __name__ == '__main__':
    controller = APCController('joint_trajectories', 'exec_status')
    work_order = BinWorkOrder('broad_tall', ['dove_beauty_bar'],
                              'dove_beauty_bar', 'simple')
    controller.execute_work_order(work_order)