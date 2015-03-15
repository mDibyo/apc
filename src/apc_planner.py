#!/usr/bin/env python

from __future__ import division

import os.path as osp

import roslib
roslib.load_manifest('apc')
import rospy

from apc.msg import BinWorkOrder
from apc.srv import *
from ros_utils import ROSNode
from rviz_grasp_handlers import Grasp

import openravepy as rave
import numpy as np


__author__ = 'dibyo'


APC_DIRECTORY = osp.abspath(osp.join(__file__, "../.."))
DATA_DIRECTORY = osp.join(APC_DIRECTORY, "data")


class APCPlanner(ROSNode):
    def __init__(self, work_orders_topic):
        super(APCPlanner, self).__init__('APCPlanner')

        self.work_orders_topic = work_orders_topic

        self.object_poses = {}
        self.cubbyhole_pose = None
        self.work_order = None
        self.target_object_grasps = []

        self.sim_env = rave.Environment()
        self.sim_env.Load("robots/pr2-beta-static.zae")

        self.work_orders_subscriber = rospy.Subscriber(self.work_orders_topic,
                                                       BinWorkOrder,
                                                       self.execute_work_order)

        rospy.wait_for_service('get_marker_pose')
        self.get_marker_pose_client = rospy.ServiceProxy('get_marker_pose',
                                                         GetMarkerPose)

    def update_cubbyhole_and_objects(self):
        self.object_poses = {}
        self.cubbyhole_pose = None

        if self.work_order:
            try:
                res = self.get_marker_pose_client('cubbyhole_{}'.
                                                  format(self.work_order.bin_type))
                self.cubbyhole_pose = res.marker_pose

                for item in self.work_order.bin_contents:
                    res = self.get_marker_pose_client(item)
                    self.object_poses[item] = res.marker_pose
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))

    def update_target_object_grasps(self):
        self.target_object_grasps = \
            Grasp.grasps_from_file(osp.join(DATA_DIRECTORY, 'grasps',
                                            '{}.json'.format(self.work_order.target_object)))]

    def update_simulation_environment(self):
        pass

    def execute_work_order(self, work_order):
        self.work_order = work_order
        self.update_cubbyhole_and_objects()
        self.update_target_object_grasps()



if __name__ == '__main__':
    planner = APCPlanner("work_orders")
