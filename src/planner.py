#!/usr/bin/env python

from __future__ import division

import roslib
roslib.load_manifest('apc')
import rospy

from apc.msg import BinWorkOrder
from apc.srv import *

from ros_utils import ROSNode

__author__ = 'dibyo'


class APCPlanner(ROSNode):
    def __init__(self, work_orders_topic):
        super(APCPlanner, self).__init__('APCPlanner')

        self.work_orders_topic = work_orders_topic

        self.object_poses = {}
        self.cubbyhole_pose = None
        self.work_order = None

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

    def execute_work_order(self, work_order):
        self.work_order = work_order
        self.update_cubbyhole_and_objects()


if __name__ == '__main__':
    planner = APCPlanner("work_orders")

