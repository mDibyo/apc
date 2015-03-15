#!/usr/bin/env python

from __future__ import division

import roslib
roslib.load_manifest('apc')

import rospy
from ros_utils import ROSNode

from apc.msg import BinWorkOrder

__author__ = 'dibyo'


class APCController(ROSNode):
    def __init__(self, work_orders_topic):
        super(APCController, self).__init__('apc_controller')

        self.work_orders_topic = work_orders_topic

        self.work_orders_publisher = rospy.Publisher(self.work_orders_topic,
                                                     BinWorkOrder)

    def execute_strategy(self, strategy):
        pass