#!/usr/bin/env python

from __future__ import division

import roslib
roslib.load_manifest('apc')

import rospy
from ros_utils import ROSNode

from apc.msg import BinWorkOrder
from apc.srv import GetMotionPlan

__author__ = 'dibyo'


class APCController(ROSNode):
    def __init__(self):
        super(APCController, self).__init__('apc_controller')

        self.get_motion_plan_client = rospy.ServiceProxy('get_motion_plan',
                                                         GetMotionPlan)

    def execute_strategy(self, strategy):
        pass