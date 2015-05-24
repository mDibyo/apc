#!/usr/bin/env python

import roslib
roslib.load_manifest('apc')
import rospy

import utils
from ros_utils import ROSNode
from apc.srv import LookAtBins, LookAtBinsResponse
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from apc.msg import MotionPlan

__author__ = 'dibyo'

BASE_OFFSET = -1
TORSO_OFFSET = 1.78
HEAD_X_OFFSET = 1
HEAD_Z_OFFSET = 0.2


class BinLooker(ROSNode):
    def __init__(self):
        super(BinLooker, self).__init__('bins_looker_service', anonymous=True)

        self.look_at_bins_service = rospy.Service('look_at_bins', LookAtBins,
                                                  self.handle_look_at_bins)

    def handle_look_at_bins(self, req):
        bin_list = req.bin_list
        if not bin_list:
            bin_list = utils.BINS

        motion_plan_list = []
        for bin in bin_list:
            motion_plan = MotionPlan()

            pose = utils.bin_pose[bin]

            # Move base
            motion_plan.base_target = Point(BASE_OFFSET, pose[5], 0)

            # Move torso
            motion_plan.torso_height = Float32(TORSO_OFFSET + pose[6])

            # Point head
            motion_plan.head_direction = Point(Point(pose[4]+HEAD_X_OFFSET, pose[5],
                                                     pose[6]+HEAD_Z_OFFSET))

            # Capture image
            motion_plan.capture_scene = bin

            motion_plan_list.append(motion_plan)

        return LookAtBinsResponse(motion_plan_list)


if __name__ == '__main__':
    bin_looker = BinLooker()
    bin_looker.spin()