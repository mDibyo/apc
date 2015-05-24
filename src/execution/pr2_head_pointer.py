#!/usr/bin/env python
__author__ = 'dibyo'

import subprocess

import roslib
roslib.load_manifest('apc')
import rospy

from ros_utils import ROSNode
from apc.srv import PointHead, PointHeadResponse


class HeadPointer(ROSNode):
    def __init__(self):
        super(HeadPointer, self).__init__('point_head_service',
                                          anonymous=False)
        self.last_head_direction  = None

        self.point_head_server = rospy.Service('point_head', PointHead,
                                               self.handle_point_head)
        rospy.logwarn('Ready to point head')

    def handle_point_head(self, req):
        if self.last_head_direction != req.head_direction:
            try:
                subprocess.check_call(['rosrun',
                                       'simple_head',
                                       'point_head',
                                       str(req.head_direction.x),
                                       str(req.head_direction.y),
                                       str(req.head_direction.z)])
                self.last_head_direction = req.head_direction
            except subprocess.CalledProcessError:
                return PointHeadResponse(False)

        return PointHeadResponse(True)

if __name__ == '__main__':
    head_pointer = HeadPointer()
    head_pointer.spin()


