#!/usr/bin/env python
__author__ = 'dibyo'

import roslib
roslib.load_manifest('stereo_click')
roslib.load_manifest('tfx')

import rospy
import tfx
from geometry_msgs.msg import PointStamped

from pr2 import arm as _arm


class StereoPointsFollower(object):
    def __init__(self, arm='right'):
        rospy.init_node('stereo_points_follower', anonymous=True)

        self.arm = _arm.Arm('right')
        self.listener = tfx.TransformListener()
        self.frame = 'base_link'

        rospy.Subscriber('stereo_points_3d', PointStamped, self.callback)

    def callback(self, point):
        """
        :param point: The point from stereo_click
        :type point: PointStamped
        :rtype: None
        """
        if point.header.frame_id != self.frame:
            self.listener.waitForTransform(self.frame,
                                           point.header.frame_id,
                                           rospy.Time.now(),
                                           rospy.Duration(1.0))
        point = self.listener.transformPoint(self.frame, point)

        target_pose = tfx.pose([point.point.x, point.point.y, point.point.z],
                               tfx.tb_angles(0, 0, 0),
                               frame=point.header.frame_id)
        self.arm.go_to_pose(target_pose, block=True)


if __name__ == '__main__':
    import sys
    arm = sys.argv[1] if len(sys.argv) > 1 else 'right'

    sp_follower = StereoPointsFollower(arm)

    # Keep python from exiting until node is stopped
    rospy.spin()