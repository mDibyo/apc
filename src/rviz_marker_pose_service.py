#!/usr/bin/env python

from __future__ import division

import roslib
roslib.load_manifest('apc')
from visualization_msgs.msg import Marker

from apc.srv import *
import rospy
from ros_utils import ROSNode


__author__ = 'dibyo'


class RvizMarkerPoseService(ROSNode):
    def __init__(self, marker_poses_topic):
        super(RvizMarkerPoseService, self).__init__("rviz_marker_pose_service",
                                                    anonymous=True)

        self.markers = {}
        self.marker_poses_topic = marker_poses_topic

        self.markers_pose_subscriber = rospy.Subscriber(self.marker_poses_topic,
                                                        Marker, self.update_markers)
        self.get_marker_pose_server = rospy.Service('get_marker_pose', GetMarkerPose,
                                                    self.handle_get_marker_pose)

    def update_markers(self, marker):
        if marker.action == Marker.ADD:
            self.markers[marker.text] = marker.pose
        elif marker.action == Marker.DELETE:
            if marker.text in self.markers:
                del self.markers[marker.text]

    def handle_get_marker_pose(self, req):
        rospy.logwarn(self.markers)
        return GetMarkerPoseResponse(self.markers.get(req.marker_name, None))


if __name__ == '__main__':
    import sys
    rviz_marker_pose_service = RvizMarkerPoseService(sys.argv[1])
    rviz_marker_pose_service.spin()
