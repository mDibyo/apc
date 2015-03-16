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
    def __init__(self, markers_pose_topic):
        super(RvizMarkerPoseService, self).__init__("rviz_marker_pose_service",
                                                    anonymous=True)

        self.markers = {}
        self.markers_pose_topic = markers_pose_topic

        self.markers_pose_subscriber = rospy.Subscriber(self.markers_pose_topic,
                                                        Marker, self.update_markers)
        self.get_marker_pose_server = rospy.Service('get_marker_pose', GetMarkerPose,
                                                    self.handle_get_marker_pose)

        self.spin()

    def update_markers(self, marker):
        if marker.action == Marker.ADD:
            self.markers[marker.text] = marker.pose
        elif marker.action == Marker.DELETE:
            if marker.text in self.markers:
                del self.markers[marker.text]

    def handle_get_marker_pose(self, req):
        return GetMarkerPoseResponse(self.markers.get(req.marker_name, None))


if __name__ == '__main__':
    import sys
    RvizMarkerPoseService(sys.argv[1])
