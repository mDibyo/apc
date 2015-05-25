#!/usr/bin/env python

from __future__ import division

import roslib
roslib.load_manifest('apc')
from visualization_msgs.msg import Marker

from apc.srv import *
import rospy
from ros_utils import ROSNode
from geometry_msgs.msg import Pose, Point, Quaternion

import utils


from watchdog.observers import Observer
from watchdog.events import PatternMatchingEventHandler

import json
import os.path as osp
import numpy as np
import openravepy as rave

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


class APCObjectPoseService(PatternMatchingEventHandler):

    patterns = ['*.json']

    def __init__(self):
        super(APCObjectPoseService, self).__init__()
        rospy.init_node('apc_object_pose_service')

        self.bins = {}
        self.get_object_pose_service = rospy.Service('get_object_pose', GetObjectPose,
                                                     self.handle_get_object_pose)
                                                    
        rospy.logwarn(utils.OBJECT_POSES_DIR)
        
    def __enter__(self):
        self.observer = Observer()
        self.observer.schedule(self, path=utils.OBJECT_POSES_DIR)
        self.observer.start()
        rospy.loginfo("object pose service ready")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.observer.stop()
        self.observer.join()

    def on_modified(self, event):
        print event.event_type, event.src_path
        if not event.is_directory:
            with open(event.src_path, 'r') as f:
                bin = json.load(f)
                bin_name = osp.splitext(osp.basename(event.src_path))[0]
                self.bins[bin_name] = bin

    def handle_get_object_pose(self, req):
        bin_name = req.bin_name
        
        if bin_name in self.bins:
            mat = self.bins[bin_name].get(req.object, None)

            rospy.logwarn(mat)

            pose = rave.poseFromMatrix(mat)
            obj_pose = Pose()
            
            obj_pose.orientation = Quaternion(pose[0], pose[1], pose[2], pose[3])
            obj_pose.position = Point(pose[-3], pose[-2], pose[-1])
            
            return GetObjectPoseResponse(obj_pose)
        return GetObjectPoseResponse(None)


# if __name__ == '__main__':
#     import sys
#     rviz_marker_pose_service = RvizMarkerPoseService(sys.argv[1])
#     rviz_marker_pose_service.spin()

if __name__ == '__main__':
    with APCObjectPoseService() as object_pose_service:
        rospy.spin()
