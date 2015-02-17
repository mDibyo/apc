#!/usr/bin/env python

import roslib
roslib.load_manifest('apc')

import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion

import os.path as osp
import json

from ros_utils import TopicSubscriberNode, ROSNode
from rviz_marker_publisher import RvizMarkerPublisher



__author__ = 'dibyo'

DATA_DIRECTORY = osp.abspath("../data")


class Grasp(object):
    def __init__(self, pose, gripper_width):
        self.pose = pose
        self.gripper_width = gripper_width

    def to_json(self):
        p = self.pose.position
        o = self.pose.orientation

        d = {
            'pose': {
                'position': {'x': p.x, 'y': p.y, 'z': p.z},
                'orientation': {'x': o.x, 'y': o.y, 'z': o.z, 'w': o.w}
            },
            'gripper_width': self.gripper_width
        }
        return d

    @classmethod
    def from_json(cls, d):
        pose = Pose()
        pose.position = Point(**d['pose']['position'])
        pose.orientation = Quaternion(**d['pose']['orientation'])

        return cls(pose, d['gripper_width'])


class RvizMarkerPublisherAugmented(RvizMarkerPublisher):

    def __init__(self, *args, **kwargs):
        super(RvizMarkerPublisherAugmented, self).__init__(*args, **kwargs)
        self.grasps = []

    def update_pose(self, move):
        if move == ' ':
            self.grasps += Grasp(self.pose, self.gripper_width)

        super(RvizMarkerPublisherAugmented, self).update_pose(move)


class RvizGraspAnnotator(ROSNode):
    def __init__(self, object_name):
        super(RvizGraspAnnotator, self).__init__('rviz_grasp_annotator',
                                                 anonymous=False)
        self.object_name = object_name

        self.listener = tf.TransformListener()

        mesh_file = osp.join(DATA_DIRECTORY, 'meshes', self.object_name)
        self.object_marker = RvizMarkerPublisher(marker_type=1,
                                                 name='object_marker',
                                                 pose=Pose(),
                                                 mesh_file=mesh_file,
                                                 remove=True)

        self.gripper_marker = RvizMarkerPublisherAugmented(marker_type=2,
                                                           name='gripper_marker',
                                                           pose=Pose(),
                                                           remove=True)

    def __enter__(self):
        grasps_file = osp.join(DATA_DIRECTORY, 'grasps', self.object_name)

        if osp.exists(grasps_file):
            with open(grasps_file, 'r') as f:
                self.gripper_marker.grasps = \
                    [Grasp.from_json(grasp_json) for grasp_json in json.load(f)]

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        grasps_file = osp.join(DATA_DIRECTORY, 'grasps', self.object_name)
        with open(grasps_file, 'w') as f:
            json.dump([grasp.to_json() for grasp in self.gripper_marker.grasps], f)

    def enter_control_loop(self):
        pass







