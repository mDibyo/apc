#!/usr/bin/env python
from __future__ import division

import roslib
roslib.load_manifest('apc')

import tf
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose, Point, Quaternion

import os.path as osp
import json
from argparse import ArgumentParser

from utils import getch
from ros_utils import ROSNode, TopicSubscriberNode
from rviz_marker_publisher import RvizMarkerPublisher


__author__ = 'dibyo'


APC_DIRECTORY = osp.abspath(osp.join(__file__, "../.."))
DATA_DIRECTORY = osp.join(APC_DIRECTORY, "data")


class Grasp(object):
    def __init__(self, pose, gripper_width=None):
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


class RvizGraspAnnotator(ROSNode):
    QUIT_MOVE = 'q'
    GRASP_SAVE_MOVE = ' '

    def __init__(self, object_name, object_marker, gripper_marker,
                 object_moves_topic, gripper_moves_topic, gripper_width_topic,
                 update_rate, keep_old_grasps=False):
        super(RvizGraspAnnotator, self).__init__('rviz_grasp_annotator',
                                                 anonymous=False)
        self.object_name = object_name
        self.update_rate = update_rate
        self.keep_old_grasps = keep_old_grasps
        self.grasps = []

        self.object_marker = object_marker
        self.object_moves_publisher = rospy.Publisher(object_moves_topic,
                                                      String)
        self.gripper_marker = gripper_marker
        self.gripper_moves_publisher = rospy.Publisher(gripper_moves_topic,
                                                       String)
        self.gripper_width_subscriber = TopicSubscriberNode(None, gripper_width_topic,
                                                            Float64, anonymous=True,
                                                            init=False)
        self.gripper_width_subscriber.add_callback()

        self.tf_listener = tf.TransformListener()

    def __enter__(self):
        grasps_file = osp.join(DATA_DIRECTORY, 'grasps',
                               "{}.json".format(self.object_name))

        if self.keep_old_grasps and osp.exists(grasps_file):
            with open(grasps_file, 'r') as f:
                self.grasps = \
                    [Grasp.from_json(grasp_json) for grasp_json in json.load(f)]

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        grasps_file = osp.join(DATA_DIRECTORY, 'grasps',
                               "{}.json".format(self.object_name))
        print grasps_file
        with open(grasps_file, 'w') as f:
            json.dump([grasp.to_json() for grasp in self.grasps], f,
                      indent=4, separators=(', ', ': '))

        self.object_moves_publisher.publish(RvizMarkerPublisher.QUIT_MOVE)
        self.gripper_moves_publisher.publish(RvizMarkerPublisher.QUIT_MOVE)

    def enter_control_loop(self):
        ch = getch()

        while ch != self.QUIT_MOVE:
            if ch == self.GRASP_SAVE_MOVE:
                try:
                    self.tf_listener.waitForTransform(self.gripper_marker,
                                                      self.object_marker,
                                                      rospy.Time(0),
                                                      rospy.Duration(2/self.update_rate))
                    trans, rot = self.tf_listener.lookupTransform(self.gripper_marker,
                                                                  self.object_marker,
                                                                  rospy.Time(0))

                    pose = Pose()
                    pose.position = Point(*trans)
                    pose.orientation = Quaternion(*rot)
                    gripper_width = self.gripper_width_subscriber.last_msg.data
                    self.grasps.append(Grasp(pose, gripper_width))
                    rospy.logwarn('saving grasp: SUCCESS!')
                except tf.Exception:
                    rospy.logerr('saving grasp: FAILURE!')
            else:
                self.gripper_moves_publisher.publish(ch)
            ch = getch()


if __name__ == "__main__":
    description = 'Annotate objects with grasps. To be used with rviz and rviz_marker'
    parser = ArgumentParser(description=description)
    parser.add_argument('--object-name')
    parser.add_argument('--object-marker')
    parser.add_argument('--gripper-marker')
    parser.add_argument('--object-moves-topic')
    parser.add_argument('--gripper-moves-topic')
    parser.add_argument('--gripper-width-topic')
    parser.add_argument('--update-rate', type=int)
    parser.add_argument('--keep-old-grasps', '-k', action='store_true')
    args = parser.parse_known_args()[0]

    with RvizGraspAnnotator(object_name=args.object_name,
                            object_marker=args.object_marker,
                            gripper_marker=args.gripper_marker,
                            object_moves_topic=args.object_moves_topic,
                            gripper_moves_topic=args.gripper_moves_topic,
                            gripper_width_topic=args.gripper_width_topic,
                            update_rate=args.update_rate,
                            keep_old_grasps=args.keep_old_grasps) as annotator:
        annotator.enter_control_loop()