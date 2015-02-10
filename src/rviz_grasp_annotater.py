#!/usr/bin/env python

import roslib
roslib.load_manifest('apc')

import rospy
import tf
from visualization_msgs.msg import Marker

from ros_utils import TopicSubscriberNode, ROSNode


__author__ = 'dibyo'


# class RvizGraspAnnotator(ROSNode):
#     def __init__(self, gripper_marker_id, object_marker_id):
#         super(RvizGraspAnnotator, self).__init__('rviz_grasp_annotator')
#
#
#         gripper_marker_listener = TopicSubscriberNode('gripper_marker_listener',
#                                                       'visualization_marker',
#                                                       Marker, anonymous=True)
#         gripper_marker_listener.set_msg_filter(lambda msg:
#                                                msg.id == gripper_marker_id)
#         gripper_marker_listener.add_callback()
#
#         object_marker_listener = TopicSubscriberNode('object_marker_listener',
#                                                      'visualization_marker',
#                                                      Marker, anonymous=True)
#         object_marker_listener.set_msg_filter(lambda msg:
#                                               msg.id == object_marker_id)
#         object_marker_listener.add_callback()
#
#
#
#     def get_gripper_transform(self, wait=False):
#         """
#         Get gripper transform with respect to object
#
#         :param wait: whether this method should wait till transforms are available
#         """
class RvizGraspAnnotator(ROSNode):
    def __init__(self, name, gripper_marker, object_marker):
        super(RvizGraspAnnotator, self).__init__(name)

        self.gripper_marker = gripper_marker
        self.object_marker = object_marker

        self.listener = tf.TransformListener()

        print self.get_gripper_transform()

    def get_gripper_transform(self):
        trans, rot = self.listener.lookupTransform(self.gripper_marker,
                                                   self.object_marker,
                                                   rospy.Time(0))
        print trans, rot

if __name__ == "__main__":
    # TODO: Get this info from command line
    gripper_marker = ''
    object_marker = ''

    node = RvizGraspAnnotator('grasp_annotator', gripper_marker, object_marker)
    node.spin()



