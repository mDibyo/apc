#!/usr/bin/env python
from __future__ import division

import roslib
roslib.load_manifest('apc')

import tf
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from apc.msg import Grasp

import os.path as osp
import json
from argparse import ArgumentParser

from utils import getch
from ros_utils import ROSNode, TopicSubscriberNode, topic
from rviz_marker_publisher import RvizMarkerPublisher
from message_wrappers import GraspWrapper


__author__ = 'dibyo'


APC_DIRECTORY = osp.abspath(osp.join(__file__, "../.."))
DATA_DIRECTORY = osp.join(APC_DIRECTORY, "data")


class RvizGraspHandler(ROSNode):
    QUIT_MOVE = 'q'

    def __init__(self, handler_name, object_name, object_marker, gripper_marker,
                 object_moves_topic, gripper_moves_topic, keep_old_grasps=False):
        super(RvizGraspHandler, self).__init__(handler_name,
                                               anonymous=False)
        self.object_name = object_name
        self.keep_old_grasps = keep_old_grasps
        self.grasps = []
        self.grasps_file = osp.join(DATA_DIRECTORY, 'grasps',
                                    "{}.json".format(self.object_name))

        self.object_marker = object_marker
        self.object_moves_publisher = rospy.Publisher(object_moves_topic, String)
        self.gripper_marker = gripper_marker
        self.gripper_moves_publisher = rospy.Publisher(gripper_moves_topic, String)

    def __enter__(self):
        if self.keep_old_grasps and osp.exists(self.grasps_file):
            self.grasps = GraspWrapper.grasps_from_file(self.grasps_file)

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.grasps:
            saved = False
            while not saved:
                try:
                    GraspWrapper.grasps_to_file(self.grasps, self.grasps_file)
                    saved = True
                except Exception as e:
                    rospy.logerr("Grasps not saved because of {}".format(e))
                    rospy.logerr("Press ENTER to retry")
                    saved = raw_input()

        self.object_moves_publisher.publish(RvizMarkerPublisher.QUIT_MOVE)
        self.gripper_moves_publisher.publish(RvizMarkerPublisher.QUIT_MOVE)


class RvizGraspViewer(RvizGraspHandler):
    def _handle_quit_move(self):
        """ Quit the viewer.  """
        pass

    def _handle_prev_grasp_move(self):
        """ Display the previous grasp.  """
        self.current_grasp_index = (self.current_grasp_index-1) % len(self.grasps)

    def _handle_next_grasp_move(self):
        """ Display the next grasp.  """
        self.current_grasp_index = (self.current_grasp_index+1) % len(self.grasps)

    def _handle_flag_grasp_move(self):
        """ Toggle the flag boolean for the current grasp.  """
        if self.modify_grasps:
            self.current_grasp.flag = not self.current_grasp.flag

    def _handle_remove_grasp_move(self):
        """ Remove the current grasp.  """
        if self.modify_grasps:
            self.grasps.pop(self.current_grasp_index)
            self.current_grasp_index %= len(self.grasps)

    def create_move_handlers_dict(self):
        self.move_handlers_dict = {
            'q': self._handle_quit_move,
            'a': self._handle_prev_grasp_move,
            'd': self._handle_next_grasp_move,
            'f': self._handle_flag_grasp_move,
            'r': self._handle_remove_grasp_move,
        }

    def __init__(self, object_name, object_marker, gripper_marker,
                 object_moves_topic, object_poses_topic,
                 gripper_moves_topic, gripper_grasps_topic, modify_grasps=False):
        super(RvizGraspViewer, self).__init__(handler_name='rviz_grasp_viewer',
                                              object_name=object_name,
                                              object_marker=object_marker,
                                              gripper_marker=gripper_marker,
                                              object_moves_topic=object_moves_topic,
                                              gripper_moves_topic=gripper_moves_topic,
                                              keep_old_grasps=True)
        self.modify_grasps = modify_grasps

        self.object_poses_publisher = rospy.Publisher(object_poses_topic, Pose)
        self.gripper_grasps_publisher = rospy.Publisher(gripper_grasps_topic, Grasp)

        self.move_handlers_dict = None
        self.create_move_handlers_dict()

        # Set state
        self.current_grasp_index = None

    def __enter__(self):
        super(RvizGraspViewer, self).__enter__()
        if not self.grasps:
            rospy.logfatal("No grasps available for {}".format(self.object_name))

        rospy.sleep(1)
        return self

    @property
    def current_grasp(self):
        return self.grasps[self.current_grasp_index]

    def update_grasp(self):
        self.object_poses_publisher.publish(Pose(Point(0.0, 0.0, 0.0),
                                                 Quaternion(0.0, 0.0, 0.0, 1.0)))
        self.gripper_grasps_publisher.publish(self.current_grasp.to_msg())

    def enter_control_loop(self):
        if not self.grasps:
            return

        self.current_grasp_index = 0
        self.update_grasp()

        ch = getch()
        while ch != self.QUIT_MOVE:
            self.execute_move(ch)
            ch = getch()

    def execute_move(self, move):
        self.move_handlers_dict[move]()
        self.update_grasp()


class RvizGraspSaver(RvizGraspHandler):
    GRASP_SAVE_MOVE = ' '

    def __init__(self, object_name, object_marker, gripper_marker,
                 object_moves_topic, gripper_moves_topic, gripper_width_topic,
                 update_rate, keep_old_grasps=False):
        super(RvizGraspSaver, self).__init__(handler_name='rviz_grasp_saver',
                                             object_name=object_name,
                                             object_marker=object_marker,
                                             gripper_marker=gripper_marker,
                                             object_moves_topic=object_moves_topic,
                                             gripper_moves_topic=gripper_moves_topic,
                                             keep_old_grasps=keep_old_grasps)

        self.update_rate = update_rate

        self.gripper_width_subscriber = TopicSubscriberNode(None, gripper_width_topic,
                                                            Float64, anonymous=True,
                                                            init=False)
        self.gripper_width_subscriber.add_callback()

        self.tf_listener = tf.TransformListener()

    def enter_control_loop(self):
        ch = getch()

        while ch != self.QUIT_MOVE:
            if ch == self.GRASP_SAVE_MOVE:
                try:
                    self.tf_listener.waitForTransform(self.object_marker,
                                                      self.gripper_marker,
                                                      rospy.Time(0),
                                                      rospy.Duration(2/self.update_rate))
                    trans, rot = self.tf_listener.lookupTransform(self.object_marker,
                                                                  self.gripper_marker,
                                                                  rospy.Time(0))

                    pose = Pose()
                    pose.position = Point(*trans)
                    pose.orientation = Quaternion(*rot)
                    gripper_width = self.gripper_width_subscriber.last_msg.data
                    self.grasps.append(GraspWrapper(pose, gripper_width))
                    rospy.logwarn('saving grasp: SUCCESS!')
                except tf.Exception:
                    rospy.logerr('saving grasp: FAILURE!')
            else:
                self.gripper_moves_publisher.publish(ch)
            ch = getch()


if __name__ == "__main__":
    description = 'Annotate objects with grasps. To be used with rviz and rviz_marker'
    parser = ArgumentParser(description=description)
    parser.add_argument('--mode', default='0', type=int, choices={0, 1},
                        help='0: View/Delete grasps, 1: Edit/Add grasps')
    parser.add_argument('--object-name')
    parser.add_argument('--object-marker')
    parser.add_argument('--gripper-marker')
    parser.add_argument('--object-moves-topic', default=None, type=topic)
    parser.add_argument('--object-poses-topic', default=None, type=topic)
    parser.add_argument('--gripper-moves-topic', default=None, type=topic)
    parser.add_argument('--gripper-grasps-topic', default=None, type=topic)
    parser.add_argument('--gripper-width-topic', default=None, type=topic)
    parser.add_argument('--update-rate', type=int)
    parser.add_argument('-k', '--keep-old-grasps', action='store_true')
    parser.add_argument('-m', '--modify-grasps', action='store_true')
    args = parser.parse_known_args()[0]

    if args.mode == 0:
        with RvizGraspViewer(object_name=args.object_name,
                             object_marker=args.object_marker,
                             gripper_marker=args.gripper_marker,
                             object_moves_topic=args.object_moves_topic,
                             object_poses_topic=args.object_poses_topic,
                             gripper_moves_topic=args.gripper_moves_topic,
                             gripper_grasps_topic=args.gripper_grasps_topic,
                             modify_grasps=args.modify_grasps) as viewer:
            viewer.enter_control_loop()

    else:
        with RvizGraspSaver(object_name=args.object_name,
                            object_marker=args.object_marker,
                            gripper_marker=args.gripper_marker,
                            object_moves_topic=args.object_moves_topic,
                            gripper_moves_topic=args.gripper_moves_topic,
                            gripper_width_topic=args.gripper_width_topic,
                            update_rate=args.update_rate,
                            keep_old_grasps=args.keep_old_grasps) as saver:
            saver.enter_control_loop()