#!/usr/bin/env python

from __future__ import division

import os.path as osp

import roslib
roslib.load_manifest('apc')
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion

from apc.msg import BinWorkOrder
from apc.srv import *
from ros_utils import ROSNode
from rviz_grasp_handlers import Grasp

import openravepy as rave
import numpy as np


__author__ = 'dibyo'


APC_DIRECTORY = osp.abspath(osp.join(__file__, "../.."))
DATA_DIRECTORY = osp.join(APC_DIRECTORY, "data")


class APCPlanner(ROSNode):
    def __init__(self, work_orders_topic):
        super(APCPlanner, self).__init__('APCPlanner')

        self.work_orders_topic = work_orders_topic

        self.object_poses = {}
        self.cubbyhole_pose = None
        self.work_order = None
        self.target_object_grasps = []



        self.work_orders_subscriber = rospy.Subscriber(self.work_orders_topic,
                                                       BinWorkOrder,
                                                       self.execute_work_order)

        rospy.wait_for_service('get_marker_pose')
        self.get_marker_pose_client = rospy.ServiceProxy('get_marker_pose',
                                                         GetMarkerPose)

    def __enter__(self):
        self.env = rave.Environment()
        self.env.Load("robots/pr2-beta-static.zae")
        self.robot = self.env.GetRobot('pr2')
        self.manip = self.robot.SetActiveManipulator('leftarm_torso')
        self.ikmodel = \
            rave.databases.inversekinematics. \
                InverseKinematicsModel(self.robot,
                                       iktype=rave.IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        rave.RaveDestroy()

    @staticmethod
    def pose_to_transform_matrix(pose):
        """
        Convert a pose to a transform matrix
        """
        rot = [pose.orientation.x,
               pose.orientation.y,
               pose.orientation.z,
               pose.orientation.w]
        trans = [pose.position.x, pose.position.y, pose.position.z]
        matrix = np.array(tf.transformations.quaternion_matrix(rot))
        matrix[0:3, 3] = trans
        return matrix

    @staticmethod
    def get_grasp_prepose(grasp_pose):
        trans = np.array([grasp_pose.position.x,
                          grasp_pose.position.y,
                          grasp_pose.position.z])
        rot = np.array([grasp_pose.orientation.x,
                        grasp_pose.orientation.y,
                        grasp_pose.orientation.z,
                        grasp_pose.orientation.w])

        T = tf.transformations.quaternion_matrix(rot)
        offset = T.dot([0.075, 0, 0])

        return Pose(Point(*(trans + offset)), Quaternion(*rot))

    def set_kinbody_transform(self, body_name, pose):
        body = self.env.GetKinBody(body_name)
        body.SetTransform(self.pose_to_transform_matrix(pose))

    def update_cubbyhole_and_objects(self):
        self.object_poses = {}
        self.cubbyhole_pose = None

        if self.work_order:
            try:
                res = self.get_marker_pose_client('cubbyhole_{}'.
                                                  format(self.work_order.bin_type))
                self.cubbyhole_pose = res.marker_pose

                for object_name in self.work_order.bin_contents:
                    res = self.get_marker_pose_client(object_name)
                    self.object_poses[object_name] = res.marker_pose
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))

    def update_target_object_grasps(self):
        self.target_object_grasps = \
            Grasp.grasps_from_file(osp.join(
                DATA_DIRECTORY, 'grasps',
                '{}.json'.format(self.work_order.target_object)))

    def update_simulation_environment(self):
        # Remove old objects
        for body in self.env.GetBodies():
            if not body.IsRobot():
                self.env.Remove(body)

        # Insert new cubbyhole
        cubbyhole_model_name = 'cubbyhole_{}'.format(self.work_order.bin_type)
        self.env.Load(osp.join(DATA_DIRECTORY, 'models', 'cubbyholes',
                                   '{}.kinbody.xml'.format(cubbyhole_model_name)))
        self.set_kinbody_transform(cubbyhole_model_name, self.cubbyhole_pose)

        # Insert new objects
        for object_name in self.work_order.bin_contents:
            self.env.Load(osp.join(DATA_DIRECTORY, 'meshes', 'objects',
                                       '{}.stl'.format(object_name)))
            self.set_kinbody_transform(object_name,
                                       self.object_poses[object_name])

    def execute_work_order(self, work_order):
        self.work_order = work_order

        self.update_cubbyhole_and_objects()
        self.update_target_object_grasps()
        self.update_simulation_environment()

        # Get grasp preposes
        target_preposes = []
        for grasp in self.target_object_grasps:
            target_preposes.append(self.get_grasp_prepose(grasp.pose))

        # Filter out impossible poses



if __name__ == '__main__':
    with APCPlanner("work_orders") as planner:
        pass

# marker_pose:
# position:
# x: 0.03
# y: 0.01
# z: 0.42
# orientation:
# x: -0.247139573138
# y: -0.304817115648
# z: -0.261567431794
# w: 0.881811224709

