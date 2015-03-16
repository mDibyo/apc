#!/usr/bin/env python

from __future__ import division

import os.path as osp
from copy import deepcopy
import json

import openravepy as rave
import numpy as np
import trajoptpy

import roslib
roslib.load_manifest('apc')
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from std_msgs.msg import Header

from apc.msg import BinWorkOrder
from apc.srv import *
from ros_utils import ROSNode
from rviz_grasp_handlers import Grasp


__author__ = 'dibyo'


APC_DIRECTORY = osp.abspath(osp.join(__file__, "../.."))
DATA_DIRECTORY = osp.join(APC_DIRECTORY, "data")


class APCPlanner(ROSNode):
    PREGRASP_POSE_DISTANCE = 0.05

    def __init__(self, work_orders_topic, update_rate,
                 manipulator_name='leftarm'):
        super(APCPlanner, self).__init__('plan')

        self.work_orders_topic = work_orders_topic
        self.update_rate = update_rate
        self.manipulator_name = manipulator_name

        self.object_poses = {}
        self.cubbyhole_pose = None
        self.work_order = None
        self.target_object_grasps = []



        self.work_orders_subscriber = rospy.Subscriber(self.work_orders_topic,
                                                       BinWorkOrder,
                                                       self.execute_work_order)
        self.tf_listener = tf.TransformListener()

        rospy.wait_for_service('get_marker_pose')
        self.get_marker_pose_client = rospy.ServiceProxy('get_marker_pose',
                                                         GetMarkerPose)

    def __enter__(self):
        self.env = rave.Environment()
        self.env.Load("robots/pr2-beta-static.zae")
        self.robot = self.env.GetRobot('pr2')
        self.manipulator = self.robot.SetActiveManipulator(self.manipulator_name)
        self.link = self.robot.GetLink('{}_gripper_tool_frame'.
                                       format(self.manipulator_name[0]))
        self.ikmodel = \
            rave.databases.inversekinematics. \
                InverseKinematicsModel(self.robot,
                                       iktype=rave.IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        rave.RaveDestroy()

    _trajopt_request_template = {
        "basic_info": {
            "n_steps": 10,
            "start_fixed": True
        },
        "costs": [{
              "type": "joint_vel",
              "params": {"coeffs": [1]}
        }, {
            "type": "collision",
            "params": {
                "coeffs": [20],
                "dist_pen": [0.005]
            }
        }],
        "constraints": [{
            "type": "joint",
            "params": {"vals": None }
        }],
        "init_info": {
            "type": "straight_line",
            "endpoint": None
        }
    }

    @property
    def trajopt_request_template(self):
        return deepcopy(self._trajopt_request_template)

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
        offset = T[:3, :3].dot([-APCPlanner.PREGRASP_POSE_DISTANCE, 0, 0])

        return Pose(Point(*(trans + offset)), Quaternion(*rot))

    def get_robot_joints(self, gripper_pose):
        T = np.linalg.solve(self.link.GetTransform(),
                            self.manipulator.GetEndEffectorTransform())
        with self.robot:
            ikparam = rave.IkParameterization(self.pose_to_transform_matrix(gripper_pose).dot(T),
                                              self.ikmodel.iktype)
            return self.manipulator.FindIKSolutions(ikparam,
                                                    rave.IkFilterOptions.CheckEnvCollisions)

    def get_robot_trajectory(self, target_joints, start_joints=None):
        if start_joints is None:
            start_joints = self.robot.GetDOFValues(self.manipulator.GetArmIndices())

        request = self.trajopt_request_template
        request['constraints'][0]['params']['vals'] = \
            request['init_info']['endpoint'] = list(target_joints)
        request['basic_info']['manip'] = self.manipulator.GetName()

        with self.robot:
            self.robot.SetDOFValues(start_joints, self.manipulator.GetArmIndices())
            problem = trajoptpy.ConstructProblem(json.dumps(request),
                                                 self.env)
            result = trajoptpy.OptimizeProblem(problem)
            return result.GetTraj().tolist()

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
                    res = self.get_marker_pose_client("object_{}".format(object_name))
                    self.object_poses[object_name] = res.marker_pose
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))

    def update_target_object_grasps(self):
        self.target_object_grasps = \
            Grasp.grasps_from_file(osp.join(
                DATA_DIRECTORY, 'grasps',
                '{}.json'.format(self.work_order.target_object)))

        object_frame = "/object_{}".format(self.work_order.target_object)
        try:
            self.tf_listener.waitForTransform(object_frame,
                                              "/base_link",
                                              rospy.Time(0),
                                              rospy.Duration(2/self.update_rate))
            header = Header(0, rospy.Time(0), object_frame)
            for grasp in self.target_object_grasps:
                grasp.pose = \
                    self.tf_listener.transformPose("/base_link",
                                                   PoseStamped(header, grasp.pose)).pose

        except tf.Exception, e:
            rospy.logerr(e)

    def update_simulation_environment(self):
        # Remove old objects
        for body in self.env.GetBodies():
            if not body.IsRobot():
                self.env.Remove(body)

        # Insert new cubbyhole
        cubbyhole_model_name = 'cubbyhole_{}'.format(self.work_order.bin_type)
        self.env.Load(osp.join(DATA_DIRECTORY, 'models',
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
        target_pregrasp_poses = []
        for grasp in self.target_object_grasps:
            target_pregrasp_poses.append(self.get_grasp_prepose(grasp.pose))

        # Filter out impossible pregrasp poses
        pregrasp_joint_angles = []
        for pregrasp_pose in target_pregrasp_poses:
            solutions = self.get_robot_joints(pregrasp_pose)
            if solutions:
                pregrasp_joint_angles.append(solutions)

        pregrasp_trajectories = [self.get_robot_trajectory(solution[0])
                                 for solution in pregrasp_joint_angles]

if __name__ == '__main__':
    with APCPlanner("work_orders", 10, 'leftarm') as planner:
        planner.spin()
