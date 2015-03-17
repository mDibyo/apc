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

from apc.srv import GetMarkerPose, GetMotionPlan
from ros_utils import ROSNode
from message_wrappers import GraspWrapper


__author__ = 'dibyo'


APC_DIRECTORY = osp.abspath(osp.join(__file__, "../.."))
DATA_DIRECTORY = osp.join(APC_DIRECTORY, "data")


class Approach(object):
    PREGRASP_POSE_DISTANCE = 0.05

    def __init__(self, target_gripper_pose, target_gripper_width):
        self.grasp_pose = target_gripper_pose
        self.gripper_width = target_gripper_width

        self.pregrasp_pose = self.get_grasp_prepose(self.grasp_pose)
        self.pregrasp_joints = None
        self.pregrasp_trajectory = None

        self.grasp_joints = None
        self.grasp_trajectory = None

        self.postgrasp_pose = None
        self.postgrasp_joints = None
        self.postgrasp_trajectory = None

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
        offset = T[:3, :3].dot([-Approach.PREGRASP_POSE_DISTANCE, 0, 0])

        return Pose(Point(*(trans + offset)), Quaternion(*rot))


class APCPlannerService(ROSNode):
    def __init__(self, work_orders_topic, update_rate,
                 manipulator_name='leftarm'):
        super(APCPlannerService, self).__init__('plan')

        self.work_orders_topic = work_orders_topic
        self.update_rate = update_rate
        self.manipulator_name = manipulator_name

        self.object_poses = {}
        self.cubbyhole_pose = None
        self.work_order = None
        self.target_object_approaches = []

        self.tf_listener = tf.TransformListener()

        rospy.wait_for_service('get_marker_pose')
        self.get_marker_pose_client = rospy.ServiceProxy('get_marker_pose',
                                                         GetMarkerPose)
        self.get_motion_plan_service = rospy.Service('get_motion_plan',
                                                     GetMotionPlan,
                                                     self.handle_get_motion_plan)

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
        self.T_l2ee = np.linalg.solve(self.link.GetTransform(),
                                      self.manipulator.GetEndEffectorTransform())

        self.joint_names = [self.robot.GetJointFromDOFIndex(index).GetName()
                            for index in self.manipulator.GetArmIndices()]
        self.joint_names += [self.robot.GetJointFromDOFIndex(index).GetName()
                             for index in self.manipulator.GetGripperIndices()]
        self.gripper_limits = \
            self.robot.GetDOFLimits(self.manipulator.GetGripperIndices())

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

    def get_robot_joints(self, gripper_pose):
        T = np.linalg.solve(self.link.GetTransform(),
                            self.manipulator.GetEndEffectorTransform())
        with self.robot:
            ikparam = rave.IkParameterization(
                self.pose_to_transform_matrix(gripper_pose).dot(T),
                self.ikmodel.iktype)
            return self.manipulator.FindIKSolution(ikparam,
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

    def update_target_object_approaches(self):
        target_object_grasps = \
            GraspWrapper.grasps_from_file(osp.join(
                DATA_DIRECTORY, 'grasps',
                '{}.json'.format(self.work_order.target_object)))

        object_frame = "/object_{}".format(self.work_order.target_object)
        try:
            self.tf_listener.waitForTransform(object_frame,
                                              "/base_link",
                                              rospy.Time(0),
                                              rospy.Duration(2/self.update_rate))
            header = Header(0, rospy.Time(0), object_frame)
            for grasp in target_object_grasps:
                grasp.gripper_pose = \
                    self.tf_listener.transformPose("/base_link",
                                                   PoseStamped(header,
                                                               grasp.gripper_pose)).pose
        except tf.Exception, e:
            rospy.logerr(e)

        self.target_object_approaches = [Approach(grasp.gripper_pose,
                                                  grasp.gripper_width)
                                         for grasp in target_object_grasps]

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

    def update_state(self):
        self.update_cubbyhole_and_objects()
        self.update_target_object_approaches()
        self.update_simulation_environment()

    def _find_joints(self, joints_attr, pose_attr):
        for approach in self.target_object_approaches:
            setattr(approach, joints_attr,
                    self.get_robot_joints(getattr(approach, pose_attr)))

    def _find_trajectory(self, trajectory_attr, target_joints_attr,
                         start_joints_attr=''):
        for approach in self.target_object_approaches:
            setattr(approach, trajectory_attr,
                    self.get_robot_trajectory(
                        getattr(approach, target_joints_attr),
                        getattr(approach, start_joints_attr, None)
                    ))

    def _filter_approaches_by(self, filter_attr):
        self.target_object_approaches[:] = \
            [approach for approach in self.target_object_approaches
             if getattr(approach, filter_attr, None) is not None]

    # def find_pregrasp_joints(self):
    #     for approach in self.target_object_approaches:
    #         approach.pregrasp_joints = \
    #             self.get_robot_joints(approach.pregrasp_pose)
    #
    # def find_pregrasp_trajectory(self):
    #     for approach in self.target_object_approaches:
    #         approach.pregrasp_trajectory = \
    #             self.get_robot_trajectory(approach.pregrasp_joints)
    #
    # def find_grasp_joints(self):
    #     for approach in self.target_object_approaches:
    #         approach.grasp_joints = \
    #             self.get_robot_joints(approach.grasp_pose)
    #
    # def find_grasp_trajectory(self):
    #     for approach in self.target_object_approaches:
    #         approach.grasp_trajectory = \
    #             self.get_robot_trajectory(approach.grasp_joints,
    #                                       approach.pregrasp_joints)

    def find_pregrasp_joints(self):
        self._find_joints('pregrasp_joints', 'pregrasp_pose')

    def find_pregrasp_trajectory(self):
        self._find_trajectory('pregrasp_trajectory', 'pregrasp_joints')

    def find_grasp_joints(self):
        self._find_joints('grasp_joints', 'grasp_pose')

    def find_grasp_trajectory(self):
        self._find_trajectory('grasp_trajectory', 'grasp_joints',
                              'pregrasp_joints')


    def handle_get_motion_plan(self, work_order):
        self.work_order = work_order
        self.update_state()

        self._find_joints('pregrasp_joints', 'pregrasp_pose')
        self._filter_approaches_by('pregrasp_joints')

        self._find_trajectory('pregrasp_trajectory', 'pregrasp_joints')
        self._filter_approaches_by('pregrasp_trajectory')

        self.robot.SetDOFValues(self.gripper_limits[1],
                                self.manipulator.GetGripperIndices())

        self._find_joints('grasp_joints', 'grasp_pose')
        self._filter_approaches_by('grasp_joints')

        self._find_trajectory('grasp_trajectory', 'grasp_joints',
                              'pregrasp_joints')
        self._filter_approaches_by('grasp_trajectory')





# if __name__ == '__main__':
#     with APCPlannerService("work_orders", 10, 'leftarm') as planner:
#         planner.spin()
