#!/usr/bin/env python

from __future__ import division
from scipy.optimize import fmin_l_bfgs_b
import os.path as osp
from copy import deepcopy
import json

import openravepy as rave
import numpy as np
import trajoptpy
from trajoptpy.check_traj import traj_is_safe

import roslib
roslib.load_manifest('apc')
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from std_msgs.msg import Header

from apc.msg import MotionPlan
from apc.srv import GetMarkerPose, GetMotionPlan, GetMotionPlanResponse
from ros_utils import ROSNode
from message_wrappers import GraspWrapper, JointTrajectoryWrapper


__author__ = 'dibyo'


APC_DIRECTORY = osp.abspath(osp.join(__file__, "../.."))
DATA_DIRECTORY = osp.join(APC_DIRECTORY, "data")


class Approach(object):
    PREGRASP_POSE_OFFSET = -0.15
    GRASP_POSE_OFFSET = 0.045
    POSTGRASP_POSE_OFFSET = -0.2

    WAYPOINT_POSE = Pose(Point(0.35, 0.00739935381367, 1.2),
                         Quaternion(-0.00363820843569, -0.0145515141784,
                                    -0.242529037717, 0.970028186569))
    DROPZONE_POSE = Pose(Point(0.37946294703187866, 0.7371332656419127,
                               0.3694018168496691),
                         Quaternion(-0.5517710937068498, 0.3620950514477245,
                                    0.5284559388562138, 0.5340132531634529))

    def __init__(self, target_gripper_pose, target_gripper_width):
        self.gripper_width = target_gripper_width

        self.pregrasp_pose = self.get_offset_pose(target_gripper_pose,
                                                  self.PREGRASP_POSE_OFFSET)
        self.pregrasp_joints = None
        self.pregrasp_trajectory = None

        self.grasp_pose = self.get_offset_pose(target_gripper_pose,
                                               self.GRASP_POSE_OFFSET)
        self.grasp_joints = None
        self.grasp_trajectory = None

        p = self.grasp_pose.position
        q = self.grasp_pose.orientation

        self.postgrasp_pose = self.get_offset_pose(target_gripper_pose,
                                                   self.POSTGRASP_POSE_OFFSET)
        self.postgrasp_joints = None
        self.postgrasp_trajectory = None

        self.waypoint_pose = self.WAYPOINT_POSE
        self.waypoint_joints = None
        self.waypoint_trajectory = None

        self.dropzone_pose = self.DROPZONE_POSE
        self.dropzone_joints = None
        self.dropzone_trajectory = None

    @staticmethod
    def get_offset_pose(pose, offset):
        if not offset:
            return pose

        trans,rot = Approach.TR_from_pose(pose)
        
        T = tf.transformations.quaternion_matrix(rot)
        offset = T[:3, :3].dot([offset, 0, 0])

        return Pose(Point(*(trans + offset)), Quaternion(*rot))
        
    @staticmethod
    def TR_from_pose(pose):
        trans = np.array([pose.position.x,
                          pose.position.y,
                          pose.position.z])
        rot = np.array([pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w])
        return trans,rot


class APCPlannerService(ROSNode):
    def __init__(self, work_orders_topic, update_rate,
                 manipulator_name='leftarm_torso'):
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
        self.env.Load(osp.join(DATA_DIRECTORY, "models", "pr2-beta-static.zae"))
        self.robot = self.env.GetRobot('pr2')
        self.manipulator = self.robot.SetActiveManipulator(self.manipulator_name)
        self.link = self.robot.GetLink('{}_gripper_tool_frame'.
                                       format(self.manipulator_name[0]))

        self.robot.SetActiveDOFs(self.manipulator.GetArmIndices(), 
                    rave.DOFAffine.X + rave.DOFAffine.Y + rave.DOFAffine.RotationAxis, [0,0,1])
                             
        self.robot.SetDOFValues([-0.4, -0.52, 0, -0.4, 0, 0, 0],
                                self.robot.GetManipulator('rightarm').GetArmIndices())
       
        self.ikmodel = \
            rave.databases.inversekinematics. \
                InverseKinematicsModel(self.robot,
                                       iktype=rave.IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()
        self.T_l2ee = np.linalg.solve(self.link.GetTransform(),
                                      self.manipulator.GetEndEffectorTransform())

        self.arm_joint_names = [self.robot.GetJointFromDOFIndex(index).GetName()
                                for index in self.manipulator.GetArmIndices()]
        self.gripper_joint_names = [self.robot.GetJointFromDOFIndex(index).GetName()
                                    for index in self.manipulator.GetGripperIndices()]
        self.torso_joint_names = ['torso_lift_joint']
        self.gripper_limits = \
            self.robot.GetDOFLimits(self.manipulator.GetGripperIndices())

        self.env.SetViewer('qtcoin')
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        rave.RaveDestroy()

    _trajopt_request_template = {
        "basic_info": {
            "n_steps": 10,
            "start_fixed": False
        },
        "costs": [{
            "type": "joint_vel",
            "params": {"coeffs": [2]}
        }, {
            "type": "collision",
            "params": {
                "coeffs": [20],
                "dist_pen": [0.02]
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
            iksol = self.manipulator.FindIKSolution(ikparam,
                                                   rave.IkFilterOptions.CheckEnvCollisions)
            distance = 0
            
            if iksol is None:
                rospy.logwarn("trying numerical IK")
                iksol, distance = self.numIK(gripper_pose)
            
            return iksol, distance
            
    def numIK(self, targetPose, weights=[1,1,1,1,15,15,15], Cw = 100):
        """
        def gripPointDir(q):
            m = rave.matrixFromQuat(q)[:3,3]
            return m.dot(np.array([[1],[0],[0]]))
            
        def gripOpenDir(q):
            m = rave.matrixFromQuat(q)[:3,3]
            return m.dot(np.array([[0],[0],[1]]))
            
        def gripperState(pose):
            q = pose[:4]
            pointdir = gripPointDir(q)
            opendir = gripOpenDir(q)
            rospy.logwarn(pointdir.shape, opendir.shape)
            return np.vstack([pointdir,opendir])
        """
        rave.RaveSetDebugLevel(rave.DebugLevel.Error)
        robot = self.env.GetRobot('pr2')
        manip = self.robot.SetActiveManipulator(self.manipulator_name)
        joint_start = robot.GetDOFValues(manip.GetArmIndices())


        robot.SetDOFValues(joint_start, manip.GetArmIndices())
        trans,rot = Approach.TR_from_pose(targetPose)
        target = np.hstack([rot, trans])
        rospy.logwarn(target)
        bounds = []
        lowers,uppers = robot.GetDOFLimits()
        for j in manip.GetArmIndices():
            bounds.append( (lowers[j], uppers[j]) )
       
        def cost(armjoints):    
            robot.SetDOFValues(armjoints, manip.GetArmIndices())
            grippose = rave.poseFromMatrix(manip.GetTransform())
            distCost = np.linalg.norm( (grippose - target) * np.array(weights) )
            collCost = Cw * sum([sum([1 if self.env.CheckCollision(f,o) else 0 for o in self.env.GetBodies()[1:]]) for f in manip.GetChildLinks()])
            return distCost + collCost
            
        final, fmin, d = fmin_l_bfgs_b(cost, joint_start, maxfun=500, approx_grad=True,bounds=bounds,pgtol=1e-12,factr=1)
        return final, fmin

    def get_robot_trajectory(self, target_joints, start_joints=None, dist_pen=0.02):
        if start_joints is None:
            start_joints = self.robot.GetDOFValues(self.manipulator.GetArmIndices())

        request = self.trajopt_request_template
        request['constraints'][0]['params']['vals'] = \
            request['init_info']['endpoint'] = list(target_joints)
        request['basic_info']['manip'] = self.manipulator.GetName()
        request['costs'][1]['params']['dist_pen'] = [dist_pen]

        with self.robot:
            self.robot.SetDOFValues(start_joints, self.manipulator.GetArmIndices())
            problem = trajoptpy.ConstructProblem(json.dumps(request),
                                                 self.env)
            result = trajoptpy.OptimizeProblem(problem)
            # problem.SetRobotActiveDOFs()
            # assert traj_is_safe(result.GetTraj(), self.robot)
            traj = result.GetTraj().tolist()
            return np.concatenate((np.tile(start_joints, (5, 1)), traj),
                                  axis=0)

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

        # Set robot joint angles
        self.robot.SetDOFValues(self.work_order.start_joints,
                                self.manipulator.GetArmIndices())

    def update_state(self):
        self.update_cubbyhole_and_objects()
        self.update_target_object_approaches()
        self.update_simulation_environment()

    def _find_joints(self, joints_attr, pose_attr):
        for approach in self.target_object_approaches:
            joints, distance = self.get_robot_joints(getattr(approach, pose_attr))
            setattr(approach, joints_attr, joints)
            setattr(approach, "{}_dist".format(joints_attr), distance)
            
            if getattr(approach, joints_attr) is None:
                rospy.logwarn("no IK solution for " + pose_attr)

    def _find_trajectory(self, trajectory_attr, target_joints_attr,
                         start_joints_attr='', dist_pen=0.02):
        for approach in self.target_object_approaches:
            setattr(approach, trajectory_attr,
                    self.get_robot_trajectory(
                        getattr(approach, target_joints_attr),
                        getattr(approach, start_joints_attr, None),
                        dist_pen
                    ))

    def _filter_approaches_by(self, filter_attr):
        self.target_object_approaches[:] = \
            [approach for approach in self.target_object_approaches
             if getattr(approach, filter_attr, None) is not None]

    def find_motion_plan(self):
        # Go to pregrasp
        self._find_joints('pregrasp_joints', 'pregrasp_pose')
        self._filter_approaches_by('pregrasp_joints')
        self._find_trajectory('pregrasp_trajectory', 'pregrasp_joints')
        self._filter_approaches_by('pregrasp_trajectory')

        # Open gripper
        self.robot.SetDOFValues(self.gripper_limits[1],
                                self.manipulator.GetGripperIndices())

        # Go to grasp
        self._find_joints('grasp_joints', 'grasp_pose')
        self._filter_approaches_by('grasp_joints')
        self._find_trajectory('grasp_trajectory', 'grasp_joints',
                              'pregrasp_joints')
        self._filter_approaches_by('grasp_trajectory')

        # Close gripper

        # Go to postgrasp
        self._find_joints('postgrasp_joints', 'postgrasp_pose')
        # self._filter_approaches_by('postgrasp_joints')
        self._find_trajectory('postgrasp_trajectory', 'postgrasp_joints',
                              'grasp_joints', 0.02)
        # self._filter_approaches_by('postgrasp_trajectory')

        # # Go to waypoint
        # self._find_joints('waypoint_joints', 'waypoint_pose')
        # self._find_trajectory('waypoint_trajectory', 'waypoint_joints',
        #                       'postgrasp_joints')

        # Go to dropzone
        self._find_joints('dropzone_joints', 'dropzone_pose')
        # self._filter_approaches_by('dropzone_joints')
        self._find_trajectory('dropzone_trajectory', 'dropzone_joints',
                              'postgrasp_joints', 0.15)
        # self._filter_approaches_by('dropzone_trajectory')

        # Open gripper

    def create_motion_plan(self):
        if not len(self.target_object_approaches):
            return None

        approach = min(self.target_object_approaches, key=lambda approach: approach.pregrasp_joints_dist + approach.grasp_joints_dist) 
        
        # Strategy: Simple
        def simulate_trajectory(traj):
            for joints in traj:
                self.robot.SetDOFValues(joints, self.manipulator.GetArmIndices())
                rospy.sleep(0.5)
        
        simulate_trajectory(approach.pregrasp_trajectory)
        simulate_trajectory(approach.grasp_trajectory)
        joint_trajectories = [
            # Adjust torse
            JointTrajectoryWrapper(self.torso_joint_names,
                                   [self.robot.GetDOFValues([12])]).to_msg(),

            # Go to pregrasp
            JointTrajectoryWrapper(self.arm_joint_names,
                                   approach.pregrasp_trajectory).to_msg(),

            # Open gripper
            JointTrajectoryWrapper(self.gripper_joint_names,
                                   [self.gripper_limits[1]]).to_msg(),

            # Go to grasp
            JointTrajectoryWrapper(self.arm_joint_names,
                                   approach.grasp_trajectory).to_msg(),

            # Close gripper
            JointTrajectoryWrapper(self.gripper_joint_names,
                                   [self.gripper_limits[0]]).to_msg(),

            # Go to postgrasp
            JointTrajectoryWrapper(self.arm_joint_names,
                                   approach.postgrasp_trajectory).to_msg(),

            # # Go to waypoint
            # JointTrajectoryWrapper(self.arm_joint_names,
            #                        approach.waypoint_trajectory).to_msg(),

            # Go to dropzone
            JointTrajectoryWrapper(self.arm_joint_names,
                                   approach.dropzone_trajectory).to_msg(),

            # Open gripper
            JointTrajectoryWrapper(self.gripper_joint_names,
                                   [self.gripper_limits[1]]).to_msg()
        ]

        return MotionPlan(self.work_order.strategy, joint_trajectories)

    def handle_get_motion_plan(self, req):
        self.work_order = req.work_order
        self.update_state()

        self.find_motion_plan()
        return GetMotionPlanResponse(self.create_motion_plan())

"""
if __name__ == '__main__':
    with APCPlannerService("work_orders", 10, 'leftarm') as planner:
        planner.spin()"""
