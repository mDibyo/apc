#!/usr/bin/env python

from __future__ import division
import os.path as osp
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

from planning import IkSolver
from apc.msg import MotionPlan
from apc.srv import GetMarkerPose, GetMotionPlan, GetMotionPlanResponse
from ros_utils import ROSNode
from message_wrappers import GraspWrapper, JointTrajectoryWrapper
from utils import MODEL_DIR, trajopt_request_template, timed, order_bin_pose

class APCPlannerService(ROSNode):
    def __init__(self, work_orders_topic, update_rate, shelf_name="pod_lowres"):
        super(APCPlannerService, self).__init__('plan')

        self.work_orders_topic = work_orders_topic
        self.update_rate = update_rate

        self.object_poses = {}
        self.shelf_name = shelf_name
        self.work_order = None

        self.tf_listener = tf.TransformListener()

        rospy.wait_for_service('get_marker_pose')
        self.get_marker_pose_client = rospy.ServiceProxy('get_marker_pose',
                                                         GetMarkerPose)
        self.get_motion_plan_service = rospy.Service('get_motion_plan',
                                                     GetMotionPlan,
                                                     self.handle_get_motion_plan)
        self.ik = IkSolver(self.env)
        
    def __enter__(self):
        self.env = rave.Environment()
        self.env.Load(osp.join(MODEL_DIR, "pr2-beta-static.zae"))
        self.env.Load(osp.join(MODEL_DIR, self.shelf_name + ".kinbody.xml"))
        
        self.robot = self.env.GetRobot('pr2')
        self.robot.SetActiveDOFs(self.manipulator.GetArmIndices(), 
                    rave.DOFAffine.X + rave.DOFAffine.Y + rave.DOFAffine.RotationAxis, [0,0,1])                            
        self.reset_robot(False)

        self.shelf = self.env.GetKinBody(self.shelf_name)
        self.env.SetViewer('qtcoin')
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        rave.RaveDestroy()

    def get_robot_trajectory(self, target_joints, start_joints=None, dist_pen=0.02):
        if start_joints is None:
            start_joints = self.robot.GetDOFValues(self.manipulator.GetArmIndices())

        request = trajopt_request_template()
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

    def find_motion_plan(self):
        iksol = timed(self.ik.GetRaveIkSol, [self.work_order.target_object, False])
        if iksol is not None:
            self.robot.SetActiveManipulator(iksol["manip"])
            traj_to_grasp = find_trajectory(iksol["joints"], iksol["base"])
            
            
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
        """ Main method of service call. """
        self.work_order = req.work_order
        self.update_state()
        self.find_motion_plan()
        return GetMotionPlanResponse(self.create_motion_plan())
         
   def update_state(self):
        """ Update body poses in the local rave env. """
        self.update_objects()
        self.update_simulation_environment()     
           
   def update_objects(self):
        """ 
        Get shelf, object, and robot state from perception.
        Save poses as static fields. Must be called before 'update_simulation_environment()'
        All poses should be relative to robot ("base_link")
        """    
        self.object_poses = {}
        
        if self.work_order:
            try:
                for object_name in self.work_order.bin_contents:
                    res = self.get_marker_pose_client("object_{}".format(object_name))  ### TODO: this part should come from perception ###
                    self.object_poses[object_name] = res.marker_pose
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))

    def update_simulation_environment(self):
        """
        Update objects with poses as from 'update_cubbyhole_and_objects()' 
        Update robot position and joint angles. Shelf stays at eye(4).
        Saved poses are relative to robot but here change everything to be relative to shelf
        """       

        [self.env.Remove(body) for body in self.env.GetBodies() if body is not self.robot and body is not self.shelf]

        world_to_robot = self.robot.GetTransform() ### TODO: this could be either from odometry or perception ###
        self.robot.SetTransform(world_to_robot)

        for object_name in self.work_order.bin_contents:
            self.env.Load(osp.join(OBJ_MESH_DIR, object_name + ".stl")
            robot_to_object = self.object_poses[object_name]
            self.env.GetKinBody(object_name).SetTransform(world_to_robot.dot(robot_to_object))

        self.reset_robot() ### TODO: listen to topic and get joint angles
                                
                                
                                
                                
    def reset_robot(self, arms_only=True):
        self.robot.SetDOFValues([0.548,-1.57, 1.57, 0.548],[22,27,15,34])
        if not arms_only:
            self.robot.SetTransform(rave.matrixFromPose(np.array([1,0,0,0,-1.2,0,0.2])))
            
            
"""
if __name__ == '__main__':
    with APCPlannerService("work_orders", 10) as planner:
        planner.spin()"""
