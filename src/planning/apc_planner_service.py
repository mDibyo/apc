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

        self.shelf = self.env.GetKinBody(self.shelf_name)
        self.env.SetViewer('qtcoin')
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        rave.RaveDestroy()




    def find_trajectory(self, manip_name, start_joints, end_joints, base_pos, dist_pen=0.02):
        """ Return a trajectory from (start_joints) to (end_joints) with the given base_pos """
                                    
        manip = self.robot.GetManipulator(manip_name)       
        self.robot.SetActiveDOFs(manip.GetArmIndices())

        request = trajopt_request_template()
        request['constraints'][0]['params']['vals'] = \
            request['init_info']['endpoint'] = end_joints.tolist()
        request['basic_info']['start_fixed'] = True
        request['basic_info']['manip'] = "active"
        request['costs'][1]['params']['dist_pen'] = [dist_pen]
        
        with self.robot:
            T = self.robot.GetTransform()
            T[:3,3] = base_pos
            r.SetTransform(T)
            
            problem = trajoptpy.ConstructProblem(json.dumps(request), self.env)
            result = trajoptpy.OptimizeProblem(problem)
            traj = result.GetTraj()
            return traj
            
 
 
        
    def find_motion_plan(self):
        """ Compute IK for pregrasp, grasp, drop, then find trajectories for each. Supports base movement. """
        pregrasp = self.ik.GetPregraspJoints(self.work_order.target_object)
        grasp = timed(self.ik.GetRaveIkSol, [self.work_order.target_object, False])
        drop = self.ik.GetDropJoints()
        
        trajectories = []
        if None not in [pregrasp, grasp, drop]:
            manip_name, base_pos = grasp["manip"], grasp["base"][:3, 3]
                       
            trajectories.append(self.find_trajectory(manip_name, self.work_order.start_joints, pregrasp["joints"], base_pos))
                                                                        
            trajectories.append(self.find_trajectory(manip_name, pregrasp["joints"], grasp["joints"], base_pos))
                                                                                                                                        
            trajectories.append(self.find_trajectory(manip_name, grasp["joints"], pregrasp["joints"], base_pos))
                                                                    
            trajectories.append(self.find_trajectory(manip_name, pregrasp["joints"], drop["joints"], base_pos))       
        
            joint_names = [j.GetName() for j in self.robot.GetJoints(manip.GetArmIndices())]                                                                                                    
            motion_plan = MotionPlanWrapper(self.work_order.strategy, trajectories, joint_names, base_pos)   
            return motion_plan.to_msg()


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
        Get shelf, object state from perception.
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

        world_to_robot = rave.matrixFromPose(np.hstack([[1,0,0,0], self.work_order.base_pos]))

        for object_name in self.work_order.bin_contents:
            self.env.Load(osp.join(OBJ_MESH_DIR, object_name + ".stl"))
            robot_to_object = self.object_poses[object_name]
            self.env.GetKinBody(object_name).SetTransform(world_to_robot.dot(robot_to_object))

        self.robot.SetDOFValues(self.work_order.start_joints, self.robot.GetManipulator("rightarm_torso").GetArmIndices())
        self.robot.SetDOFValues([0.548, 0.548],[22, 34]) # open the grippers
        self.robot.SetTransform(world_to_robot)
                                
        
            
            
"""
if __name__ == '__main__':
    with APCPlannerService("work_orders", 10) as planner:
        planner.spin()"""
