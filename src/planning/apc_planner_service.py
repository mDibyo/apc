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
from ros_utils import ROSNode, fromPoseMsg
from utils import MODEL_DIR, OBJ_MESH_DIR, trajopt_request_template, timed, order_bin_pose
from message_wrappers import MotionPlanWrapper, JointTrajectoryWrapper

class APCPlannerService(ROSNode):

    arm_joint_names =     ["_shoulder_pan_joint",
                           "_shoulder_lift_joint",
                           "_upper_arm_roll_joint",
                           "_elbow_flex_joint",
                           "_forearm_roll_joint",
                           "_wrist_flex_joint",
                           "_wrist_roll_joint"]
                           
    gripper_joint_name = "_gripper_l_finger_joint"                     
                           
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
        
    def __enter__(self):
        self.env = rave.Environment()
        self.env.Load(osp.join(MODEL_DIR, "pr2-new-wrists.dae"))#"pr2-beta-static.zae")) #
        self.env.Load(osp.join(MODEL_DIR, self.shelf_name + ".kinbody.xml"))
        self.env.Load(osp.join(MODEL_DIR, "order_bin.kinbody.xml"))
        
        self.robot = self.env.GetRobot('pr2')
        self.robot.SetDOFValues([0.548,-1.57, 1.57, 0.548],[22,27,15,34])
        self.shelf = self.env.GetKinBody(self.shelf_name)
        
        T = self.shelf.GetTransform()
        self.shelf.SetTransform(T)

        self.env.SetViewer('qtcoin')
        for manip in ["rightarm", "rightarm_torso", "leftarm", "leftarm_torso", "leftarm_box"]:
            m = self.robot.SetActiveManipulator(manip)
            ik = IkSolver(self.env)
            m.SetIkSolver(ik.ikmodel.iksolver)
        self.ik = IkSolver(self.env)
        print "ready for planning"
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass#rave.RaveDestroy()

    def find_trajectory(self, manip, start_joints, end_joints, dist_pen=0.02):
        """ Return a trajectory from (start_joints) to (end_joints) with the given base_pos, torso_height """
                                    
        manip = self.robot.SetActiveManipulator(manip)      
        self.robot.SetActiveDOFs(manip.GetArmIndices())
        self.robot.SetDOFValues(start_joints, manip.GetArmIndices())
        
        request = trajopt_request_template()
        request['constraints'][0]['params']['vals'] = \
            request['init_info']['endpoint'] = end_joints.tolist()
        request['basic_info']['start_fixed'] = True
        request['basic_info']['manip'] = "active"
        request['costs'][1]['params']['dist_pen'] = [dist_pen]
        
        with self.robot:       
            problem = trajoptpy.ConstructProblem(json.dumps(request), self.env)
            result = trajoptpy.OptimizeProblem(problem)
            traj = result.GetTraj()
            return traj
            
 
 
        
    def find_motion_plan(self):
        """ Compute IK for pregrasp, grasp, drop, then find trajectories for each. Supports base movement. """
 
        grasp = timed(self.ik.GetRaveIkSol, [self.work_order.target_object, False])
        self.plan = {}
        self.plan["grasp"] = grasp
        #plotPose(self.env, grasp["target"])
        
        if grasp is not None:   
            self.robot.SetTransform(self.plan["grasp"]["base"])
            self.robot.SetDOFValues([self.plan["grasp"]["joints"][0]], [self.robot.GetJoint("torso_lift_joint").GetDOFIndex()])
            
            self.plan["pregrasp"] = self.ik.GetPregraspJoints(self.plan["grasp"]["target"])
            self.plan["drop"] = self.ik.GetDropJoints(self.plan["grasp"]["target"], self.work_order.bin_name)
            self.plan["postgrasp"] = self.ik.GetPostgraspJoints(self.plan["grasp"]["target"])
            self.plan["binholder"] = self.ik.GetBinholderJoints(self.work_order.bin_name)
 
            for k,v in self.plan.items():
                if v is None:
                    rospy.logwarn("no pose found for " + k)
                    return MotionPlanWrapper("failed", [], [], [])  
            
            trajectories = []
            
            trajectories.append(self.find_trajectory("leftarm", self.work_order.left_joints[1:], self.plan["binholder"]["joints"]))
            trajectories.append([[0.54]])     
            
            trajectories.append(self.find_trajectory("rightarm", self.work_order.right_joints[1:], self.plan["pregrasp"]["joints"][1:]))                                                           
            trajectories.append(self.find_trajectory("rightarm", self.plan["pregrasp"]["joints"][1:], self.plan["grasp"]["joints"][1:]))       
            trajectories.append([[0]])                 
                                                                                                                                                                                              
            trajectories.append(self.find_trajectory("rightarm", self.plan["grasp"]["joints"][1:], self.plan["postgrasp"]["joints"][1:]))   
            trajectories.append(self.find_trajectory("rightarm", self.plan["postgrasp"]["joints"][1:], self.plan["drop"]["joints"][1:]))    
            trajectories.append([[0.54]])     
            
            
            self.robot.SetTransform(grasp["base"])
            self.robot.SetDOFValues([grasp["joints"][0]], [self.robot.GetJoint("torso_lift_joint").GetDOFIndex()])
            
            traj_wrap = []
            for i,traj in enumerate(trajectories):
                m = "leftarm" if i == 0 else "rightarm"                     
                for t in traj:
                    if len(t) > 1:
                        self.robot.SetDOFValues(t, self.robot.GetManipulator(m).GetArmIndices())
                        names = [self.gripper_joint_name]            
                    else:
                        self.robot.SetDOFValues(t, [34])
                        names = self.arm_joint_names
                        
                    joint_names = ["r"+j if "r" in m else "l"+j for j in names]  
                    traj_wrap.append(JointTrajectoryWrapper(joint_names, t))
                    rospy.sleep(0.5)
                                                                                                             
            motion_plan = MotionPlanWrapper(self.work_order.strategy, traj_wrap, grasp["base"][:3,3], grasp["joints"][0])   
            return motion_plan.to_msg()
        else:
            rospy.logwarn("no iksol found")
            return MotionPlanWrapper("failed", [], [], [])   
            
            
    def handle_get_motion_plan(self, req):
        """ Main method of service call. """
        self.work_order = req.work_order
        self.update_state()
        msg = self.find_motion_plan()
        return GetMotionPlanResponse(msg)
 
 
 
         
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

        [self.env.Remove(body) for body in self.env.GetBodies() if body != self.robot and body != self.shelf]

        world_to_robot = rave.matrixFromPose(self.work_order.robot_pose)

        for object_name in self.work_order.bin_contents:
            self.env.Load(osp.join(OBJ_MESH_DIR, object_name + ".stl"))
            robot_to_object = rave.matrixFromPose(fromPoseMsg(self.object_poses[object_name]))
            self.env.GetKinBody(object_name).SetTransform(world_to_robot.dot(robot_to_object))
            self.env.GetKinBody(object_name).SetTransform( \
                rave.matrixFromPose(np.array([ 1.      ,  0.      ,  0.      ,  0.      , -0.4318  ,  0.2794  , 1.007835])))

        #self.robot.SetDOFValues(self.work_order.start_joints, self.robot.GetManipulator("rightarm_torso").GetArmIndices())
        self.robot.SetDOFValues([0.548, 0.548],[22, 34]) # open the grippers
        self.robot.SetTransform(world_to_robot)

if __name__ == '__main__':
    with APCPlannerService("work_orders", 10, "cubbyhole_all_combined") as self:
        self.spin()
