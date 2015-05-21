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
from test_utils import plotPose
from message_wrappers import MotionPlanWrapper

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
        
        self.order_bin = self.env.GetKinBody("order_bin")
        self.order_bin.SetTransform(order_bin_pose)
        
        self.env.SetViewer('qtcoin')
        for manip in ["rightarm", "rightarm_torso", "leftarm", "leftarm_torso"]:
            m = self.robot.SetActiveManipulator(manip)
            ik = IkSolver(self.env)
            m.SetIkSolver(ik.ikmodel.iksolver)
        self.ik = IkSolver(self.env)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass#rave.RaveDestroy()




    def find_trajectory(self, manip, start_joints, end_joints, base_pos, torso_height, dist_pen=0.02):
        """ Return a trajectory from (start_joints) to (end_joints) with the given base_pos, torso_height """
                                    
        manip = self.robot.SetActiveManipulator(manip)      
        self.robot.SetActiveDOFs(manip.GetArmIndices())
        self.robot.SetDOFValues(start_joints, manip.GetArmIndices())
        
        T = self.robot.GetTransform()
        T[:2,3] = base_pos[:2]
        self.robot.SetTransform(T)
        self.robot.SetDOFValues([torso_height], [self.robot.GetJoint("torso_lift_joint").GetDOFIndex()])
       
        
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
        self.grasp = grasp
        print "Grasp",grasp
        #plotPose(self.env, grasp["target"])
        if self.grasp is not None:   
            self.robot.SetTransform(grasp["base"])
            self.robot.SetDOFValues([grasp["joints"][0]], [self.robot.GetJoint("torso_lift_joint").GetDOFIndex()])
            
            pregrasp = self.ik.GetPregraspJoints(grasp["target"])
            drop = self.ik.GetDropJoints(grasp["target"])
            postgrasp = self.ik.GetPostgraspJoints(grasp["target"])
            binholder = self.ik.GetBinholderJoints(self.work_order.bin_name)
            print "found all poses"
            
            trajectories = []
            if drop is not None and pregrasp is not None and postgrasp is not None and binholder is not None:
                manip_name, base_pos, torso_height = "rightarm", grasp["base"][:3, 3], grasp["joints"][0]
                manip = self.robot.GetManipulator(manip_name)
                
                trajectories.append(self.find_trajectory("leftarm", self.work_order.left_joints[1:], binholder["joints"], base_pos, torso_height))
                trajectories.append([[0.54]])     
                
                trajectories.append(self.find_trajectory(manip_name, self.work_order.right_joints[1:], pregrasp["joints"][1:], base_pos, torso_height))                                                           
                trajectories.append(self.find_trajectory(manip_name, pregrasp["joints"][1:], grasp["joints"][1:], base_pos, torso_height))       
                trajectories.append([[0]])                 
                                                                                                                                                                                                  
                trajectories.append(self.find_trajectory(manip_name, grasp["joints"][1:], postgrasp["joints"][1:], base_pos, torso_height))   
                trajectories.append(self.find_trajectory(manip_name, postgrasp["joints"][1:], drop["joints"][1:], base_pos, torso_height))    
                trajectories.append([[0.54]])     
                
                
                self.robot.SetTransform(grasp["base"])
                self.robot.SetDOFValues([grasp["joints"][0]], [self.robot.GetJoint("torso_lift_joint").GetDOFIndex()])
                for traj in trajectories:
                    for t in traj:
                        if len(t) > 1:
                            self.robot.SetDOFValues(t, self.robot.GetManipulator("rightarm").GetArmIndices())
                        else:
                            self.robot.SetDOFValues(t, [34])
                        rospy.sleep(0.2)
                
                            
                joint_names = [j.GetName() for j in self.robot.GetJoints(manip.GetArmIndices()) if "torso" not in j.GetName()]                                                                                                    
                motion_plan = MotionPlanWrapper(self.work_order.strategy, trajectories, joint_names, base_pos, torso_height)   
                return motion_plan.to_msg()
            else:
                print "couldn't find pose"

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

        [self.env.Remove(body) for body in self.env.GetBodies() if body != self.robot and body != self.shelf and body != self.order_bin]

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
