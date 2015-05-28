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
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from std_msgs.msg import Header

from planning import IkSolver
from apc.msg import MotionPlan
from apc.srv import GetObjectPose, GetMotionPlan, GetMotionPlanResponse
from ros_utils import ROSNode, fromPoseMsg
from utils import FAKE, NEW_SHELF, MODEL_DIR, OBJ_MESH_DIR, MESH_DIRECTORY, \
                  trajopt_request_template, timed, order_bin_pose, drop_bin_pose
from test_utils import *
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
    
    waypoint_L = np.array([1.57,0.2,1.57,-.8,2,-1.57,-1.57])

    waypoint_R = np.array([-1.68, -0.35, -0.008, -1.53, -1.57, -2.02, 3.1])
    
    pickup_bin_start = np.array([ 1.87963349,  1.3823445 ,  1.0119    , -1.17227282,  2.97539337, -1.27516323,  2.49392547])
    pickup_bin_end = np.array([ 1.32378987,  0.62894078,  1.3       , -1.46773524,  2.23295032, -1.72606248,  3.13967596])

    def __init__(self, update_rate, work_orders_topic, plan_type, shelf_name):
        super(APCPlannerService, self).__init__('plan_' + plan_type)

        self.plan_type = plan_type
        self.work_orders_topic = work_orders_topic
        self.update_rate = update_rate

        self.object_poses = {}
        self.shelf_name = shelf_name
        self.work_order = None

        self.get_marker_pose_client = rospy.ServiceProxy('/apc/get_object_pose', GetObjectPose)
                                                         
        self.get_motion_plan_service = rospy.Service('get_motion_plan_' + self.plan_type,
                                                     GetMotionPlan,
                                                     self.handle_get_motion_plan)
        
    def __enter__(self):
        self.env = rave.Environment()
        
        if NEW_SHELF:
            self.env.Load(osp.join(MODEL_DIR, self.shelf_name + ".kinbody.xml"))
        else:
            self.env.Load(osp.join(MESH_DIRECTORY, 'cubbyholes', self.shelf_name + ".stl"))
        self.shelf = self.env.GetKinBody(self.shelf_name)
        
        if self.plan_type == "hook":
            self.env.Load(osp.join(MODEL_DIR, "pr2-ow-box-hook.xml"))
            all_manips = ["rightarm", "rightarm_torso", "leftarm", "leftarm_torso", "leftarm_box", "rightarm_hook", "rightarm_torso_hook"]
        elif self.plan_type == "grasp":
            self.env.Load(osp.join(MODEL_DIR, "pr2-beta-static.zae"))
            all_manips = ["rightarm", "rightarm_torso", "leftarm", "leftarm_torso"]
            self.env.Load(osp.join(MODEL_DIR, "order_bin.kinbody.xml"))
            self.order_bin = self.env.GetKinBody("order_bin")
            self.order_bin.SetTransform(order_bin_pose)
            
        #'pr2-with-box.xml')) #"pr2-beta-static.zae"))
 
        self.robot = self.env.GetRobot('pr2')
        self.robot.SetDOFValues([0.548,-1.57, 1.57, 0.548],[22,27,15,34])

        self.env.SetViewer('qtcoin')
        for manip in all_manips:
            m = self.robot.SetActiveManipulator(manip)
            ik = IkSolver(self.env)
            m.SetIkSolver(ik.ikmodel.iksolver)
        self.ik = IkSolver(self.env)
        rospy.logwarn("ready for planning, strategy is " + self.plan_type)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        print self.plan
        #rave.RaveDestroy()

    def find_trajectory(self, manip, start_joints, end_joints, fail_coll=True, dist_pen=0.02, coll_cost=100):
        """ Return a trajectory from (start_joints) to (end_joints) with the given base_pos, torso_height """
                                    
        manip = self.robot.SetActiveManipulator(manip)      
        self.robot.SetActiveDOFs(manip.GetArmIndices())
        self.robot.SetDOFValues(start_joints, manip.GetArmIndices())
        
        request = trajopt_request_template()
        request['constraints'][0]['params']['vals'] = \
            request['init_info']['endpoint'] = list(end_joints)
        request['basic_info']['start_fixed'] = True
        request['basic_info']['manip'] = "active"
        request['costs'][1]['params']['dist_pen'] = [dist_pen]
        request['costs'][1]['params']['coeffs'] = [coll_cost]
        
        
         
        if "box" in manip.GetName():
            request['basic_info']['n_steps'] = 4
        elif "box" in manip.GetName():
            request['basic_info']['n_steps'] = 6  
            
        problem = trajoptpy.ConstructProblem(json.dumps(request), self.env)
        """
        if "box" in manip.GetName() or "hook" in manip.GetName():
            arm_inds = manip.GetArmIndices()
            if manip.GetName() == "leftarm_box":
                local_dir = np.array([0,0,1])
            elif manip.GetName() == "rightarm_hook" or manip.GetName() == "rightarm_torso_hook":
                local_dir = manip.GetTransform().dot([0,0,1,1]) - manip.GetTransform().dot([0,0,0,1])
                local_dir = local_dir[:3]
            tool_link = manip
            robot = self.robot
            arm_joints = [robot.GetJointFromDOFIndex(d) for d in manip.GetArmIndices()]
            def f(x):
                robot.SetDOFValues(x, arm_inds, False)
                return tool_link.GetTransform()[:2,:3].dot(local_dir)
            def dfdx(x):
                robot.SetDOFValues(x, arm_inds, False)
                world_dir = tool_link.GetTransform()[:3,:3].dot(local_dir)
                return np.array([np.cross(joint.GetAxis(), world_dir)[:2] for joint in arm_joints]).T.copy()       
            for t in xrange(1,request['basic_info']['n_steps']):
                problem.AddConstraint(f, dfdx, [(t,j) for j in xrange(7)], "EQ", "up%i"%t)
        """
        result = trajoptpy.OptimizeProblem(problem)
        traj = result.GetTraj()
        if fail_coll and not traj_is_safe(traj, self.robot):
            return None
        return traj
        
    def intr_trajectory(self, manip, start, end):
        return np.array([np.linspace(start[i], end[i], 10) for i in range(len(start))]).T
        
            
    def find_motion_plan_grasp(self):
        """ Compute IK for pregrasp, grasp, drop, then find trajectories for each. Supports base movement. """
    
        self.plan = {}
        
        if self.try_default:
            self.plan["grasp"] = self.ik.GetDefaultGrasp(self.work_order.bin_name, self.work_order.target_object)
        else:
            self.plan["grasp"]  = timed(self.ik.GetRaveIkSol, [self.work_order.target_object, False])
        
        manip = "leftarm"
        
        print self.plan["grasp"]
        if self.plan["grasp"] is None or self.plan["grasp"]["joints"] is None:
            rospy.logwarn("trying default grasp")
            self.plan["grasp"] = self.ik.GetDefaultGrasp(self.work_order.bin_name, self.work_order.target_object)
            if self.plan["grasp"] is None or self.plan["grasp"]["joints"] is None:
                rospy.logwarn("no iksol found")
                return None
            
        self.robot.SetTransform(self.plan["grasp"]["base"])
        self.robot.SetDOFValues(self.plan["grasp"]["joints"], self.robot.GetManipulator(self.plan["grasp"]["manip"]).GetArmIndices())
        
        self.plan["pregrasp"] = self.ik.GetPregraspJoints(manip, self.plan["grasp"]["target"])
        self.plan["drop"] = self.ik.GetDropJoints(manip, self.plan["grasp"]["target"], self.order_bin.GetTransform()[:3,3])
        self.plan["postgrasp"] = self.ik.GetPostgraspJoints(manip, self.plan["grasp"]["target"])
        
        for k,v in self.plan.items():
            if v is None:
                rospy.logwarn("no pose found for " + k)
                return None
        
        objs = [self.env.GetKinBody(ob) for ob in self.work_order.bin_contents]
        objposes = [ob.GetTransform() for ob in objs]     
        [self.env.Remove(ob) for ob in objs]
        
        trajectories = []
        trajectories.append([[0.54]])     
        
        trajectories.append(self.find_trajectory(manip, self.work_order.left_joints[1:], self.plan["pregrasp"]["joints"]))                                                           
        trajectories.append(self.find_trajectory(manip, self.plan["pregrasp"]["joints"], self.plan["grasp"]["joints"][1:]))       
        trajectories.append([[0]])                 
                                                                                                                                                                                          
        trajectories.append(self.find_trajectory(manip, self.plan["grasp"]["joints"][1:], self.plan["postgrasp"]["joints"]))   
           
        for i,ob in enumerate(objs):
            self.env.AddKinBody(ob)
            ob.SetTransform(objposes[i])
        
        self.trajs = trajectories
        
        self.robot.SetTransform(self.plan["grasp"]["base"])
        self.robot.SetDOFValues([self.plan["grasp"]["joints"][0]], [self.robot.GetJoint("torso_lift_joint").GetDOFIndex()])
        
        traj_wrap = self.replay(trajectories,8)
        if traj_wrap is None:
            return None  
                        
        drop_traj = []
        self.robot.SetTransform(drop_bin_pose)

        drop_traj.append(self.find_trajectory(manip, self.plan["postgrasp"]["joints"], self.plan["drop"]["joints"]))  
        trajectories.append([[0.54]]) 
        drop_wrap = self.replay(drop_traj, 2)
                                                                                                            
        self.motion_plan = MotionPlanWrapper(self.work_order.strategy, traj_wrap, self.plan["grasp"]["base"][:3,3], self.plan["grasp"]["joints"][0])   
        self.drop_plan = MotionPlanWrapper(self.work_order.strategy, drop_wrap, drop_bin_pose[:3,3], self.plan["grasp"]["joints"][0])   
        
        return [self.motion_plan.to_msg(), self.drop_plan.to_msg()]
           
    def find_motion_plan_hook(self):
        self.plan = {}
        
        base, torso = self.ik.GetHookBaseTorso(self.work_order.bin_name)     
        self.robot.SetDOFValues([torso], [self.robot.GetJoint("torso_lift_joint").GetDOFIndex()])
        self.robot.SetTransform(base)
    
        self.plan["binholder"] = self.ik.GetBinholderJoints(self.work_order.bin_name)      
        if self.plan["binholder"] is None:
            print "no pose found for binholder"
            return self.plan
                        
        self.plan["binholder"] = self.ik.GetBinholderJoints(self.work_order.bin_name)      
        if self.plan["binholder"] is None:
            print "no pose found for binholder"
            return self.plan
                        
        self.robot.SetDOFValues(plan["binholder"]["joints"], self.robot.GetManipulator("leftarm_box").GetArmIndices())

        for which in ['prehook', 'insert', 'twist', 'out']:
            self.plan[which] = ik.GetHookJoints(self.work_order.bin_name, self.work_order.target_object, which)

        for k,v in self.plan.items():
            if v is None:
                print "no pose found for", k
                return plan
                
        objs = [self.env.GetKinBody(ob) for ob in self.work_order.bin_contents]
        objposes = [ob.GetTransform() for ob in objs]     
        [self.env.Remove(ob) for ob in objs]
        
        trajectories = []
        trajectories.append(self.find_trajectory("leftarm_box", self.work_order.left_joints[1:], self.plan["binholder"]["joints"], 0.05))
        self.robot.SetDOFValues(self.plan["binholder"]["joints"], self.robot.GetManipulator("leftarm_box").GetArmIndices())
        
        trajectories.append([[0]])
        trajectories.append(self.find_trajectory("rightarm_hook", self.work_order.right_joints[1:], self.waypoint_R))  
        trajectories.append(self.find_trajectory("rightarm_hook", self.waypoint_R, self.plan["prehook"]["joints"][1:]))  
        trajectories.append(self.find_trajectory("rightarm_hook", self.plan["prehook"]["joints"][1:], self.plan["insert"]["joints"])) 
        trajectories.append(self.find_trajectory("rightarm_hook", self.plan["insert"]["joints"], self.plan["twist"]["joints"], False)) 
        trajectories.append(self.find_trajectory("rightarm_hook", self.plan["twist"]["joints"], self.plan["out"]["joints"], False, 0.1, 10)) 
        trajectories.append(self.find_trajectory("rightarm_hook", self.plan["out"]["joints"],self.waypoint_R)) 
        
        self.trajs = trajectories
        
        for i,ob in enumerate(objs):
            self.env.AddKinBody(ob)
            ob.SetTransform(objposes[i])
        
        traj_wrap = self.replay(trajectories,1)
        if traj_wrap is None:
            return None
        self.motion_plan = MotionPlanWrapper(self.work_order.strategy, traj_wrap, base[:3,3], torso)   
        return self.motion_plan.to_msg()
        
    
    def find_pickup_bin_plan():
        if self.plan_type == "grasp":     
            """
            
            
            pos, size = order_bin.ComputeAABB().pos(), order_bin.ComputeAABB().extents()

            x = pos[0] - size[0]
            y = pos[1]
            z = pos[2] + size[2] - 0.05
            
            pose = np.array([0,0.7071,0.7071,0,x,y,z])

            ikparam = rave.IkParameterization(pose, self.ik.ikmodel.iktype)
            opt = rave.IkFilterOptions.CheckEnvCollisions
            iksol = self.robot.GetManipulator('leftarm_torso').FindIKSolution(ikparam, opt)
            """
            T = drop_bin_pose.copy()
            self.robot.SetTransform(T)
            
            trajectories = []
            self.robot.SetDOFValues([0], [12])
            trajectories.append([[0.54]])
            trajectories.append(self.find_trajectory("leftarm", self.work_order.left_joints[1:], self.pickup_bin_start))
            trajectories.append([[0]])
            
            motion_plans = []
            
            traj_wrap = self.replay(trajectories, 3)
            if traj_wrap is None:
                return None
            motion_plans.append(MotionPlanWrapper(self.work_order.strategy, traj_wrap, T[:3,3], 0).to_msg())
            
            trajectories = []
            T[0,3] = -1.5
            self.robot.SetTransform(T)
            self.robot.SetDOFValues([0.1], [12])
            trajectories.append(self.intr_trajectories('leftarm', self.pickup_bin_start, self.pickup_bin_end))
            
            traj_wrap = self.replay_trajectories(trajectories, 2)
            motion_plans.append(MotionPlanWrapper(self.work_order.strategy, traj_wrap, T[:3,3], 0.1).to_msg())
            
            return motion_plans
            
    def replay(self, trajectories, numL=0):
        #raw_input("start replay?")
        traj_wrap = []
        for i,traj in enumerate(trajectories):
            m = "leftarm" if i < numL else "rightarm"
            if traj is None:
                rospy.logwarn("trajectory not safe: " + str(i))
                return None                     
            for t in traj:
                if len(t) == 1:
                    self.robot.SetDOFValues(t, self.robot.GetManipulator(m).GetGripperJoints())
                    names = ['r'+self.gripper_joint_name if 'right' in m else 'l'+self.gripper_joint_name]            
                else:
                    self.robot.SetDOFValues(t, self.robot.GetManipulator(m).GetArmIndices())
                    names = self.arm_joint_names
                rospy.sleep(0.1)
            joint_names = ["r"+j if "right" in m else "l"+j for j in names]
            traj_wrap.append(JointTrajectoryWrapper(joint_names, traj))    
        return traj_wrap
            
                    
    def handle_get_motion_plan(self, req):
        """ Main method of service call. """
        self.work_order = req.work_order
        
        msgs = []
        
        if self.work_order.strategy != 'pick_up_bin':
            self.update_state()

            if self.no_perception:
                rospy.logwarn("no perception")
                msg = None  
                
            elif self.plan_type == "grasp":
                msg_lst = self.find_motion_plan_grasp()
                if len(msg_lst) >= 2:
                    msgs.extend(msg_lst)
                    
            elif self.plan_type == "hook":
                msg = self.find_motion_plan_hook()
                if msg is not None:
                    msgs.append(msg)
        else:
            msgs = self.find_pickup_bin_plan()
                
        if len(msgs) > 0:
            return GetMotionPlanResponse(msgs)
        else:
            return GetMotionPlanResponse([MotionPlanWrapper("failed", [], [], 0).to_msg()])
 
 
 
         
    def update_state(self):
        """ Update body poses in the local rave env. """
        self.no_perception = False
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
                    res = self.get_marker_pose_client(self.work_order.bin_name, object_name)
                    self.object_poses[object_name] = res.obj_pose
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))

    def update_simulation_environment(self):
        """
        Update objects with poses as from 'update_cubbyhole_and_objects()' 
        Update robot position and joint angles. Shelf stays at eye(4).
        Saved poses are relative to robot but here change everything to be relative to shelf
        """       

        [self.env.Remove(body) for body in self.env.GetBodies() if body != self.robot and body != self.shelf and body.GetName() != "order_bin"]

        world_to_robot = rave.matrixFromPose(self.work_order.robot_pose)

        for object_name in self.work_order.bin_contents:
            self.env.Load(osp.join(OBJ_MESH_DIR, object_name + ".stl"))
            
            obj_mat = None
            
            if FAKE:
                rospy.logwarn("using fake data")
                obj_mat = rave.matrixFromPose(np.loadtxt("obj.txt")[0])
            elif object_name in self.object_poses:        
                pose = fromPoseMsg(self.object_poses[object_name])
                if pose is not None:
                    if len(pose) == 7:
                    #robot_to_object = rave.matrixFromPose(pose)
                    #obj_mat = world_to_robot.dot(robot_to_object)
                        obj_mat = rave.matrixFromPose(pose)
                        self.try_default = False
                    else:
                        self.try_default = True
                elif object_name == self.work_order.target_object:
                    self.no_perception = True
                    rospy.logwarn("no perception for object " + object_name)
            elif object_name == self.work_order.target_object:
                self.no_perception = True
                rospy.logwarn("no perception for object " + object_name)
                
            if obj_mat is not None:
                rospy.logwarn(object_name + ": " + rave.poseFromMatrix(obj_mat).tolist().__str__())
                self.env.GetKinBody(object_name).SetTransform(obj_mat)
                
                  
        self.robot.SetDOFValues([0.548, 0.548],[22, 34]) # open the grippers
        self.robot.SetDOFValues([-1.57,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ], self.robot.GetManipulator("rightarm").GetArmIndices())
        self.robot.SetDOFValues([ 1.57,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ], self.robot.GetManipulator("leftarm").GetArmIndices())
        self.robot.SetTransform(world_to_robot)
        
if __name__ == '__main__':
    with APCPlannerService(10, "work_orders", "hook", "pod_lowres") as hook:
        hook.spin()
