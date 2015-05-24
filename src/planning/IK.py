from __future__ import division

import os.path as osp
import time


import numpy as np
from scipy.interpolate import NearestNDInterpolator as interp
import openravepy as rave

from utils import DATA_DIRECTORY, runInParallel, NEW_WRISTS, NEW_SHELF, bin_pose, MOVE_BASE
from Grasps import Grasp, GraspSet
from test_utils import *

class IkSolver(object):

    env = None
    robot = None
    basePositions = np.load(osp.join(DATA_DIRECTORY,"simulated","reachability_NW_R_base.npy"))
    objPoses = np.load(osp.join(DATA_DIRECTORY,"simulated","reachability_NW_R_gripper.npy"))
    nn = interp(objPoses,basePositions)
    
    over_bin_quat = np.array([0, np.sqrt(2)/2, -np.sqrt(2)/2, 0])
    
    if NEW_WRISTS:
        face_front_quat = np.array([np.sqrt(2)/2, 0, 0, np.sqrt(2)/2])
    else:
        face_front_quat = np.array([np.sqrt(2)/2, 0, np.sqrt(2)/2, 0])
    over_bin_quat = np.array([0, np.sqrt(2)/2, -np.sqrt(2)/2, 0])
    
    def __init__(self, env):
        IkSolver.env = env
        r = env.GetRobot("pr2")
        IkSolver.robot = r
        m = r.GetActiveManipulator()
        
        IkSolver.ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(r, iktype=rave.IkParameterization.Type.Transform6D)
        if not IkSolver.ikmodel.load():
            IkSolver.ikmodel.autogenerate()
            
            
    @staticmethod
    def GetHookJoints(bin_N, objName, which):
        obj = IkSolver.env.GetKinBody(objName)
        obj_mat, obj_size = obj.GetTransform(), obj.ComputeAABB().extents()
        bin_mat = rave.matrixFromPose(bin_pose[bin_N])
        dist = (obj_mat - bin_mat)[:3,3]
        thresh = 0.02
        if which == 'prehook':
            m = "rightarm_torso_hook"
            if dist[2] > thresh:
                rot = rave.quatFromAxisAngle(0 * np.array([1,0,0])) 
                dx, dy, dz = -0.10, -0.08, 0.05
                if dist[1] < 0:
                    dy = -dy  
            else:
                rot = rave.quatFromAxisAngle(np.pi/2 * np.array([1,0,0])) 
                dx, dy, dz = -0.05, 0.04, 0.15
            trans = [bin_mat[0,3] + dx,
                     obj_mat[1,3] + dy,
                     bin_mat[2,3] + dz]
                     
        elif which == 'insert':
            m = "rightarm_hook"
            if dist[2] > thresh:
                rot = rave.quatFromAxisAngle(0 * np.array([1,0,0]))  
                dx, dy, dz = 0.05, -0.1, 0.02
                if dist[1] < 0:
                    dy = -dy         
            else:
                rot = rave.quatFromAxisAngle(np.pi/2 * np.array([1,0,0]))
                dx, dy, dz = 0.05, 0.04, 0.15
            trans = [obj_mat[0,3] + obj_size[0] + dx,
                     obj_mat[1,3] + dy,
                     bin_mat[2,3] + dz]
        elif which == 'twist':
            m = "rightarm_hook"
            if dist[2] > thresh:
                if dist[1] > 0:
                    rot = rave.quatFromAxisAngle(np.pi/2 * np.array([1,0,0]))  
                else:
                    rot = rave.quatFromAxisAngle(-np.pi/2 * np.array([1,0,0])) 
                    
                dx, dy, dz = 0.05, 0.01, 0
                if dist[1] < 0:
                    dy = -dy            
            else:
                rot = rave.quatFromAxisAngle(0 * np.array([1,0,0]))  
                dx, dz = 0.05, -dist[2]/2
            trans = [obj_mat[0,3] + obj_size[0] + dx,
                     obj_mat[1,3] + dy,
                     obj_mat[2,3] + dz]
        elif which == 'out':
            m = "rightarm_hook"
            if dist[2] > thresh:
                dx, dy, dz = -0.10, 0, 0.02
                if dist[1] > 0:
                    rot = rave.quatFromAxisAngle(np.pi/2 * np.array([1,0,0]))
                else:
                    rot = rave.quatFromAxisAngle(-np.pi/2 * np.array([1,0,0]))
                    dy = -dy
                
            else:
                rot = rave.quatFromAxisAngle(0 * np.array([1,0,0]))  
                dx, dy, dz = -0.05, 0, 0.02
            trans = [bin_mat[0,3] + dx,
                     obj_mat[1,3] + dy,
                     obj_mat[2,3] + dz]
                                
        pose = np.hstack([rot, trans])
        ikparam = rave.IkParameterization(pose, IkSolver.ikmodel.iktype)
        opt = rave.IkFilterOptions.CheckEnvCollisions
        iksol = IkSolver.robot.GetManipulator(m).FindIKSolution(ikparam, opt)
        if iksol is not None:
            return {"joints": iksol,
                    "target": pose,
                    "manip" : m}
        else:
            plotPose(IkSolver.env, pose)
            
    @staticmethod
    def GetBinholderJoints(bin_N):
        bin_mat = rave.matrixFromPose(bin_pose[bin_N])  
        z, iksol = bin_mat[2,3]-0.3, None
        while z < bin_mat[2,3] and iksol is None:
            pose = rave.poseFromMatrix(bin_mat)
            pose[-3] -= 0.25
            pose[-1] = z
            ikparam = rave.IkParameterization(pose, IkSolver.ikmodel.iktype)
            opt = rave.IkFilterOptions.CheckEnvCollisions
            iksol = IkSolver.robot.GetManipulator("leftarm_box").FindIKSolution(ikparam, opt)
            if iksol is not None:
                return {"joints": iksol,
                        "target": pose,
                        "manip" : "leftarm_box"}
            z += 0.05
                    
    @staticmethod
    def GetPregraspJoints(graspPose):
        """ IK for pregrasp: face front outside cubbyhole """
        pose = graspPose; pose[-3] -= 0.1
        ikparam = rave.IkParameterization(pose, IkSolver.ikmodel.iktype)
        opt = rave.IkFilterOptions.CheckEnvCollisions
        iksol = IkSolver.robot.GetManipulator("rightarm").FindIKSolution(ikparam, opt)
        if iksol is not None:
            return {"joints": iksol,
                    "manip" : "rightarm"}
                
    @staticmethod
    def GetPostgraspJoints(graspPose):
        """ IK for postgrap: avoid lip"""
        pose = graspPose; pose[-1] += 0.01
        ikparam = rave.IkParameterization(pose, IkSolver.ikmodel.iktype)
        opt = rave.IkFilterOptions.CheckEnvCollisions
        iksol = IkSolver.robot.GetManipulator("rightarm").FindIKSolution(ikparam, opt)
        if iksol is not None:
            return {"joints": iksol,
                    "target": pose,
                    "manip" : "rightarm"}          
                    
    @staticmethod
    def GetDropJoints(graspPose, bin_loc):
        """ IK for above target bin """
        
        dz, iksol = 0.1, None
        while dz < 0.4 and iksol is None:           
            pose = np.hstack([graspPose[:4], bin_loc + np.array([0,-0.1,dz]) ])

            ikparam = rave.IkParameterization(pose, IkSolver.ikmodel.iktype)
            opt = rave.IkFilterOptions.CheckEnvCollisions
            iksol = IkSolver.robot.GetManipulator("rightarm").FindIKSolution(ikparam, opt)
            if iksol is not None:
                return {"joints": iksol,
                        "target": pose,
                        "manip" : "rightarm"}
            dz += 0.05
            
    @staticmethod        
    def solveIK(queue, target, manipName, i):
        """ Actually solves IK for target, manipulator. Append result to multiprocessing.Queue if given. """
        m = IkSolver.robot.GetManipulator(manipName)
       
        ikparam = rave.IkParameterization(target, IkSolver.ikmodel.iktype)
        opt = rave.IkFilterOptions.CheckEnvCollisions 
        
        iksol = m.FindIKSolution(ikparam, opt)
        if iksol is not None:
            sol = {"joints" : iksol,
                   "manip"  : manipName,
                   "target" : target,
                   "point"  : IkSolver.grasps[i].point(target),
                   "grasp"  : IkSolver.grasps[i].mat,
                   "base"   : IkSolver.robot.GetTransform(),
                   "width"  : IkSolver.grasps[i].width}
            if queue is None:
                return sol
            else:
                queue.put(sol)

    @staticmethod
    def GetIkSol(obj, targets, parallel):
        """ Iterate through targets and manipulators to find an IK solution. """
        manips = ["rightarm_torso"]
        if obj.GetTransform()[1,3] < 0:
            manips.reverse()
        
        for manip in manips:
            IkSolver.resetArms()
            if parallel:
                soln = runInParallel(IkSolver.solveIK, [[t,manip,i] for i,t in enumerate(targets)])
                return soln
            else:
                for i,t in enumerate(targets):
                    soln = IkSolver.solveIK(None,t,manip,i)
                    if soln is not None:
                        return soln
   
    @staticmethod    
    def GetRaveIkSol(objName, parallel=False, q=None):
        """ This is the only method that should get called from outside. """       
        IkSolver.resetArms()
        obj = IkSolver.env.GetKinBody(objName)
        pos = IkSolver.robot.GetTransform()
        IkSolver.grasps = GraspSet(objName, IkSolver.env.CloneSelf(rave.CloningOptions.Bodies))

        targets = IkSolver.grasps.GetTargets(obj)
        IkSolver.targets = targets
        
        if len(targets) == 0:
            if q is not None:
                q.put(None)
            return None
            
        rsol, i = None, 0
        if NEW_SHELF:
            shelf = IkSolver.env.GetKinBody("cubbyhole_all_combined")
        else:
            shelf = IkSolver.env.GetKinBody("pod_lowres")
        
            
        while rsol is None and not IkSolver.env.CheckCollision(IkSolver.robot.GetLink("base_link"), shelf):
            IkSolver.resetArms()
            if i == 0:
                pass
            elif i == 1 and MOVE_BASE:
                pos[:2,3] = IkSolver.nn(rave.poseFromMatrix(obj.GetTransform())[np.newaxis])[:,:2]   
            
            """
            elif i == 2:
                IkSolver.positions = IkSolver.naiveMoves()
                pos[:2,3] = IkSolver.positions.pop()
            else:
                if len(IkSolver.positions) == 0:
                    return None
                pos[:2,3] = IkSolver.positions.pop()   
            """
                               
            IkSolver.robot.SetTransform(pos)
            #i += 1
            rsol = IkSolver.GetIkSol(obj, targets, parallel)
        
           
        if q is not None:
            q.put(rsol)
        return rsol
        
    @staticmethod
    def naiveMoves():
        positions = []
        for x in np.linspace(-1.2, 0.7, 30):
            for y in np.linspace(-0.2, 0.9, 30):
                positions.append([x,y])
        return positions
        
    @staticmethod
    def computeMove(obj):
        if obj[1,3] < 0:
            shoulder = IkSolver.robot.GetLink("r_shoulder_pan_link").GetTransform()
            shoulder[1,3] -= 0.1
        else:
            shoulder = IkSolver.robot.GetLink("l_shoulder_pan_link").GetTransform()
            shoulder[1,3] += 0.1

        disp = obj[:3,3] - shoulder[:3,3]
        dy = np.sign(disp[1]) * 0.05
        dx = 0.05
        return dx,dy
        
        
    @staticmethod
    def resetArms():
        IkSolver.robot.SetDOFValues([0.54,-1.57, 1.57, 0.54],[22,27,15,34])
        
        
