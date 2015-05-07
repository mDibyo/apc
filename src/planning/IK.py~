from __future__ import division

import os.path as osp
import time


import numpy as np
from scipy.interpolate import NearestNDInterpolator as interp
import openravepy as rave

from utils import DATA_DIRECTORY, runInParallel
from Grasps import Grasp, GraspSet


class IkSolver(object):

    env = None
    robot = None
    basePositions = np.load(osp.join(DATA_DIRECTORY,"simulated","reachability_NW_R_base.npy"))
    objPoses = np.load(osp.join(DATA_DIRECTORY,"simulated","reachability_NW_R_gripper.npy"))
    nn = interp(objPoses,basePositions)
    
    over_bin_quat = np.array([0, np.sqrt(2)/2, -np.sqrt(2)/2, 0])
    face_front_quat = np.array([np.sqrt(2)/2, 0, np.sqrt(2)/2, 0])
    
    def __init__(self, env):
        IkSolver.env = env
        r = env.GetRobot("pr2")
        IkSolver.robot = r
        m = r.GetActiveManipulator()
        
        IkSolver.ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(r, iktype=rave.IkParameterization.Type.Transform6D)
        
        if not IkSolver.ikmodel.load():
            IkSolver.ikmodel.autogenerate()
            
    @staticmethod
    def GetPregraspJoints(objName):
        """ IK for pregrasp: face front outside cubbyhole """
        obj = IkSolver.env.GetKinBody(objName)
        pose = np.hstack([IkSolver.face_front_quat, [-0.5], obj.GetTransform()[1:3,3]])    
        ikparam = rave.IkParameterization(pose, IkSolver.ikmodel.iktype)
        opt = rave.IkFilterOptions.CheckEnvCollisions
        iksol = IkSolver.robot.GetManipulator("rightarm_torso").FindIKSolution(ikparam, opt)
        return {"joints": iksol,
                "manip" : "rightarm_torso"}
                
                    
    @staticmethod
    def GetDropJoints():
        """ IK for above target bin """
        order_bin = IkSolver.env.GetKinBody("order_bin")
        pose = np.hstack([IkSolver.over_bin_quat, order_bin.GetTransform()[:2,3], 0.5])    
        ikparam = rave.IkParameterization(pose, IkSolver.ikmodel.iktype)
        opt = rave.IkFilterOptions.CheckEnvCollisions
        iksol = IkSolver.robot.GetManipulator("rightarm").FindIKSolution(ikparam, opt)
        return {"joints": iksol,
                "manip" : "rightarm"}
            
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
                   "base"   : IkSolver.robot.GetTransform()}
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
        obj = IkSolver.env.GetKinBody(objName)
        pos = IkSolver.robot.GetTransform()
        IkSolver.grasps = GraspSet(objName, IkSolver.env.CloneSelf(rave.CloningOptions.Bodies))
        obj = IkSolver.env.GetKinBody(objName)
        targets = IkSolver.grasps.GetTargets(obj)
        IkSolver.targets = targets
        if len(targets) == 0:
            if q is not None:
                q.put(None)
            return None
            
        rsol, i = None, 0
        while rsol is None and not IkSolver.env.CheckCollision(IkSolver.robot.GetLink("base_link"),
                                                             IkSolver.env.GetKinBody("pod_lowres")):
            IkSolver.resetArms()
            if i == 0:
                pass
            elif i == 1:
                pos[:3,3] = IkSolver.nn(rave.poseFromMatrix(obj.GetTransform())[np.newaxis])                
            elif i == 2:
                IkSolver.positions = IkSolver.naiveMoves()
                pos[:2,3] = IkSolver.positions.pop()
            else:
                if len(IkSolver.positions) == 0:
                    return None
                pos[:2,3] = IkSolver.positions.pop()   
                                
            IkSolver.robot.SetTransform(pos)
            i += 1
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
        
        
