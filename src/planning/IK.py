from __future__ import division

import os.path as osp
import time

import numpy as np
from scipy.interpolate import NearestNDInterpolator as interp
import openravepy as rave

from utils import DATA_DIRECTORY, runInParallel
from Grasps import Grasp, GraspSet

class IkSolver(object):

    EPS = 1e-4
    env = None
    robot = None
    basePositions = np.load(osp.join(DATA_DIRECTORY,"simulated","basePositions.npy"))
    objPoses = np.load(osp.join(DATA_DIRECTORY,"simulated","poseData.npy"))
    nn = interp(objPoses,basePositions)
    
    @staticmethod
    def resetArms():
        IkSolver.robot.SetDOFValues([0.54,-1.57, 1.57, 0.54],[22,27,15,34])
    
    def __init__(self, env):
        IkSolver.env = env
        r = env.GetRobot("pr2")
        IkSolver.robot = r
        m = r.GetActiveManipulator()
        
        IkSolver.ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(r, iktype=rave.IkParameterization.Type.Transform6D)
        
        if not IkSolver.ikmodel.load():
            IkSolver.ikmodel.autogenerate()
            
    @staticmethod            
    def solveIK_Parallel(queue, target, manipName, i, check=True):
        ikparam = rave.IkParameterization(target, IkSolver.ikmodel.iktype)
        if check:
            opt = rave.IkFilterOptions.CheckEnvCollisions
        else:
            opt = rave.IkFilterOptions.IgnoreEndEffectorEnvCollisions
            
        iksol = IkSolver.robot.GetManipulator(manipName).FindIKSolution(ikparam, opt)
        if iksol is not None:
            if check:
                queue.put({"joints" : iksol,
                           "manip"  : manipName,
                           "target" : target,
                           "point"  : IkSolver.grasps[i].point(target),
                           "grasp"  : IkSolver.grasps[i].mat,
                           "base"   : IkSolver.robot.GetTransform()})
 
            else:
                IkSolver.robot.SetDOFValues(iksol, IkSolver.robot.GetManipulator(manipName).GetArmIndices())
                links = IkSolver.robot.GetLinks()
                numCols = sum([sum([1 if IkSolver.env.CheckCollision(l,o) else 0 for l in links]) for o in
                 IkSolver.env.GetBodies()[1:]])
                if numCols == 0:
                    queue.put({"joints" : iksol,
                               "manip"  : manipName,
                               "target" : target,
                               "point"  : IkSolver.grasps[i].point(target),
                               "grasp"  : IkSolver.grasps[i].mat,
                               "base"   : IkSolver.robot.GetTransform()})
                else:
                    print "found solution:",iksol,"for",manipName,"but has",numCols,"collisions"

    @staticmethod            
    def solveIK(target, manipName, check=True):
        ikparam = rave.IkParameterization(target, IkSolver.ikmodel.iktype)
        if check:
            opt = rave.IkFilterOptions.CheckEnvCollisions
        else:
            opt = rave.IkFilterOptions.IgnoreEndEffectorEnvCollisions
            
        iksol = IkSolver.robot.GetManipulator(manipName).FindIKSolution(ikparam, opt)
        if iksol is not None:
            if check:
                return {"joints": iksol,
                        "manip": manipName,
                        "target":target}
            else:
                IkSolver.robot.SetDOFValues(iksol, IkSolver.robot.GetManipulator(manipName).GetArmIndices())
                links = IkSolver.robot.GetLinks()
                numCols = sum([sum([1 if IkSolver.env.CheckCollision(l,o) else 0 for l in links]) for o in
                 IkSolver.env.GetBodies()[1:]])
                if numCols == 0:
                    return {"joints": iksol,
                            "manip": manipName,
                            "target":target}
                else:
                    print "found solution:",iksol,"for",manipName,"but has",numCols,"collisions"

    @staticmethod
    def GetIkSol(obj, targets, parallel):
        manips = ["leftarm_torso", "rightarm_torso"]
        if obj.GetTransform()[1,3] < 0:
            manips.reverse()
        
        for manip in manips:
            IkSolver.resetArms()
            if parallel:
            
            
            
                soln = runInParallel(IkSolver.solveIK_Parallel, [[t,manip,i] for i,t in enumerate(targets)])
                return soln
            else:
                for i,t in enumerate(targets):
                    soln = IkSolver.solveIK(t,manip)
                    if soln is not None:
                        soln["point"] = IkSolver.grasps[i].point(t)
                        soln["grasp"] = IkSolver.grasps[i].mat
                        soln["base"] = IkSolver.robot.GetTransform()
                        return soln
                        
    @staticmethod    
    def GetRaveIkSol(objName, parallel=False):       
        obj = IkSolver.env.GetKinBody(objName)
        pos = IkSolver.robot.GetTransform()
        IkSolver.grasps = GraspSet(objName, IkSolver.env)
        obj = IkSolver.env.GetKinBody(objName)
        targets = IkSolver.grasps.GetTargets(obj)
        IkSolver.targets = targets
        if len(targets) == 0:
            print "no grasps, cannot reach object",
            return None
            
        rsol, i = None, 0
        while rsol is None and not IkSolver.env.CheckCollision(IkSolver.robot.GetLink("base_link"),
                                                             IkSolver.env.GetKinBody("pod_lowres")):
            IkSolver.resetArms()
            if i == 0:
                st = time.time()
                pos[:3,3] = IkSolver.nn(rave.poseFromMatrix(obj.GetTransform())[np.newaxis])
            else:
                pos[:2,3] += IkSolver.computeMove(obj.GetTransform())  
                          
            IkSolver.robot.SetTransform(pos)
            rsol = IkSolver.GetIkSol(obj, targets, parallel)        

        return rsol
        
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
        
        
