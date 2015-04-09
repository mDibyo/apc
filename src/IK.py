#!/usr/bin/env python

import os.path as osp
import numpy as np
from scipy.optimize import fmin_l_bfgs_b
from scipy.interpolate import NearestNDInterpolator as interp
import openravepy as rave
import json 
import multiprocessing as mp
from joblib import Parallel, delayed
from utils import *
from IK_utils import *

class IkSolver(object):

    O = np.array([0,0,0,1])
    EPS = 1e-4
    env = None
    robot = None
    
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
    def solveIK(target, manipName, check=True):
        ikparam = rave.IkParameterization(target, IkSolver.ikmodel.iktype)
        if check:
            opt = rave.IkFilterOptions.CheckEnvCollisions
        else:
            opt = rave.IkFilterOptions.IgnoreEndEffectorEnvCollisions
            
        iksol = IkSolver.robot.GetManipulator(manipName).FindIKSolution(ikparam, opt)        
        if iksol is not None:
            if check:
                return {"joints": iksol, "manip": manipName,"target":target}
            else:
                IkSolver.robot.SetDOFValues(iksol, IkSolver.robot.GetManipulator(manipName).GetArmIndices())
                links = IkSolver.robot.GetLinks()
                numCols = sum([sum([1 if IkSolver.env.CheckCollision(l,o) else 0 for l in links]) for o in IkSolver.env.GetBodies()[1:]])
                if numCols == 0:
                    return {"joints": iksol, "manip": manipName,"target":target}
                else:
                    print "found solution:",iksol,"for",manipName,"but has",numCols,"collisions"

    @staticmethod
    def GetIkSol(objName, parallel):
        grasps = GraspSet(osp.join(DATA_DIRECTORY, "grasps", objName + ".json"))
        obj = IkSolver.env.GetKinBody(objName)
        targets = grasps.GetTargets(obj)

        manips = ["leftarm_torso", "rightarm_torso"]
        if obj.GetTransform()[1,3] < 0:
            manips.reverse()
            
        for manip in manips:
            for opt in [True]:
                IkSolver.resetArms()
                if parallel:
                    soln = runInParallel(IkSolver.solveIK, [[t,manip] for t in targets])
                    sols = soln
                else:
                    for t in targets:
                        soln = IkSolver.solveIK(t,manip,opt)
                        if soln is not None:
                            return soln
        
    @staticmethod    
    def GetRaveIkSol(objName, parallel=False):
        rsol = None
        obj = IkSolver.env.GetKinBody(objName)
        pos = IkSolver.robot.GetTransform()
        while rsol is None and not IkSolver.env.CheckCollision(IkSolver.robot.GetLink("base_link"),
                                                             IkSolver.env.GetKinBody("pod_lowres")):
            pos = IkSolver.robot.GetTransform()
            pos[0,3] += 0.05
            IkSolver.robot.SetTransform(pos)
            rsol = IkSolver.GetIkSol("dove_beauty_bar_centered", parallel)
            IkSolver.resetArms()
        return rsol
        
