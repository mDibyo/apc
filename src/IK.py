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
    
    def resetArms(self):
        self.robot.SetDOFValues([0.54,-1.57, 1.57, 0.54],[22,27,15,34])
    

    def __init__(self, env):
        self.env = env
        r = env.GetRobot("pr2")
        self.robot = r
        m = r.GetActiveManipulator()
        lower = [float(j.GetLimits()[0])+self.EPS for j in r.GetJoints(m.GetArmIndices())]
        upper = [float(j.GetLimits()[1])-self.EPS for j in r.GetJoints(m.GetArmIndices())]
        self.limits = zip(lower, upper)
        
        self.ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(r, iktype=rave.IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
                self.ikmodel.autogenerate()
                
    def solveIK(self, target, manipName, out):
        ikparam = rave.IkParameterization(target, self.ikmodel.iktype)
        iksol = self.env.GetRobot("pr2").GetManipulator(manipName).FindIKSolution(ikparam, rave.IkFilterOptions.IgnoreEndEffectorEnvCollisions)
        if iksol is not None:
            self.robot.SetDOFValues(iksol, self.robot.GetManipulator(manipName).GetArmIndices())
            links = self.robot.GetLinks()
            numCols = sum([sum([1 if self.env.CheckCollision(l,o) else 0 for l in links]) for o in self.env.GetBodies()[1:]])
            if numCols == 0:
                out.append({"joints": iksol, "manip": manipName})
                return {"joints": iksol, "manip": manipName}

    def GetIkSol(self, objName, parallel):
        grasps = GraspSet(osp.join(DATA_DIRECTORY, "grasps", objName + ".json"))
        obj = self.env.GetKinBody(objName)
        targets = grasps.GetTargets(obj)
        sols = []
        manips = ["leftarm_torso", "rightarm_torso"]
        if obj.GetTransform()[1,3] < 0:
            manips.reverse()
            
        for manip in manips:
            if parallel:
                soln = runInParallel(self.solveIK, [[t,manip] for t in targets])
                sols.extend(soln)
            else:
                for t in targets:
                    soln = self.solveIK(t,manip,[])
                    if soln is not None:
                        sols.append(soln)
            if sols != []:
                break
        return sols
        
    def GetRaveIkSol(self, objName, parallel=False):
        rsol = []
        obj = self.env.GetKinBody(objName)
        pos = self.robot.GetTransform()
        while rsol == [] and pos[0,3] < rave.poseFromMatrix(obj.GetTransform())[-3]:
            pos = self.robot.GetTransform()
            pos[0,3] += 0.05
            self.robot.SetTransform(pos)
            rsol = self.GetIkSol("dove_beauty_bar_centered", parallel)
            self.resetArms()
        return rsol
        
