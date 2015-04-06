#!/usr/bin/env python

import os.path as osp
import numpy as np
from scipy.optimize import fmin_l_bfgs_b
from scipy.interpolate import NearestNDInterpolator as interp
import openravepy as rave
import json 
import multiprocessing as mp
from utils import *
from IK_utils import *

class IkSolver(object):

    O = np.array([0,0,0,1])
    EPS = 1e-4

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

    def GetRaveIkSol(self, objName):
        grasps = GraspSet(osp.join(DATA_DIRECTORY, "grasps", objName + ".json"))
        obj = self.env.GetKinBody(objName)
        targets = grasps.GetTargets(obj)
        sols = []
        manips = ["leftarm_torso", "rightarm_torso"]
        if obj.GetTransform()[1,3] < 0:
            manips.reverse()
            
        for manip in manips:
            sols.extend(runInParallel(self.solveIK, [[t,manip] for t in targets]))
            print '',
            if sols != []:
                break
        return sols
        
