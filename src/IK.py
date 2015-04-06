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
        m = r.GetActiveManipulator()
        lower = [float(j.GetLimits()[0])+self.EPS for j in r.GetJoints(m.GetArmIndices())]
        upper = [float(j.GetLimits()[1])-self.EPS for j in r.GetJoints(m.GetArmIndices())]
        self.limits = zip(lower, upper)
        
        self.ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(r, iktype=rave.IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
                self.ikmodel.autogenerate()
                
    def solveIK(self, target, result):
        ikparam = rave.IkParameterization(target, self.ikmodel.iktype)
        iks = self.env.GetRobot("pr2").GetActiveManipulator().FindIKSolution(ikparam, rave.IkFilterOptions.CheckEnvCollisions) #IgnoreEndEffectorEnvCollisions)
        if iks is not None:
            result.append(iks)


    def GetRaveIkSol(self, objName):
        grasps = GraspSet(osp.join(DATA_DIRECTORY, "grasps", objName + ".json"))
        obj = self.env.GetKinBody(objName)
        targets = grasps.GetTargets(obj)
        sols = runInParallel(self.solveIK, [[t] for t in targets])
        return sols
        

        """
        sols = []
        for t in targets:
            iksol = self.solveIK(t)
            if iksol is not None:
                sols.append(iksol)
        return sols
        """
