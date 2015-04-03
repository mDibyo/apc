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
        
        X, Y = np.load(osp.join(DATA_DIRECTORY, "numIK", "poses.npy")), np.load(osp.join(DATA_DIRECTORY, "numIK", "joints.npy"))
        self.interpolator = interp(X, Y)
        
        
        self.ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(r, iktype=rave.IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
                self.ikmodel.autogenerate()
        
    def solveNumIk(self, targetPose, env, grasp):
        rob = env.GetRobot("pr2")
        man = rob.GetActiveManipulator()
        
        jointStart = self.interpolator(targetPose[np.newaxis] )[0]

        rob.SetDOFValues(jointStart, man.GetArmIndices())
        bodies = env.GetBodies()
        bodies.remove(env.GetKinBody("pr2"))
        
        target = targetPose
        rob.SetDOFValues(jointStart, man.GetArmIndices())

        def cost(joints, weights=[1,1,1,1,15,15,15], collW=100, ignoreShelf=True):
            rob.SetDOFValues(joints, man.GetArmIndices())  
            current = man.GetTransformPose()
            distCost = np.linalg.norm( (current - target)*np.array(weights))
            collCost = collW * sum([sum([1 if env.CheckCollision(link, obj) else 0 for obj in bodies]) for link in rob.GetLinks()])
            return distCost + collCost
                
        final, fmin, d = fmin_l_bfgs_b(cost, jointStart, maxfun=200, iprint=10, m=20, approx_grad=True, bounds=self.limits)
        rob.SetDOFValues(final, man.GetArmIndices()) 
        iks = {"joints": final, "pose":man.GetTransformPose(), "dist":fmin, "target":target}
        self.sols.put(iks)
        return iks
        
    def GetNumIkSol(self, objName):
        self.sols = mp.Queue()
        grasps = GraspSet(osp.join(DATA_DIRECTORY, "grasps", "dove_beauty_bar.json"))
        obj = self.env.GetKinBody(objName)
        g = grasps[0]
        return self.solveNumIk(g.GetTargetPose(obj), self.env.CloneSelf(rave.CloningOptions.Bodies), g)
        """processes = [mp.Process(target=self.solveNumIk,args=(g.GetTargetPose(obj), self.env.CloneSelf(rave.CloningOptions.Bodies), g)) for g in grasps]
        [p.start() for p in processes]
        [p.join() for p in processes]
        return [self.sols.get() for p in processes] """
        
        
    def GetRaveIkSol(self, objName):
        grasps = GraspSet(osp.join(DATA_DIRECTORY, "grasps", "dove_beauty_bar.json"))
        obj = self.env.GetKinBody(objName)
        targets = grasps[0].GetTargetPose(obj)
        sols = []
        for target in targets:
            ikparam = rave.IkParameterization(target, self.ikmodel.iktype)
            iks = self.env.GetRobot("pr2").GetActiveManipulator().FindIKSolution(ikparam, rave.IkFilterOptions.IgnoreEndEffectorEnvCollisions)
            if iks is not None:
                sols.append(iks)
        return sols
