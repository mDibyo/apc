from __future__ import division
import os.path as osp
import json

import openravepy as rave
import numpy as np

from utils import GRASP_DIR

class Grasp():

    initial = rave.matrixFromQuat([np.sqrt(2)/2, 0, np.sqrt(2)/2, 0])

    T_fix = np.array([[-0.    , -0.    , -1.    , -0.    ],
                      [ 0.    ,  1.    ,  0.    ,  0.008 ],
                      [ 1.    ,  0.    ,  0.    , -0.0375],
                      [ 0.    ,  0.    ,  0.    ,  1.    ]])
    T_fix_inv = np.linalg.inv(T_fix)

    def __init__(self, quat, pos, width, parent):
        self.pose = np.hstack([quat, pos])
        self.mat = rave.matrixFromPose(self.pose)
        self.width = width
        self.parent = parent
            
    def prune(self, pose, objName):
        obj = self.parent.env.GetKinBody(objName)
        shelf = self.parent.env.GetBodies()[1]
        lipX = (shelf.ComputeAABB().pos() - shelf.ComputeAABB().extents())[0]
        dx = abs(lipX - obj.GetTransform()[0,3])
        validDir = self.point(pose).dot([-1,0,0]) > np.cos(np.arctan(0.13/dx))
        
        if not validDir:
            return False
        else:
            r = self.parent.env.GetRobots()[0]
            shelf = self.parent.env.GetBodies()[1]
            worldToGrip = r.GetManipulator("rightarm_torso").GetTransform()
            objToGrip, worldToObj = self.mat, obj.GetTransform()
            gripToObj, objToWorld = np.linalg.inv(objToGrip), np.linalg.inv(worldToObj)
            
            obj.SetTransform(worldToGrip.dot(Grasp.T_fix).dot(gripToObj))
            shelf.SetTransform(obj.GetTransform().dot(objToWorld))
            for link in r.GetManipulator("rightarm_torso").GetChildLinks():
                for ob in self.parent.env.GetBodies()[1:]:
                    if self.parent.env.CheckCollision(link, ob):
                        obj.SetTransform(worldToObj)
                        return False
            obj.SetTransform(worldToObj)
            return True
            
    def GetTargetPose(self, obj):       
        mat = obj.GetTransform().dot(self.mat).dot(Grasp.T_fix_inv)
        pose = rave.poseFromMatrix(mat)
        if self.parent.grasps.index(self) and self.prune(pose, obj.GetName()):
            
            return pose
        
    def point(self, target):
        mat = rave.matrixFromPose(target)
        O = mat.dot([0, 0, 0, 1])
        P = mat.dot([0, 0, 1, 1])
        return (O - P)[:3]
    

class GraspSet():

    def __init__(self, objName, env):
        fname = osp.join(GRASP_DIR, objName + "_sorted.json")
        self.json = json.load(open(fname))
        self.env = env.CloneSelf(rave.CloningOptions.Bodies)
        self.grasps = []
        self.targets = []
        for grasp in self.json:
            qdict, pdict = grasp["gripper_pose"]["orientation"], grasp["gripper_pose"]["position"]
            quat = np.array([qdict['w'], qdict['x'], qdict['y'], qdict['z']])
            pos = np.array([pdict['x'], pdict['y'], pdict['z']])
            width = grasp["gripper_width"]
            self.grasps.append(Grasp(quat, pos, width, self))
            
    def __getitem__(self, i):
        return self.grasps[i]

    def __len__(self):
        return len(self.grasps)
        
    def GetTargets(self, obj):
        targets, good, bad = [], [], []
        for i,g in enumerate(self.grasps):
            t = g.GetTargetPose(obj)
            if t is not None:
                targets.append(t)
                good.append(g)
            else:
                bad.append(i)
        print "discarded",len(bad),"of",len(self.grasps),"grasps"
        self.grasps = good
        return targets
        
        

