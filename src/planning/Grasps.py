from __future__ import division
import os.path as osp
import json

import openravepy as rave
import numpy as np

from utils import GRASP_DIR

class Grasp():

    initial = rave.matrixFromQuat(np.array([np.sqrt(2)/2, 0, np.sqrt(2)/2, 0]))
    
    def __init__(self, quat, pos, width, parent):
        self.pose = np.hstack([quat, pos])
        self.mat = rave.matrixFromPose(self.pose)
        self.width = width
        self.parent = parent
            
    def prune(self, pose, objPose):
        return np.sign((pose - objPose)[-3])      
            
    def GetTargetPose(self, obj):       
        mat = obj.GetTransform().dot(self.mat).dot(Grasp.initial)
        pose = rave.poseFromMatrix(mat)
        objPose = rave.poseFromMatrix(obj.GetTransform())
        if self.parent.grasps.index(self) and self.prune(pose, objPose):
            return pose
        

class GraspSet():

    def __init__(self, objName):
        fname = osp.join(GRASP_DIR, objName + ".json")
        self.json = json.load(open(fname))
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
        targets = []
        bad = []
        for i,g in enumerate(self.grasps):
            t = g.GetTargetPose(obj)
            if t is not None:
                targets.append(t)
            else:
                bad.append(i)
        print "pruned",len(bad),"of",len(self.grasps),"grasps"
        self.grasps = [g for i,g in enumerate(self.grasps) if i not in bad]
        return targets
        
        

