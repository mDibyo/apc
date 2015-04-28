import time
import os.path as osp
from pymongo import MongoClient

import numpy as np
import openravepy as rave

from test_utils import *
from planning import IkSolver
from utils import SHELF_MESH_DIR, OBJ_MESH_DIR

rave.raveSetDebugLevel(rave.DebugLevel.Error)
e = rave.Environment()
e.Load("robots/pr2-beta-sim.robot.xml")
e.Load(osp.join(SHELF_MESH_DIR, "pod_lowres.stl"))
e.Load(osp.join(OBJ_MESH_DIR, "cheezit_big_original.stl"))
e.SetViewer("qtcoin")

r = e.GetRobots()[0]
m = r.SetActiveManipulator("leftarm_torso")
shelf = e.GetBodies()[1]
obj = e.GetBodies()[2]


db = MongoClient()['apc']
db_collection = db['unreachable']

if __name__ == "__main__":
    ik = IkSolver(e)

    i,N = 0, 1e5
    while i < N:
        resetRobot(r)
        objPose = randomObjPose(obj)
        obj.SetTransform(rave.matrixFromPose(objPose))
        
        st = time.time()
        sol = ik.GetRaveIkSol(obj.GetName(), parallel=False)
        dur = time.time() - st
        i += 1
        print i
        
        db_collection.insert_one({
            'gripper_pose': sol["target"],
            'joint_angles': sol["joints"],
            'manip'       : sol["manip"],
            'base_pos'    : r.GetTransform()[:3,3].tolist(),
            'runtime'     : dur
        })

