import time
import os.path as osp
from pymongo import MongoClient

import numpy as np
import openravepy as rave

from test_utils import *
from planning import IkSolver
from utils import SHELF_MESH_DIR, OBJ_MESH_DIR, OBJ_LIST, MODEL_DIR

rave.raveSetDebugLevel(rave.DebugLevel.Error)
e = rave.Environment()

e.Load(osp.join(MODEL_DIR, "pr2-new-wrists.dae"))  
#e.Load(osp.join("robots/pr2-beta-sim.robot.xml"))

e.Load(osp.join(SHELF_MESH_DIR, "pod_lowres.stl"))

e.Load(osp.join(OBJ_MESH_DIR, "cheezit_big_original.stl"))


e.SetViewer("qtcoin")

r = e.GetRobot("pr2")
m = r.SetActiveManipulator("rightarm_torso")
shelf = e.GetKinBody("pod_lowres")
obj = e.GetBodies()[-1]



db = MongoClient()['apc']
db_collection = db['reachability2']

if __name__ == "__main__":
    ik = IkSolver(e)
    m.SetIkSolver(ik.ikmodel.iksolver)
    m = r.SetActiveManipulator("leftarm_torso")
    ik = IkSolver(e)
    m.SetIkSolver(ik.ikmodel.iksolver)
    
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
            'object'      : obj.GetName(),
            'object_pose' : objPose.tolist(),
            'gripper_pose': sol["target"].tolist() if sol is not None else None,
            'joint_angles': sol["joints"].tolist() if sol is not None else None,
            'manip'       : sol["manip"] if sol is not None else None,
            'base_pos'    : sol["base"][:3,3].tolist() if sol is not None else None,
            'runtime'     : dur
        })

