import time
import os.path as osp

import numpy as np
import openravepy as rave
from pymongo import MongoClient

from test_utils import *
from planning import IkSolver
from utils import SHELF_MESH_DIR, OBJ_MESH_DIR, OBJ_LIST, MODEL_DIR, \
                  order_bin_pose, timed, NEW_WRISTS, NEW_SHELF, bin_pose

rave.raveSetDebugLevel(rave.DebugLevel.Error)
e = rave.Environment()

e.Load(osp.join(MODEL_DIR, "pr2-beta-static.zae"))
e.Load(osp.join(MODEL_DIR, "pod_lowres.kinbody.xml"))
e.Load(osp.join(OBJ_MESH_DIR, 'expo_dry_erase_board_eraser.stl'))

r = e.GetRobot("pr2")
shelf = e.GetKinBody("pod_lowres")
obj = e.GetBodies()[-1]

db = MongoClient()['apc']
db_collection = db['reachability_left']

if __name__ == "__main__":
    for manip in ["rightarm", "rightarm_torso", "leftarm", "leftarm_torso"]:
        m = r.SetActiveManipulator(manip)
        ik = IkSolver(e)
        m.SetIkSolver(ik.ikmodel.iksolver)
    
    i,N = 0, 1e5
    while i < N:
        resetRobot(r)
        e.Remove(obj)
        e.Load(osp.join(OBJ_MESH_DIR, OBJ_LIST[np.random.randint(len(OBJ_LIST))] + ".stl"))
        obj = e.GetBodies()[-1]; objName = obj.GetName()
        bin_name, objPose = randomObjPose(obj)
        obj.SetTransform(rave.matrixFromPose(objPose))
        
        st = time.time()
        sol = timed(ik.GetRaveIkSol, [obj.GetName(), False])
        if sol is None:
            sol = ik.GetDefaultGrasp(bin_name, obj.GetName())
            
        dur = time.time() - st
        i += 1
        print bin_name, obj.GetName(), i
        
        db_collection.insert_one({
            'object'      : obj.GetName(),
            'object_pose' : objPose.tolist(),
            'gripper_pose': sol["target"].tolist() if sol['joints'] is not None else None,
            'joint_angles': sol["joints"].tolist() if sol['joints'] is not None else None,
            'manip'       : sol["manip"] if sol is not None else None,
            'base_pos'    : sol["base"][:3,3].tolist() if sol['base'] is not None else None,
            'runtime'     : dur
        })

