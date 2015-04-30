import json
import os.path as osp
import openravepy as rave
import numpy as np

from utils import GRASP_DIR, OBJ_MESH_DIR

rave.raveSetDebugLevel(rave.DebugLevel.Error)
e = rave.Environment()

objects = ["dove_beauty_bar", "cheezit_big_original", "champion_copper_plus_spark_plug"]

for objName in objects:
    e.Load(osp.join(OBJ_MESH_DIR, objName + ".stl"))
    obj = e.GetKinBody(objName)
    com = obj.ComputeAABB().pos()
    def comp(g):
        posdict = g["gripper_pose"]["position"]
        pos = np.array([posdict["x"], posdict["y"], posdict["z"]])
        return np.linalg.norm(com - pos)
        
    grasps = json.load(open(osp.join(GRASP_DIR, objName + ".json")))
    
    grasps.sort(key=comp)
        
    json.dump(grasps, open(osp.join(GRASP_DIR, objName + "_sorted.json"), "w"))
