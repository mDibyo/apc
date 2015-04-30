import json
import os.path as osp
import openravepy as rave

from utils import GRASP_DIR, OBJ_MESH_DIR

rave.raveSetDebugLevel(rave.DebugLevel.Error)
e = rave.Environment()

objects = ["dove_beauty_bar"]

for objName in objects:
    e.Load(osp.join(OBJ_MESH_DIR, objName + "_new.stl"))
    obj = e.GetKinBody(objName + "_new")
    
    offset = obj.ComputeAABB().pos()
    grasps = json.load(open(osp.join(GRASP_DIR, objName + ".json")))
    
    for g in grasps:
        g["gripper_pose"]["position"]["x"] -= offset[0]
        g["gripper_pose"]["position"]["y"] -= offset[1]
        g["gripper_pose"]["position"]["z"] -= offset[2]
        
    json.dump(grasps, open(osp.join(GRASP_DIR, objName + "_new.json"), "w"))
