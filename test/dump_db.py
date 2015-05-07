import os.path as osp
from pymongo import MongoClient
import numpy as np
from utils import DATA_DIRECTORY

db_name = "reachability_NW_R"

db = MongoClient()["apc"][db_name]

base, gripper, fails = [], [], []
total_time = 0

for i,sol in enumerate(db.find()):
    if sol["gripper_pose"] is not None and sol["joint_angles"] is None:
        fail.append((i,sol))
    
    if sol["joint_angles"] is not None:
        base.append(sol["base_pos"])
        gripper.append(sol["gripper_pose"])
    
    total_time += sol["runtime"]
        
base = np.array(base)
gripper = np.array(gripper)

np.save(osp.join(DATA_DIRECTORY, "simulated", db_name + "_base.npy"), base)
np.save(osp.join(DATA_DIRECTORY, "simulated", db_name + "_gripper.npy"), gripper)

print len(base),"successes,",len(fails),"IK failures"
print "average",total_time/i,"sec per trial"
