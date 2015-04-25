import time

import numpy as np
import openravepy as rave

from pymongo import MongoClient

from test_utils import *
from planning import IkSolver


rave.raveSetDebugLevel(rave.DebugLevel.Error)
e = rave.Environment()
e.Load("robots/pr2-beta-sim.robot.xml")
e.Load("../data/meshes/cubbyholes/pod_lowres.stl")
e.Load("../data/meshes/objects/dove_beauty_bar_centered.stl")
#e.SetViewer("qtcoin")

r = e.GetRobots()[0]
resetRobot(r)

m = r.SetActiveManipulator("leftarm_torso")
shelf = e.GetBodies()[1]
obj = e.GetBodies()[2]

db = MongoClient()['apc']
db_collection = db['ik3']

if __name__ == "__main__":
    ik = IkSolver(e)

    i,N = 0, 1e5
    while i < N:
        resetRobot(r)
        objPose = randomObjPose()
        obj.SetTransform(rave.matrixFromPose(objPose))
        
        st = time.time()
        sol = ik.GetRaveIkSol("dove_beauty_bar_centered", parallel=False)
        dur = time.time() - st
        i += 1
        print i
        
        db_collection.insert_one({
            'pose': objPose.tolist(),
            'joint_angles': sol['joints'].tolist() if sol is not None else None,
            'manip': sol['manip'] if sol is not None else None,
            'base_pos': r.GetTransform()[:3,3].tolist(),
        })
        
    print time.time()-start

