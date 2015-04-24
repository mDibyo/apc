"""
+X is front, +Y is left, +Z is up
"""

import numpy as np
import openravepy as rave
from IK import *
import time
from pymongo import MongoClient

def resetRobot():
    r.SetTransform(rave.matrixFromPose(np.array([1,0,0,0,-1.50,0,0.40])))
    resetArms()
    
def resetArms():
    r.SetDOFValues([0.54,-1.57, 1.57, 0.54],[22,27,15,34])
    
rave.raveSetDebugLevel(rave.DebugLevel.Error)
e = rave.Environment()
e.Load("robots/pr2-beta-sim.robot.xml")
e.Load("../data/meshes/cubbyholes/pod_lowres.stl")
e.Load("../data/meshes/objects/dove_beauty_bar_centered.stl")

r = e.GetRobots()[0]
resetRobot()

m = r.SetActiveManipulator("leftarm_torso")
shelf = e.GetBodies()[1]
obj = e.GetBodies()[2]
    
db = MongoClient()['apc']
db_collection = db['failures_ik3']
    
failures = np.load(osp.join(DATA_DIRECTORY, "simulated", "failures.npy"))
if __name__ == "__main__":
    ik = IkSolver(e)

    for i,objPose in enumerate(failures):
        obj.SetTransform(rave.matrixFromPose(objPose))
        
        resetRobot()
        robotPos = r.GetTransform()  
        sol = None       
        while sol is None and not IkSolver.env.CheckCollision(IkSolver.robot.GetLink("base_link"),
                                                             IkSolver.env.GetKinBody("pod_lowres")):
            for y in np.linspace(-0.8, 0.8, 80):
                robotPos[1,3] = y
                r.SetTransform(robotPos)                                       
                                                             
                sol = ik.GetIkSol("dove_beauty_bar_centered", False)
                if sol is not None:
                    db_collection.insert_one({
                        'pose': objPose.tolist(),
                        'joint_angles': sol['joints'].tolist(),
                        'manip': sol['manip'],
                        'base_pos': r.GetTransform()[:3,3].tolist(),
                    })
                    break;
                    
            robotPos[0,3] += 0.025;
        print i
        
