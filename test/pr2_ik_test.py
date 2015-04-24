"""
+X is front, +Y is left, +Z is up
"""

import numpy as np
import openravepy as rave
from IK import *
import time
from pymongo import MongoClient


def plotPose(env, toPlot):
    if toPlot.shape == (4,4):
        mat = toPlot
    else:
        mat = rave.matrixFromPose(toPlot)
    p1 = mat.dot([0,0,0,1])[:3]
    p2 = mat.dot([1,0,0,1])[:3]
    return env.drawarrow(p1,p2,linewidth=0.02)
    
def resetRobot():
    r.SetTransform(rave.matrixFromPose(np.array([1,0,0,0,-1.2,0,0.2])))
    resetArms()
    
def resetArms():
    r.SetDOFValues([0.54,-1.57, 1.57, 0.54],[22,27,15,34])
    
rave.raveSetDebugLevel(rave.DebugLevel.Error)
e = rave.Environment()
e.Load("robots/pr2-beta-sim.robot.xml")
e.Load("../data/meshes/cubbyholes/pod_lowres.stl")
e.Load("../data/meshes/objects/dove_beauty_bar_centered.stl")
#e.SetViewer("qtcoin")

r = e.GetRobots()[0]
resetRobot()

m = r.SetActiveManipulator("leftarm_torso")
shelf = e.GetBodies()[1]
obj = e.GetBodies()[2]

def randomObjPose():
    biny, binz = np.random.randint(3), np.random.randint(4)
    xlow, xhigh = -0.15, -0.43
    ystep = 0.55/2
    zss = 0.23
    zsl = 0.27
    zvals = [0, zsl, zsl+zss, zsl+2*zss]
    size = obj.ComputeAABB().extents()
    theta = 0.4*np.random.random()-0.2
    quat = np.array([theta,0,0,np.sqrt(1-theta**2)])
    x = np.random.random()*(xhigh-xlow+size[1]) + xlow
    y = ystep * (biny-1)
    z = zvals[binz] + 0.80 + size[2]
    return np.hstack([quat, [x,y,z]])
    
    
db = MongoClient()['apc']
db_collection = db['ik3']

if __name__ == "__main__":
    ik = IkSolver(e)

    i,N = 0, 1e5
    while i < N:
        resetRobot()
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

