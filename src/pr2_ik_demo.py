"""
+X is front, +Y is left, +Z is up
"""

import numpy as np
import openravepy as rave
from IK import *
import time

def plotPose(env, toPlot):
    if toPlot.shape == (4,4):
        mat = toPlot
    else:
        mat = rave.matrixFromPose(toPlot)
    p1 = mat.dot([0,0,0,1])[:3]
    p2 = mat.dot([1,0,0,1])[:3]
    return env.drawarrow(p1,p2,linewidth=0.02)
    
def resetRobot():
    r.SetTransform(rave.matrixFromPose(np.array([1,0,0,0,-1.1,0,0])))
    r.SetDOFValues([0.54,-1.57, 1.57, 0.54],[22,27,15,34])
    
rave.raveSetDebugLevel(rave.DebugLevel.Error)
e = rave.Environment()
e.Load("robots/pr2-beta-sim.robot.xml")
e.Load("../data/meshes/cubbyholes/pod_lowres.stl")
e.Load("../data/meshes/objects/dove_beauty_bar_centered.stl")
e.SetViewer("qtcoin")

r = e.GetRobots()[0]
resetRobot()

m = r.SetActiveManipulator("leftarm_torso")
shelf = e.GetBodies()[1]
obj = e.GetBodies()[2]

def randomObjPose():
    biny, binz = np.random.randint(3), np.random.randint(3)
    xlow, xhigh = -0.2, -0.43
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

ik = IkSolver(e)

print "press 'ENTER' to start, 'q' to quit"

while "q" not in str(raw_input("continue?" )):
    resetRobot()
    objPose = randomObjPose()
    obj.SetTransform(rave.matrixFromPose(objPose))
    
    st = time.time()
    rsol = ik.GetRaveIkSol("dove_beauty_bar_centered")
    pos = r.GetTransform()
    while rsol == [] and pos[0,3] < rave.poseFromMatrix(obj.GetTransform())[-3]:
        pos = r.GetTransform()
        pos[0,3] += 0.02
        r.SetTransform(pos)
        rsol = ik.GetRaveIkSol("dove_beauty_bar_centered")
    if rsol != []:
        print "found " + str(len(rsol)) + " sol in " + str(time.time()-st) + "s",
        for sol in rsol:
            m = r.SetActiveManipulator(sol["manip"])
            r.SetDOFValues(sol["joints"], m.GetArmIndices())
            raw_input("next sol? ")
    else:
        print "no IK sol found"
    
    
    
