"""
+X is front, +Y is left, +Z is up
"""

import numpy as np
import openravepy as rave
from IK import *
import time

def plotPose(env, robot, pose):
    point = rave.matrixFromPose(pose).dot(O)[:3]
    return env.plot3(points=point,pointsize=15)
    
def plotPoint(env, point):
    px = point.copy() + [0.25,0,0]
    py = point.copy() + [0,0.25,0]
    pz = point.copy() + [0,0,0.25]
    linepts = np.vstack([px,point,py,point,pz])
    return linepts, e.drawlinestrip(points=linepts, linewidth=2.0)
    
rave.raveSetDebugLevel(rave.DebugLevel.Error)
e = rave.Environment()
e.Load("robots/pr2-beta-sim.robot.xml")
e.Load("../data/meshes/cubbyholes/pod_lowres.stl")
e.Load("../data/meshes/objects/dove_beauty_bar_centered.stl")
e.SetViewer("qtcoin")
r = e.GetRobots()[0]
r.SetDOFValues([0.54,-1.57],[22,27])
m = r.SetActiveManipulator("leftarm_torso")
initPose = m.GetTransformPose()
shelf = e.GetBodies()[1]
shelf.SetTransform(rave.matrixFromPose(np.array([1,0,0,0,1.3,0,0])))
obj = e.GetBodies()[2]
e.Remove(shelf)
aabb = shelf.ComputeAABB()
def randomObjPose():
    y = np.random.random() * aabb.extents()[1] + aabb.pos()[1]
    z = 0.8*np.random.random() + 0.78
    theta = 0.8*np.random.random()-0.4
    quat = np.array([theta,0,0,np.sqrt(1-theta**2)])
    return np.hstack([quat, [0.92, y, z]])

ik = IkSolver(e)

print "press 'ENTER' to start, 'q' to quit",

while "q" not in str(raw_input()):
    r.SetTransform(np.eye(4))
    objPose = randomObjPose()
    obj.SetTransform(rave.matrixFromPose(objPose))
    
    st = time.time()
    rsol = ik.GetRaveIkSol("dove_beauty_bar_centered")
    pos = r.GetTransform()
    while rsol == [] and pos[0,3] < obj.GetTransformPose()[-3]:
        pos = r.GetTransform()
        pos[0,3] += 0.02
        r.SetTransform(pos)
        rsol = ik.GetRaveIkSol("dove_beauty_bar_centered")
    if rsol != []:
        print "found sol in " + str(time.time()-st) + "s",
        r.SetDOFValues(rsol[0], m.GetArmIndices())
    else:
        print "no IK sol found"
    
    
    
