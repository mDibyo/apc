"""
+X is front, +Y is left, +Z is up
"""

import numpy as np
import openravepy as rave
from numIK import NumIkSolver
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
e.Load("../data/meshes/objects/dove_beauty_bar.stl")
e.SetViewer("qtcoin")
r = e.GetRobots()[0]
r.SetDOFValues([0.54,-1.57],[22,27])
m = r.SetActiveManipulator("leftarm_torso")
initPose = m.GetTransformPose()
shelf = e.GetBodies()[1]
shelf.SetTransform(rave.matrixFromPose(np.array([1,0,0,0,1.1,0,0])))
obj = e.GetBodies()[2]
obj.SetTransform(rave.matrixFromPose(np.array([0.85,0,0,np.sqrt(1-0.85**2),0.72,0.2,1.1])))
r.SetTransform(rave.matrixFromPose(np.array([1,0,0,0,-0.5,0,0])))

irm = rave.databases.inversereachability.InverseReachabilityModel(robot=r)
irm.load()
quat = np.array([np.sqrt(2)/2, 0, np.sqrt(2)/2, 0])
Tgrasp = rave.matrixFromPose(np.hstack( [ quat, obj.ComputeAABB().pos() ] ) )
densityfn,samplerfn,bounds = irm.computeBaseDistribution(Tgrasp)

N = 100
goals = []
numfailures = 0
with r:
    poses,jointstate = samplerfn(N-len(goals))
    for pose in poses:
        r.SetTransform(pose)  
        r.SetDOFValues(*jointstate)
          
        if not m.CheckIndependentCollision(rave.CollisionReport()):
            q = m.FindIKSolution(Tgrasp,filteroptions=rave.IkFilterOptions.CheckEnvCollisions)
            if q is not None:
                values = r.GetDOFValues()
                values[m.GetArmIndices()] = q
                goals.append((Tgrasp,pose,values))
            elif m.FindIKSolution(Tgrasp,0) is None:
                numfailures += 1
