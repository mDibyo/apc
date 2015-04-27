"""
+X is front, +Y is left, +Z is up
"""

import numpy as np
from scipy.optimize import fmin_l_bfgs_b
import openravepy as rave

EPS = 1e-4

def randomJoints(limits):
    joints = []
    for low,high in limits:
        joints.append(np.random.random()*(high - low) + low)
    return np.array(joints)
    
rave.raveSetDebugLevel(rave.DebugLevel.Error)
e = rave.Environment()
e.Load("robots/pr2-beta-sim.robot.xml")
#e.SetViewer("qtcoin")

r = e.GetRobots()[0]
m = r.SetActiveManipulator("leftarm_torso")

r.SetDOFValues([0.5],[22])

lower = [float(j.GetLimits()[0])+EPS for j in r.GetJoints(m.GetArmIndices())]
upper = [float(j.GetLimits()[1])-EPS for j in r.GetJoints(m.GetArmIndices())]
limits = zip(lower, upper)
jointStart = [np.mean(lim) for lim in limits]


N, i = 7, 0
X, Y = np.zeros([8**N, 7]), np.zeros([8**N, len(limits)])
for j0 in np.linspace(limits[0][0], limits[0][1], N):
    for j1 in np.linspace(limits[1][0], limits[1][1], N):
        for j2 in np.linspace(limits[2][0], limits[2][1], N):
            for j3 in np.linspace(limits[3][0], limits[3][1], N):
                for j4 in np.linspace(limits[4][0], limits[4][1], N):
                    for j5 in np.linspace(limits[5][0], limits[5][1], N):
                        for j6 in np.linspace(limits[6][0], limits[6][1], N):
                            for j7 in np.linspace(limits[7][0], limits[7][1], N):
                                j = [j0, j1, j2, j3, j4, j5, j6, j7]
                                r.SetDOFValues(j, m.GetArmIndices())
                                p = rave.poseFromMatrix(m.GetTransform())
                                
                                X[i,:], Y[i,:] = p, j
                                if not i % 100000 and i:
                                    print "sampled pose ",i," of ", 8**N
                                i += 1
"""
N = 1000000
X, Y = np.zeros([N, 7]), np.zeros([N, len(limits)])
weights = np.array([10,10,10,10,1,1,1])
for i in range(N):
    j = randomJoints(limits)
    r.SetDOFValues(j, m.GetArmIndices())
    
    p = rave.poseFromMatrix(m.GetTransform())
    X[i], Y[i] = p, j
    if not i % 10000 and i:
        print "sampled pose ",i," of ", N
"""    
np.save("poses.npy", X)
np.save("joints.npy", Y)
