import numpy as np
import openravepy as rave

def plotPose(env, toPlot):
    if toPlot.shape == (4,4):
        mat = toPlot
    else:
        mat = rave.matrixFromPose(toPlot)
    p1 = mat.dot([0,0,0,1])[:3]
    p2 = mat.dot([1,0,0,1])[:3]
    return env.drawarrow(p1,p2,linewidth=0.02)
    
def resetRobot(r):
    r.SetTransform(rave.matrixFromPose(np.array([1,0,0,0,-1.2,0,0.2])))
    resetArms(r)
    
def resetArms(r):
    r.SetDOFValues([0.54,-1.57, 1.57, 0.54],[22,27,15,34])
    
def randomObjPose(obj):
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
    
