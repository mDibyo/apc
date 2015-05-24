import numpy as np
import openravepy as rave
from utils import NEW_WRISTS, NEW_SHELF, SHELF_X, SHELF_Y, SHELF_Z

handles = []

def plotPose(env, toPlot):
    if toPlot.shape == (4,4):
        mat = toPlot
    else:
        mat = rave.matrixFromPose(toPlot)
    p1 = mat.dot([0,     0,    0, 1])[:3]
    p2 = mat.dot([0.5,   0,    0, 1])[:3]
    p3 = mat.dot([0,   0.5,    0, 1])[:3]
    p4 = mat.dot([0,     0, 0.25, 1])[:3]
    handles.append( [env.drawarrow(p1,p2,linewidth=0.015),
            env.drawarrow(p1,p3,linewidth=0.007),
            env.drawarrow(p1,p4,linewidth=0.007)] )
    return handles
            
def clear(handles):
    [[p.Close() for p in h] for h in handles]
    handles = []
        
def resetRobot(r):
    if NEW_SHELF:
        r.SetTransform(rave.matrixFromPose(np.array([1,0,0,0,-1,0.2,0])))
    else:
        r.SetTransform(rave.matrixFromPose(np.array([1,0,0,0,-1.2,0.2,0])))
    resetArms(r)
    
def resetArms(r):
    r.SetDOFValues([0.548,-1.57, 1.57, 0.548],[22,27,15,34])
    
    
def randomObjPoses(objlist):
    biny, binz = np.random.randint(3), 1 #np.random.randint(4)
    xlow, xhigh = -0.15, -0.43
    ystep = 0.55/2
    zss = 0.23
    zsl = 0.27
    zvals = [0, zsl, zsl+zss, zsl+2*zss]
    
    poses = {}
    yrange = np.linspace(-0.1, 0.1, len(objlist) + 1)
    for i,obj in enumerate(objlist):
        size = obj.ComputeAABB().extents()
    
        yaw = (0.4*np.random.random()-0.2) + np.pi/2
        mat1 = rave.matrixFromAxisAngle(yaw * np.array([0,0,1]))
        #quat = np.array([theta,0,0,np.sqrt(1-theta**2)])
        quat = rave.quatFromRotationMatrix(mat1)
    
        x = np.random.random()*(xhigh-xlow+size[0]) + xlow   
        y = ystep * (biny-1) + yrange[i] + np.random.random()*(yrange[i+1] - yrange[i])
        z = zvals[binz] + 0.80 + size[2]
        poses[obj] = np.hstack([quat, [x,y,z]])
    return poses
        
def randomObjPose(obj):

    if NEW_SHELF:
        x = SHELF_X[np.random.randint(len(SHELF_X))] + 0.2*np.random.random()
        y = SHELF_Y[np.random.randint(len(SHELF_Y))]
        z = SHELF_Z[np.random.randint(len(SHELF_Z))] + obj.ComputeAABB().extents()[2]
        
        yaw = (0.4*np.random.random()-0.2) * 0.7071
        mat1 = rave.matrixFromAxisAngle(yaw * np.array([0,0,1]))
        mat2 = rave.matrixFromAxisAngle(np.pi/2 * np.array([0,1,0]))
        quat = rave.quatFromRotationMatrix(mat1.dot(mat2))  
        return np.hstack([ quat, [x,y,z] ])
    else:
        biny, binz = np.random.randint(3), np.random.randint(4)
        xlow, xhigh = -0.25, -0.43
        ystep = 0.55/2
        zss = 0.23
        zsl = 0.27
        zvals = [0, zsl, zsl+zss, zsl+2*zss]
        size = obj.ComputeAABB().extents()
        
        yaw = (0.4*np.random.random()-0.2) * 0.7071
        mat1 = rave.matrixFromAxisAngle(yaw * np.array([0,0,1]))
        quat = rave.quatFromRotationMatrix(mat1)
        
        x = np.random.random()*(xhigh-xlow+size[0]) + xlow
        y = ystep * (biny-1) + (0.2 * np.random.random() - 0.1)
        z = zvals[binz] + 0.80 + size[2]
        return np.hstack([quat, [x,y,z]])
    
