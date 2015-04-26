"""
+X is front, +Y is left, +Z is up
"""
import os
import sys
import time

import numpy as np
import openravepy as rave
from planning import IK as ik
from pymongo import MongoClient

from message_wrappers import GraspWrapper

import IPython
from utils import OBJ_MESH_DIR as OBJECT_MESH_DIR, DATA_DIRECTORY, GRASP_DIR

PR2_MODEL_FILE = "robots/pr2-beta-sim.robot.xml"
CUBBYHOLE_MODEL_FILE = os.path.join(DATA_DIRECTORY, "meshes/cubbyholes/pod_lowres.stl") #"data/meshes/cubbyholes/pod_lowres.stl"

def plotPose(env, toPlot):
    if toPlot.shape == (4,4):
        mat = toPlot
    else:
        mat = rave.matrixFromPose(toPlot)
    p1 = mat.dot([0,0,0,1])[:3]
    p2 = mat.dot([1,0,0,1])[:3]
    return env.drawarrow(p1,p2,linewidth=0.02)
    
def resetRobot():
    r.SetTransform(rave.matrixFromPose(np.array([1,0,0,0,0,0,0])))
    resetArms()
    
def resetArms():
    r.SetDOFValues([0.54,-1.57, 1.57, 0.54],[22,27,15,34])
    
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
    z = zvals[binz] + 1.20 + size[2]
    return np.hstack([quat, [x,y,z]])
    
    
if __name__ == "__main__":
    # get sys input
    argc = len(sys.argv)
    if argc < 2:
        print 'Need to supply object name'
        exit(0)

    object_name = sys.argv[1]
    object_filename = os.path.join(OBJECT_MESH_DIR, object_name + '.stl')

    object_grasps_filename = os.path.join(GRASP_DIR,
                                          "{}.json".format(object_name))
    object_grasps_out_filename = os.path.join(DATA_DIRECTORY, 'grasps',
                                              "{}.json".format(object_name + '_coll_free'))
    auto_step = False
    
    # load openrave environment
    rave.raveSetDebugLevel(rave.DebugLevel.Error)
    e = rave.Environment()
    e.Load(PR2_MODEL_FILE)
    e.Load(object_filename)
    e.SetViewer("qtcoin")

    # set robot in front of the shelf
    r = e.GetRobots()[0]
    resetRobot()
    m = r.SetActiveManipulator("leftarm_torso")
    T_gripper_world = r.GetManipulator("leftarm_torso").GetTransform()

    # intialize object, grasps
    obj = e.GetBodies()[1]
    object_grasps = GraspWrapper.grasps_from_file(object_grasps_filename)

    # set transform between rviz and openrave
    R_fix = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
    T_fix = np.eye(4)
    T_fix[:3,:3] = R_fix
    T_fix[2,3] = -0.05

    # set OR viewer
    height = 1200
    width  = 1200
    cam_dist = 0.5
    viewer = e.GetViewer()
    viewer.SetSize(width, height)

    T_cam_obj = np.eye(4)
    R_cam_obj = np.array([[0,  0, 1],
                          [-1, 0, 0],
                          [0, -1, 0]])
    T_cam_obj[:3,:3] = R_cam_obj
    T_cam_obj[0,3] = -cam_dist

    # set view based on object
    T_obj_world = np.eye(4)
    T_cam_world = T_obj_world.dot(T_cam_obj)
    viewer.SetCamera(T_cam_world, cam_dist)

    # set only left robot gripper as visible
    for link in r.GetLinks():
        link_name = link.GetName()
        if link_name[0] == u'l' and link_name.find('gripper') != -1:
            link.SetVisible(True)
        else:
            link.SetVisible(False)

    # loop through grasps and set end effector to appropriate grasp
    ind = 0
    object_grasps_keep = []
    for grasp in object_grasps:
        # get grasp pose
        gripper_position = grasp.gripper_pose.position
        gripper_orientation = grasp.gripper_pose.orientation
        gripper_pose = np.array([gripper_orientation.w, gripper_orientation.x, gripper_orientation.y, gripper_orientation.z, gripper_position.x, gripper_position.y, gripper_position.z])

        # get grasp pose relative to object
        T_gripper_obj = rave.matrixFromPose(gripper_pose)
        T_obj_world = T_gripper_world.dot(T_fix).dot(np.linalg.inv(T_gripper_obj))

        # set robot position as inverse of object (for viewing purposes)
        T_robot_world = np.linalg.inv(T_obj_world)
        r.SetTransform(T_robot_world)

        # prune grasps in collision
        coll = e.CheckCollision(r, obj)
        if not coll:
            object_grasps_keep.append(grasp)
            if auto_step:
                time.sleep(0.5)
            else:
                user_input = 'x'
                while user_input != '':
                    user_input = raw_input()


    # resave the json file
    GraspWrapper.grasps_to_file(object_grasps_keep, object_grasps_out_filename)

