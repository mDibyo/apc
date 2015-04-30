import time
import os.path as osp

import numpy as np
import openravepy as rave

from test_utils import *
from planning import IkSolver
from utils import SHELF_MESH_DIR, OBJ_MESH_DIR

rave.raveSetDebugLevel(rave.DebugLevel.Error)
e = rave.Environment()
e.Load("robots/pr2-beta-sim.robot.xml")
e.Load(osp.join(SHELF_MESH_DIR, "pod_lowres.stl"))
e.Load(osp.join(OBJ_MESH_DIR, "dove_beauty_bar.stl")) 
e.Load(osp.join(OBJ_MESH_DIR, "cheezit_big_original.stl")) #"champion_copper_plus_spark_plug.stl")) #
e.SetViewer("qtcoin")

r = e.GetRobots()[0]
m = r.SetActiveManipulator("leftarm_torso")
shelf = e.GetBodies()[1]
objs = e.GetBodies()[2:]

taskprob = rave.interfaces.TaskManipulation(r)
    
if __name__ == "__main__":
    ik = IkSolver(e)

    print "press 'ENTER' to start, 'q' to quit"
    while "q" not in str(raw_input("continue?" )):
        resetRobot(r)
        objPoses = randomObjPoses(objs)
        for ob,po in objPoses.items():
            ob.SetTransform(rave.matrixFromPose(po))
        
        obj = objs[-1]
        
        st = time.time()
        sol = ik.GetRaveIkSol(obj.GetName(), parallel=False)
        if sol is not None:
            print "found sol in " + str(time.time()-st) + "s",
            m = r.SetActiveManipulator(sol["manip"])
            r.SetDOFValues(sol["joints"], m.GetArmIndices())
            r.SetTransform(sol["base"])
            taskprob.CloseFingers()

        else:
            print "no IK sol found"

        
