import time
import os.path as osp


import numpy as np
import openravepy as rave

from test_utils import *
from planning import IkSolver
from utils import SHELF_MESH_DIR, OBJ_MESH_DIR, OBJ_LIST, MODEL_DIR, \
                  order_bin_pose, timed

rave.raveSetDebugLevel(rave.DebugLevel.Error)
e = rave.Environment()

e.Load(osp.join(MODEL_DIR, "pr2-new-wrists.dae"))  
#e.Load(osp.join("robots/pr2-beta-sim.robot.xml"))

e.Load(osp.join(MODEL_DIR, "pod_lowres.kinbody.xml"))
e.Load(osp.join(MODEL_DIR, "order_bin.kinbody.xml"))

#e.Load(osp.join(OBJ_MESH_DIR, OBJ_LIST[np.random.randint(len(OBJ_LIST))] + ".stl"))
e.Load(osp.join(OBJ_MESH_DIR, "cheezit_big_original.stl"))
#e.Load(osp.join(OBJ_MESH_DIR, 'expo_dry_erase_board_eraser.stl'))


e.SetViewer("qtcoin")

r = e.GetRobot("pr2")
shelf = e.GetKinBody("pod_lowres")
order_bin = e.GetKinBody("order_bin")
order_bin.SetTransform(order_bin_pose)
obj = e.GetBodies()[-1]

taskprob = rave.interfaces.TaskManipulation(r)
    
if __name__ == "__main__":
    for manip in ["rightarm", "rightarm_torso", "leftarm", "leftarm_torso"]:
        m = r.SetActiveManipulator(manip)
        ik = IkSolver(e)
        m.SetIkSolver(ik.ikmodel.iksolver)
    
    print "press 'ENTER' to start, 'q' to quit"
    while "q" not in str(raw_input("continue?" )):
        resetRobot(r)
        objPose = randomObjPose(obj)
        obj.SetTransform(rave.matrixFromPose(objPose))
        
        st = time.time()
        #sol = ik.GetRaveIkSol(obj.GetName(), parallel=False)
        sol = timed(ik.GetRaveIkSol, [obj.GetName(), False])
        
        if sol is not None:
            print "found sol in " + str(time.time()-st) + "s",
            m = r.SetActiveManipulator(sol["manip"])
            r.SetDOFValues(sol["joints"], m.GetArmIndices())
            r.SetTransform(sol["base"])
            taskprob.CloseFingers()

        else:
            print "no IK sol found in " + str(time.time()-st) + "s",

        
