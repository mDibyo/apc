import time
import os.path as osp

import numpy as np
import openravepy as rave

from test_utils import *
from planning import IkSolver
from utils import SHELF_MESH_DIR, OBJ_MESH_DIR, OBJ_LIST, MODEL_DIR

rave.raveSetDebugLevel(rave.DebugLevel.Error)
e = rave.Environment()

#e.Load(osp.join(MODEL_DIR, "pr2-new-wrists.dae"))  
e.Load(osp.join("robots/pr2-beta-sim.robot.xml"))

e.Load(osp.join(SHELF_MESH_DIR, "pod_lowres.stl"))

#e.Load(osp.join(OBJ_MESH_DIR, OBJ_LIST[np.random.randint(len(OBJ_LIST))] + ".stl"))
e.Load(osp.join(OBJ_MESH_DIR, "cheezit_big_original.stl"))
#e.Load(osp.join(OBJ_MESH_DIR, "crayola_64_ct.stl"))

e.SetViewer("qtcoin")

r = e.GetRobot("pr2")
m = r.SetActiveManipulator("rightarm_torso")
shelf = e.GetKinBody("pod_lowres")
obj = e.GetBodies()[-1]

taskprob = rave.interfaces.TaskManipulation(r)
    
if __name__ == "__main__":
    ik = IkSolver(e)
    m.SetIkSolver(ik.ikmodel.iksolver)
    m = r.SetActiveManipulator("leftarm_torso")
    ik = IkSolver(e)
    m.SetIkSolver(ik.ikmodel.iksolver)
    
    print "press 'ENTER' to start, 'q' to quit"
    while "q" not in str(raw_input("continue?" )):
        resetRobot(r)
        objPose = randomObjPose(obj)
        obj.SetTransform(rave.matrixFromPose(objPose))
        
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

        
