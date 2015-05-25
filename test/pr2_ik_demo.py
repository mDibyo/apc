import time
import os.path as osp

from test_utils import *
from planning import IkSolver
from utils import SHELF_MESH_DIR, OBJ_MESH_DIR, OBJ_LIST, MODEL_DIR, \
                  order_bin_pose, timed, NEW_WRISTS, NEW_SHELF, bin_pose

rave.raveSetDebugLevel(rave.DebugLevel.Error)
e = rave.Environment()

if NEW_WRISTS:
    e.Load(osp.join(MODEL_DIR, "pr2-new-wrists.dae"))  #"pr2-ow-box-hook.xml")) #
else:
    e.Load(osp.join("robots/pr2-beta-sim.robot.xml"))
    
if NEW_SHELF:
    e.Load(osp.join(MODEL_DIR, "cubbyhole_all_combined.kinbody.xml"))
    shelf = e.GetKinBody('cubbyhole_all_combined')
else:
    e.Load(osp.join(MODEL_DIR, "pod_lowres.kinbody.xml"))
    shelf = e.GetKinBody('pod_lowres')
    
e.Load(osp.join(MODEL_DIR, "order_bin.kinbody.xml"))
order_bin = e.GetKinBody("order_bin")
order_bin.SetTransform(order_bin_pose)   

#e.Load(osp.join(OBJ_MESH_DIR, OBJ_LIST[np.random.randint(len(OBJ_LIST))] + ".stl"))
#e.Load(osp.join(OBJ_MESH_DIR, "cheezit_big_original.stl"))
e.Load(osp.join(OBJ_MESH_DIR, 'expo_dry_erase_board_eraser.stl'))
#e.Load(osp.join(OBJ_MESH_DIR, "dove_beauty_bar.stl"))

e.SetViewer("qtcoin")

r = e.GetRobot("pr2")


obj = e.GetBodies()[-1]

#taskprob = rave.interfaces.TaskManipulation(r)
    
if __name__ == "__main__":
    for manip in ["leftarm", "leftarm_torso", "rightarm", "rightarm_torso"]:
        m = r.SetActiveManipulator(manip)
        ik = IkSolver(e)
        m.SetIkSolver(ik.ikmodel.iksolver)
    
    print "press 'ENTER' to start, 'q' to quit"
    while "q" not in str(raw_input("continue?" )):
        resetRobot(r)
        e.Remove(obj)
        e.Load(osp.join(OBJ_MESH_DIR, OBJ_LIST[np.random.randint(len(OBJ_LIST))] + ".stl"))
        obj = e.GetBodies()[-1]; objName = obj.GetName()
        objPose = randomObjPose(obj)
        obj.SetTransform(rave.matrixFromPose(objPose))
        
        st = time.time()
        #sol = ik.GetRaveIkSol(obj.GetName(), parallel=False)
        sol = timed(ik.GetRaveIkSol, [obj.GetName(), ], max_time=10)
        
        if sol is not None:
            print "found sol in " + str(time.time()-st) + "s",
            m = r.SetActiveManipulator(sol["manip"])
            r.SetDOFValues(sol["joints"], m.GetArmIndices())
            r.SetTransform(sol["base"])
            #taskprob.CloseFingers()

        else:
            print "no IK sol found in " + str(time.time()-st) + "s",
        
        
