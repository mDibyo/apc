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
    e.Load(osp.join(MODEL_DIR, "pr2-ow-box-hook-small.xml"))
    
if NEW_SHELF:
    e.Load(osp.join(MODEL_DIR, "cubbyhole_all_combined.kinbody.xml"))
    shelf = e.GetKinBody('cubbyhole_all_combined')
else:
    e.Load(osp.join(MODEL_DIR, "pod_lowres.kinbody.xml"))
    shelf = e.GetKinBody('pod_lowres')

#e.Load(osp.join(OBJ_MESH_DIR, OBJ_LIST[np.random.randint(len(OBJ_LIST))] + ".stl"))
#e.Load(osp.join(OBJ_MESH_DIR, "cheezit_big_original.stl"))
e.Load(osp.join(OBJ_MESH_DIR, 'expo_dry_erase_board_eraser.stl'))
#e.Load(osp.join(OBJ_MESH_DIR, "dove_beauty_bar.stl"))

e.SetViewer("qtcoin")

r = e.GetRobot("pr2")
obj = e.GetBodies()[-1]

def get_hook_plan(bin_name, target_object):
    plan = {}
    
    base, torso = ik.GetHookBaseTorso(bin_name)     
    r.SetDOFValues([torso], [r.GetJoint("torso_lift_joint").GetDOFIndex()])
    r.SetTransform(base)
    
    plan["binholder"] = ik.GetBinholderJoints(bin_name)      
    if plan["binholder"] is None:
        print "no pose found for binholder"
        return plan
                    
    r.SetDOFValues(plan["binholder"]["joints"], r.GetManipulator("leftarm_box").GetArmIndices())

    for which in ['prehook', 'insert', 'twist', 'out']:
        plan[which] = ik.GetHookJoints(bin_name, target_object, which)

    for k,v in plan.items():
        if v is None:
            print "no pose found for", k
            return plan
    return plan
            
if __name__ == "__main__":
    for manip in ["leftarm", "leftarm_torso", "leftarm_box", 
                  "rightarm", "rightarm_torso", "rightarm_hook", "rightarm_torso_hook"]:
        m = r.SetActiveManipulator(manip)
        ik = IkSolver(e)
        m.SetIkSolver(ik.ikmodel.iksolver)
    
    print "press 'ENTER' to start, 'q' to quit"
    while "q" not in str(raw_input("continue?" )):
        resetRobot(r)
        e.Remove(obj)
        e.Load(osp.join(OBJ_MESH_DIR, OBJ_LIST[np.random.randint(len(OBJ_LIST))] + ".stl"))
        obj = e.GetBodies()[-1]; objName = obj.GetName()
        binName, objPose = randomObjPose(obj)
        obj.SetTransform(rave.matrixFromPose(objPose))
        
        st = time.time()
        sol = get_hook_plan(binName, obj.GetName())
        
        for p in ['binholder', 'prehook', 'insert', 'twist', 'out']:
            if p in sol:
                j = sol[p]
                if j is not None:
                    print p,
                    r.SetDOFValues(j['joints'], r.GetManipulator(j['manip']).GetArmIndices())
                    raw_input()
                else:
                    break
            else:
                break
        
