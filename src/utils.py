from __future__ import division

import threading
import multiprocessing as mp
import os.path as osp
import time
from waiting import wait, TimeoutExpired
from copy import deepcopy

import numpy as np

APC_DIRECTORY = osp.abspath(osp.join(__file__, "../.."))
DATA_DIRECTORY = osp.join(APC_DIRECTORY, "data")
MESH_DIRECTORY = osp.join(DATA_DIRECTORY, "meshes")
SHELF_MESH_DIR = osp.join(MESH_DIRECTORY, "cubbyholes")
MODEL_DIR = osp.join(DATA_DIRECTORY, "models")

if 0:
    OBJ_MESH_DIR = osp.join(MESH_DIRECTORY, "objects", "good")
    GRASP_DIR = osp.join(DATA_DIRECTORY, "grasps", "coll_free")
    GRASP_TAG = "_coll_free.json"
else:
    OBJ_MESH_DIR = osp.join(MESH_DIRECTORY, "objects", "old")
    GRASP_DIR = osp.join(DATA_DIRECTORY, "grasps", "v1")
    GRASP_TAG = "_sorted.json"

OBJ_LIST = ["champion_copper_plus_spark_plug",      "kong_sitting_frog_dog_toy",
            "cheezit_big_original",                 "kygen_squeakin_eggs_plush_puppies",
            "crayola_64_ct",                        "mark_twain_huckleberry_finn",
            "dove_beauty_bar",                      "mead_index_cards",
            "dr_browns_bottle_brush",                "mommys_helper_outlet_plugs",
            "elmers_washable_no_run_school_glue",    "munchkin_white_hot_duck_bath_toy",
            "expo_dry_erase_board_eraser",           "one_with_nature_soap_dead_sea_mud",
            "feline_greenies_dental_treats",         "oreo_mega_stuf",
            "first_years_take_and_toss_straw_cups",  "paper_mate_12_count_mirado_black_warrior",
            "genuine_joe_plastic_stir_sticks",       "rollodex_mesh_collection_jumbo_pencil_cup",
            "highland_6539_self_stick_notes",        "safety_works_safety_glasses",
            "kong_air_dog_squeakair_tennis_ball",    "sharpie_accent_tank_style_highlighters",
            "kong_duck_dog_toy",                     "stanley_66_052"]
            
_trajopt_request_template = {
    "basic_info": {
        "n_steps": 10,
        "start_fixed": False
    },
    "costs": [{
        "type": "joint_vel",
            "params": {"coeffs": [2]}
        }, {
            "type": "collision",
            "params": {
                "coeffs": [20],
                "dist_pen": [0.02]
            }
        }],
        "constraints": [
            { "type": "joint",
              "params": { "vals": None }
            }
        ],
        "init_info": {
            "type": "straight_line",
            "endpoint": None
        }
    }
  
order_bin_pose = np.array([[ 0, 1, 0, -0.75],
                           [-1, 0, 0, -0.60],
                           [ 0, 0, 1,  0.30],
                           [ 0, 0, 0,     1]])
  
def trajopt_request_template():
    return deepcopy(_trajopt_request_template)  
    
def timed(func, args, max_time=30):
    q = mp.Queue()
    p = mp.Process(target=func, args=args+[q])
    try:
        p.start()
        wait(lambda: not q.empty(), timeout_seconds=max_time)
        p.terminate()
        if not q.empty():
            return q.get_nowait()
    except TimeoutExpired:
        return None
    
def runInParallel(func, argslist):
    q = mp.Queue()
    processes = []
    processes = [mp.Process(target=func,args=[q]+arg) for arg in argslist]
    [p.start() for p in processes]
        
    while not q.empty():
        res = q.get()
        if res is not None:
            [p.terminate() for p in processes]
            return res
            
def getch():
    import sys
    import tty
    import termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
    
    
    

class LoopingThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(),
                 kwargs={}, rate=1):
        super(LoopingThread, self).__init__(group=group, target=target,
                                            name=name, args=args,
                                            kwargs=kwargs)
        self.__target = target
        self.__args = args
        self.__kwargs = kwargs

        self._stop_event = threading.Event()
        self.rate = rate

    def run(self):
        import rospy

        if self.__target is not None:
            try:
                rate = rospy.Rate(self.rate)

                while not self._stop_event.is_set():
                    self.__target(*self.__args, **self.__kwargs)
                    rate.sleep()
            finally:
                del self.__target, self.__args, self.__kwargs

    def stop(self):
        self._stop_event.set()

        import time
        time.sleep(1/self.rate)
