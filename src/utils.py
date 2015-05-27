from __future__ import division

import threading
import json
import multiprocessing as mp
import os.path as osp
import time
from waiting import wait, TimeoutExpired
from copy import deepcopy

import numpy as np

# Define robot configuration
NEW_WRISTS = True
NEW_SHELF = True
MOVE_BASE = False
FAKE = False
FIX_YAW = False

# Define computer configuration
APC_DIRECTORY = osp.abspath(osp.join(__file__, "../.."))
config = json.load(open(osp.join(APC_DIRECTORY, 'config.json')))

COMPUTER = config['computer']
PERCEPTION_COMPUTER = config['perception_computer']
ROS_COMPUTER = config['ROS_computer']

USER = config['user']
PERCEPTION_USER = config['perception_user']
ROS_USER = config['ROS_user']


# Define directories
DATA_DIRECTORY = osp.join(APC_DIRECTORY, "data")
MESH_DIRECTORY = osp.join(DATA_DIRECTORY, "meshes")
SHELF_MESH_DIR = osp.join(MESH_DIRECTORY, "cubbyholes")
MODEL_DIR = osp.join(DATA_DIRECTORY, "models")
JSON_DIR = osp.join(APC_DIRECTORY, "json")


# Perception
PERCEPTION_DIR = osp.join(DATA_DIRECTORY, "perception")
OBJECT_POSES_DIR = osp.join(DATA_DIRECTORY, "object_poses")
POSE_FIX_DIR = osp.join(DATA_DIRECTORY, 'meshes', 'pose_fix')
PERCEPTION_REQUEST_DIR = '/home/{}/perception_request_dir'.format(USER)
CAMERA_CALIBRATION_FILE = osp.join(DATA_DIRECTORY, 'perception', 'calibration.h5')
CAMERA_NAME = 'PR2'


# Define files
SHELF_POSE_FILE = '/home/nmishra/workspace/apc/src/perception/shelf_finder/shelf_pose.txt'




# Define grasp configuration
grasps_fn = "latest"
GRASP_DIR = osp.join(DATA_DIRECTORY, "grasps", grasps_fn)
if grasps_fn == "coll_free":
    OBJ_MESH_DIR = osp.join(MESH_DIRECTORY, "objects", "good")
    GRASP_TAG = "_coll_free.json"
elif grasps_fn == "v1":
    OBJ_MESH_DIR = osp.join(MESH_DIRECTORY, "objects", "old")
    GRASP_TAG = "_sorted.json"
elif grasps_fn == "tmp_grasps":
    OBJ_MESH_DIR = osp.join(MESH_DIRECTORY, "objects", "good")
    GRASP_TAG = "_sorted.json"
elif grasps_fn == "tmp_grasps":
    OBJ_MESH_DIR = osp.join(MESH_DIRECTORY, "objects", "good")
    GRASP_TAG = "_sorted.json"
elif grasps_fn == "latest":
    OBJ_MESH_DIR = osp.join(MESH_DIRECTORY, "objects", "apc_models")
    GRASP_TAG = "_coll_free.json"


# Define shelf cubbyhole configuration
inch_to_m = 0.0254
if NEW_SHELF:
    SHELF_Y = [11*.0254, 0, -11*.0254]
    SHELF_Z = [37*0.0254, 28*0.0254, ]
    SHELF_X = [-17*0.0254]
    bin_pose = {}
    BINS = [
        "bin_G",
        "bin_H",
        "bin_I",
        "bin_J",
        "bin_K",
        "bin_L"
    ]
    i = 0

    for x in SHELF_X:
        for z in SHELF_Z:
            for y in SHELF_Y:
                bin_pose[BINS[i]] = np.array([1, 0, 0, 0, x, y, z])
                i += 1

    _bin_dims_in_inches = {
        "bin_G": [-19, -1, 6, 17, 37.5, 47],
        "bin_H": [-19, -1, -6, 6, 37.5, 47],
        "bin_I": [-19, -1, -17, -3, 37.5, 47],
        "bin_J": [-19, -1, 6, 17, 28, 34.5],
        "bin_K": [-19, -1, -6, 6, 28, 34.5],
        "bin_L": [-19, -1, -17, -6, 28, 34.5],
    }
    # bin_dims = [x * inch_to_m for x in _bin_dims_in_inches]
    bin_dims = {}
    for bin_name, dims in _bin_dims_in_inches.iteritems():
        bin_dims[bin_name] = [x*inch_to_m for x in dims]

else:
    BINS = [
        "bin_A",
        "bin_B",
        "bin_C",
        "bin_D",
        "bin_E",
        "bin_F"
        "bin_G",
        "bin_H",
        "bin_I",
        "bin_J",
        "bin_K",
        "bin_L"
    ]


# Define object ease
obj_ease = {
    "champion_copper_plus_spark_plug":              3,
    "kong_sitting_frog_dog_toy":                    4,
    "cheezit_big_original":                         1,
    "kyjen_squeakin_eggs_plush_puppies":            4,
    "crayola_64_ct":                                1,
    "mark_twain_huckleberry_finn":                  4,
    #"dove_beauty_bar":                              1,
    "mead_index_cards":                             2,
    "dr_browns_bottle_brush":                       3,
    "mommys_helper_outlet_plugs":                   3,
    "elmers_washable_no_run_school_glue":           2,
    "munchkin_white_hot_duck_bath_toy":             4,
    "expo_dry_erase_board_eraser":                  2,
    #"one_with_nature_soap_dead_sea_mud":            2,
    "feline_greenies_dental_treats":                4,
    "oreo_mega_stuf":                               1,
    "first_years_take_and_toss_straw_cups":         3,
    "paper_mate_12_count_mirado_black_warrior":     3,
    "genuine_joe_plastic_stir_sticks":              1,
    "rollodex_mesh_collection_jumbo_pencil_cup":    5,
    "highland_6539_self_stick_notes":               2,
    "safety_works_safety_glasses":                  4,
    "kong_air_dog_squeakair_tennis_ball":           4,
    "sharpie_accent_tank_style_highlighters":       3,
    "kong_duck_dog_toy":                            4,
    "stanley_66_052":                               3,
    "laugh_out_loud_joke_book":                     4
}

OBJ_LIST = obj_ease.keys()


# Define order bin configuration
order_bin_pose = np.array([[ 1, 0, 0, -0.75],
                           [ 0, 1, 0,  0.60],
                           [ 0, 0, 1,  0.10],
                           [ 0, 0, 0,     1]])


# Trajopt request
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

def trajopt_request_template():
    return deepcopy(_trajopt_request_template)



def timed(func, args, max_time=20):
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


def datetime_now_string():
    import datetime
    return datetime.datetime.now().strftime("%Y%m%d%H%M%S%f")


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
