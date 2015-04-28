from __future__ import division

import threading
import multiprocessing as mp
import os.path as osp

import numpy as np

APC_DIRECTORY = osp.abspath(osp.join(__file__, "../.."))
DATA_DIRECTORY = osp.join(APC_DIRECTORY, "data")
MESH_DIRECTORY = osp.join(DATA_DIRECTORY, "meshes")
OBJ_MESH_DIR = osp.join(MESH_DIRECTORY, "objects", "clean")
SHELF_MESH_DIR = osp.join(MESH_DIRECTORY, "cubbyholes")
GRASP_DIR = osp.join(DATA_DIRECTORY, "grasps", "coll_free")

    
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
