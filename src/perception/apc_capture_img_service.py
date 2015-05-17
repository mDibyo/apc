#!/usr/bin/env python
import shutil
import time
import os
import h5py

from camera import Carmine
import cyni

roslib.load_manifest('apc')
import rospy
from ros_utils import ROSNode
import tf

from apc.msg import RobotStateBase
from apc.srv import *

class APCCaptureSceneService(ROSNode):

    def __init__(self, update_rate, output_path):
        super(APCCaptureSceneService, self).__init__('capture')
        
        self.capture_scene_service = rospy.Service('capture_scene',
                                                   CaptureScene,
                                                   self.handle_capture_scene)
        self.output_dir = output_path

        if os.path.exists(self.output_dir):
            shutil.rmtree(self.output_dir)
        os.mkdir(self.output_dir)
            
        self.initialize_cameras()
        self.next_scene = 0

    def initialize_cameras(self):
        settings = {"exposure": CARMINE_EXPOSURE,
                    "gain": 100}
        cyni.initialize()
        device = cyni.getAnyDevice()
        if device is None:
            print "No carmine found."
            print "Please restart the program with the Carmine plugged in."
            self.set_test_set_name.config(state='disabled')
        else:
            self.carmine = Carmine(device, settings)
            self.carmine.initialize()
            self.carmine.start_color_stream()
            self.carmine.start_depth_stream()

    def shutter(self, highres_filename, color_filename, depth_filename):
        self.carmine.capture_depth(depth_filename)
        self.carmine.stop_depth_stream()
        print "Took depth"
        self.carmine.capture_color(color_filename)
        self.carmine.start_depth_stream()
        print "Took carmine color"

        with h5py.File(depth_filename, 'r') as df:
            depth_map = cyni.depthMapToImage(df["depth"][:])

    def handle_capture_scene(self):
        path_base = os.path.join(self.output_dir, "scene_{0}/".format(self.next_scene))
        os.makedirs(path_base)
        self.next_scene += 1

        highres_filename = path_base + "rgb.jpg"
        rgbd_filename = path_base + "rgbd.jpg"
        depth_filename = path_base + "rgbd.h5"
        
        self.shutter(highres_filename, rgbd_filename, depth_filename)
        return path_base

