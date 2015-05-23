#!/usr/bin/env python
import shutil
import os
import h5py
from argparse import ArgumentParser

import cyni

import roslib
roslib.load_manifest('apc')
import rospy
from ros_utils import ROSNode
from camera import Carmine
import utils
from apc.srv import CaptureScene, GetCameraPose

CARMINE_EXPOSURE=80


class APCCaptureSceneService(ROSNode):
    def __init__(self, output_directory=None, camera_pose_service_name=None):
        super(APCCaptureSceneService, self).__init__('capture_scene')
        
        self.capture_scene_service = rospy.Service('capture_scene',
                                                   CaptureScene,
                                                   self.handle_capture_scene)

        self.camera_pose_service_name = camera_pose_service_name
        if self.camera_pose_service_name is None:
            self.camera_pose_service_name = 'get_camera_pose'
        self.camera_pose_client = rospy.ServiceProxy(self.camera_pose_service_name,
                                                     GetCameraPose)
        self.output_dir = utils.PERCEPTION_DIR if output_directory is None else self.output_dir

        if os.path.exists(self.output_dir):
            shutil.rmtree(self.output_dir)
        os.mkdir(self.output_dir)
            
        self.initialize_cameras()
        self.scene_count = 0
        print "ready to capture images"
                                                  
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.carmine.close()
        
    def __enter__(self):
        return self
        
    def initialize_cameras(self):
        settings = {"exposure": CARMINE_EXPOSURE,
                    "gain": 100}
        cyni.initialize()
        device = cyni.getAnyDevice()
        if device is None:
            rospy.logfatal("No carmine found. ")
            rospy.logfatal("Please restart the program with the Carmine plugged in. ")
        else:
            self.carmine = Carmine(device, settings)
            self.carmine.initialize()
            self.carmine.start_color_stream()
            self.carmine.start_depth_stream()

    def shutter(self, highres_filename, color_filename, depth_filename, cloud_filename):
        self.carmine.capture_depth(depth_filename)
        self.carmine.stop_depth_stream()
        rospy.loginfo("Took depth")
        self.carmine.capture_color(color_filename)
        self.carmine.start_depth_stream()
        rospy.loginfo("Took carmine color")

        self.carmine.save_cloud(cloud_filename)

        with h5py.File(depth_filename, 'r') as df:
            depth_map = cyni.depthMapToImage(df["depth"][:])

    def handle_capture_scene(self, req):
        if req.request:
            path_base = os.path.join(self.output_dir, req.request)
        else:
            path_base = os.path.join(self.output_dir, "scene_{0}".format(self.scene_count))

        os.makedirs(path_base)
        self.scene_count += 1

        highres_filename = os.path.join(path_base, "rgb.jpg")
        rgbd_filename = os.path.join(path_base, "rgbd.jpg")
        depth_filename = os.path.join(path_base, "rgbd.h5")
        cloud_filename = os.path.join(path_base, "rgbd.pcd")
        
        self.shutter(highres_filename, rgbd_filename, depth_filename, cloud_filename)

        response = self.camera_pose_client("")
        mat = np.loadtxt(response.transform_mat_path)
        np.save_txt(path_base + "transform.txt")

        return path_base
        
        
if __name__ == '__main__':
    description = "Capture rgbd from Carmine sensor and save"
    parser = ArgumentParser(description=description)
    parser.add_argument('-d', '--output-directory', default=None)
    parser.add_argument('-c', '--camera-pose-service', default=None)
    args = parser.parse_known_args()[0]

    with APCCaptureSceneService(output_directory=args.output_directory,
                                camera_pose_service_name=args.camera_pose_service) as node:
        node.spin()

