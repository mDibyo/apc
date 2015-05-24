#!/usr/bin/env python
import shutil
import os
import subprocess
import h5py
from argparse import ArgumentParser
import numpy as np
import openravepy as rave

import cyni

import roslib
roslib.load_manifest('apc')
import rospy
from ros_utils import ROSNode
from camera import Carmine
import utils
from apc.srv import *

CARMINE_EXPOSURE=80


class APCCaptureSceneService(ROSNode):
    def __init__(self, output_directory=None, camera_pose_service_name=None, robot_state_service_name=None):
        super(APCCaptureSceneService, self).__init__('capture_scene')
        
        self.capture_scene_service = rospy.Service('capture_scene',
                                                   CaptureScene,
                                                   self.handle_capture_scene)

        self.camera_pose_service_name = camera_pose_service_name
        self.robot_state_service_name = robot_state_service_name
        
        if self.camera_pose_service_name is None:
            self.camera_pose_service_name = 'get_camera_pose'
        if self.robot_state_service_name is None:
            self.robot_state_service_name = 'get_latest_robot_state'
                
        self.camera_pose_client = rospy.ServiceProxy(self.camera_pose_service_name,
                                                     GetCameraPose)
        self.robot_state_client = rospy.ServiceProxy(self.robot_state_service_name,
                                                     GetLatestRobotState)
                                                                                                 
        self.perception_dir = utils.PERCEPTION_DIR if output_directory is None else self.perception_dir
        self.ssh_perception_dir = None
        if utils.COMPUTER != utils.PERCEPTION_COMPUTER:
            self.ssh_perception_dir = '{}:{}'.format(utils.PERCEPTION_COMPUTER, self.perception_dir)

        if os.path.exists(self.perception_dir):
            shutil.rmtree(self.perception_dir)
        os.mkdir(self.perception_dir)
            
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

    def shutter(self, color_filename, depth_filename, cloud_filename):
        self.carmine.capture_depth(depth_filename)
        self.carmine.stop_depth_stream()
        rospy.loginfo("Took depth")
        self.carmine.capture_color(color_filename)
        self.carmine.start_depth_stream()
        rospy.loginfo("Took carmine color")

        self.carmine.save_cloud(cloud_filename)
        #
        # with h5py.File(depth_filename, 'r') as df:
        #     depth_map = cyni.depthMapToImage(df["depth"][:])

    def handle_capture_scene(self, req):
        if req.request:
            path_base = os.path.join(self.perception_dir, req.request)
        else:
            path_base = os.path.join(self.perception_dir, "scene_{0}".format(self.scene_count))

        if os.path.exists(path_base):
            shutil.rmtree(path_base)
        os.mkdir(path_base)
        
        self.scene_count += 1

        rgbd_filename = os.path.join(path_base, "rgbd.jpg")
        depth_filename = os.path.join(path_base, "rgbd.h5")
        cloud_filename = os.path.join(path_base, "rgbd.pcd")
        
        self.shutter(rgbd_filename, depth_filename, cloud_filename)

        response = self.camera_pose_client("")
        state = self.robot_state_client("base")
        
        camera_to_robot = np.loadtxt(response.transform_mat_path)
        robot_to_world = rave.matrixFromPose(state.state_with_base.base_pose)
        camera_to_world = camera_to_robot.dot(robot_to_world)
        
        np.savetxt(os.path.join(path_base, "transform.txt"), camera_to_world)

        if self.ssh_perception_dir is not None:
            subprocess.check_call(['scp', '-r', path_base, self.ssh_perception_dir])

        return path_base
        
        
if __name__ == '__main__':
    description = "Capture rgbd from Carmine sensor and save"
    parser = ArgumentParser(description=description)
    parser.add_argument('-d', '--output-directory', default=None)
    parser.add_argument('-c', '--camera-pose-service', default=None)
    parser.add_argument('-r', '--robot-state-service', default=None)
    args = parser.parse_known_args()[0]

    with APCCaptureSceneService(output_directory=args.output_directory,
                                camera_pose_service_name=args.camera_pose_service,
                                robot_state_service_name=args.robot_state_service) as node:
        node.spin()

