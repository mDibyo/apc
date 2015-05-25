#!/usr/bin/env python

from __future__ import division
import os.path as osp
import json
import subprocess

import roslib
roslib.load_manifest('apc')

import rospy
from ros_utils import ROSNode
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Point
from apc.msg import BinWorkOrder, ExecStatus
from apc.srv import *
import utils
from strategy import parse_json
import time


class APCController(ROSNode):
    def __init__(self, joint_trajectories_topic, exec_status_topic):
        super(APCController, self).__init__('apc_controller')
        self.work_order_sequence = None

        self.perception_request_dir = utils.PERCEPTION_REQUEST_DIR
        self.ssh_perception_request_dir = None
        if utils.COMPUTER != utils.PERCEPTION_COMPUTER:
            self.ssh_perception_request_dir = '{}:{}'.format(utils.PERCEPTION_COMPUTER,
                                                             self.perception_request_dir)

        self.joint_trajectories_topic = joint_trajectories_topic
        self.exec_status_topic = exec_status_topic

        self._robot_exec_status = None


        self.get_robot_state_client = rospy.ServiceProxy("/apc/get_latest_robot_state", GetLatestRobotState)
        self.look_at_bins_client = rospy.ServiceProxy("/apc/look_at_bins", LookAtBins)
        
        self.get_grasp_plan_client = rospy.ServiceProxy('get_motion_plan_grasp', GetMotionPlan)
        self.get_hook_plan_client = rospy.ServiceProxy('get_motion_plan_hook', GetMotionPlan)
        
        self.joint_trajectories_publisher = rospy.Publisher(self.joint_trajectories_topic, JointTrajectory)
        self.base_movement_publisher = rospy.Publisher("base_movement", Point)
        self.torso_height_publisher = rospy.Publisher("torso_height", Float32)
        self.head_point_publisher = rospy.Publisher("head_point", Point)
        self.capture_scene_publisher = rospy.Publisher("capture_scene", String)

        self.exec_status_subscriber = rospy.Subscriber(self.exec_status_topic,
                                                       ExecStatus,
                                                       self.record_exec_status)                                     

    @property
    def robot_exec_status(self):
        return self._robot_exec_status

    @robot_exec_status.setter
    def robot_exec_status(self, exec_status):
        if exec_status == ExecStatus.ERROR:
            rospy.logerr("Error in robot execution status")
        self._robot_exec_status = exec_status

    def record_exec_status(self, status):
        self.robot_exec_status = status.status

    def wait_for_busy_exec_status(self):
        while self.robot_exec_status != ExecStatus.BUSY:
            rospy.sleep(0.1)
        rospy.loginfo("robot execution status busy")

    def wait_for_idle_exec_status(self):
        while self.robot_exec_status != ExecStatus.IDLE:
            rospy.sleep(0.1)
        rospy.loginfo("robot execution status idle")

    def execute_motion_plan(self, motion_plan):
        if motion_plan.strategy != "failed":
            # Move base
            if utils.MOVE_BASE:
                print "start base"
                self.base_movement_publisher.publish(motion_plan.base_target)    
                rospy.sleep(1.0)
                self.wait_for_idle_exec_status()
                print "end base"
            
            # Move Torso
            print "start torso"
            self.torso_height_publisher.publish(Float32(motion_plan.torso_height.data))
            rospy.sleep(1.0)
            self.wait_for_idle_exec_status()
            print "end torso"
            
            # Move arms
            print "start arms"
            for joint_trajectory in motion_plan.trajectories:
                print joint_trajectory
                self.joint_trajectories_publisher.publish(joint_trajectory)
                #self.wait_for_busy_exec_status()
                rospy.sleep(1.0)
            print "end arms"

            # Point Head
            if motion_plan.head_direction.x != 0:
                print "start head"
                self.head_point_publisher.publish(motion_plan.head_direction)
                rospy.sleep(1.0)
                self.wait_for_idle_exec_status()
                print "end head"
            
            # Capture Image
            if motion_plan.capture_scene:
                print "start scene"
                self.capture_scene_publisher.publish(motion_plan.capture_scene)
                rospy.sleep(1.0)
                self.wait_for_idle_exec_status()
                print "end scene"
                
            rospy.sleep(1.0)    
            rospy.logwarn("completed executing motion plan")
        
    def execute_work_order(self, work_order):
        try:
            rospy.logwarn("Here")
            if work_order.strategy == "grasp":
                res = self.get_grasp_plan_client(work_order)
            elif work_order.strategy == "hook":
                res = self.get_hook_plan_client(work_order)
            print res
            rospy.logwarn("Here too")
            self.execute_motion_plan(res.motion_plan)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def execute_perception(self, bin_name):
    
        perception_request = {
            "bin_name": bin_name,
            "objects": self.bin_contents[bin_name]["bin_contents"]
        }
        
        perception_file = '{}.json'.format(utils.datetime_now_string())

        with open(perception_file, 'w') as f:
            json.dump(perception_request, f)
        if self.ssh_perception_request_dir is not None:
            subprocess.check_call(['scp', perception_file, self.ssh_perception_request_dir])
            
    def get_robot_state(self, manip):
        try:
            res = self.get_robot_state_client(manip)
            return res.state_with_base
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
            
    def get_startup_sequence(self):
        return self.look_at_bins_client(['bin_G', 'bin_H', 'bin_I'])

if __name__ == '__main__':
    controller = APCController('joint_trajectories', 'exec_status')
    controller.work_order_sequence, controller.bin_contents = parse_json(osp.join(utils.JSON_DIR, "apc.json"))
    
    controller.execute_perception("bin_G")
    
    rightjoints = controller.get_robot_state("rightarm_torso")
    leftjoints = controller.get_robot_state("leftarm_torso")
    work_order = BinWorkOrder('bin_H',
                              'all_combined',
                               ["expo_dry_erase_board_eraser"],
                               "expo_dry_erase_board_eraser",
                               rightjoints.joint_values,
                                leftjoints.joint_values,
                                rightjoints.base_pose,
                                "grasp")
    controller.execute_work_order(work_order)
    
    """
    startup = controller.get_startup_sequence()
    for mp in startup.motion_plan_list:
        print mp
        controller.execute_motion_plan(mp)
     
    strategy = 'hook'
        
    for order in controller.work_order_sequence:
        controller.execute_perception(order["bin_name"])
        
        rightjoints = controller.get_robot_state("rightarm_torso")
        leftjoints = controller.get_robot_state("leftarm_torso")
        
        work_order = BinWorkOrder(order['bin_name'],
                                  'all_combined',
                                  order['bin_contents'],
                                  order['target_object'],
                                  rightjoints.joint_values,
                                  leftjoints.joint_values,
                                  rightjoints.base_pose,
                                  strategy)
        controller.execute_work_order(work_order)
<<<<<<< HEAD
    """
    
=======
>>>>>>> 50a220e69b8c0ab92dee278dc023b28836e5f973
