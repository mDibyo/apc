#!/usr/bin/env python

from __future__ import division

import roslib
roslib.load_manifest('apc')

import rospy
from ros_utils import ROSNode
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from apc.msg import BinWorkOrder, ExecStatus, RobotStateBase
from apc.srv import *
import openravepy as rave
import numpy as np

class APCController(ROSNode):
    def __init__(self, joint_trajectories_topic, exec_status_topic, shelf_pose_file):
        super(APCController, self).__init__('apc_controller')
        self.joint_trajectories_topic = joint_trajectories_topic
        self.exec_status_topic = exec_status_topic

        self._robot_exec_status = None

        self.get_motion_plan_client = rospy.ServiceProxy('get_motion_plan', GetMotionPlan)
                                                         
        self.get_robot_state_client = rospy.ServiceProxy("get_latest_robot_state", GetLatestRobotState)                                                 

        self.joint_trajectories_publisher = rospy.Publisher(self.joint_trajectories_topic, JointTrajectory)

        self.base_movement_publisher = rospy.Publisher("base_movement", Point)

        self.torso_height_publisher = rospy.Publisher("torso_height", Float32)

        self.point_head_client = rospy.ServiceProxy("point_head", PointHead)

        self.capture_scene_client = rospy.ServiceProxy("capture_scene", CaptureScene)

        self.look_at_bins_client = rospy.ServiceProxy("look_at_bins", LookAtBins)
        
        self.exec_status_subscriber = rospy.Subscriber(self.exec_status_topic,
                                                       ExecStatus,
                                                       self.record_exec_status)                                     
        self.robot_start_pose = rave.poseFromMatrix(np.linalg.inv(np.loadtxt(shelf_pose_file)))
        
        self.robot_initial_odom = self.get_robot_state("").base_pos

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
            self.base_movement_publisher.publish(motion_plan.base_target)    
            rospy.sleep(1.0)
            self.wait_for_idle_exec_status()

            # Move Torso
            self.torso_height_publisher.publish(Float32(motion_plan.torso_height.data))
            rospy.sleep(1.0)
            self.wait_for_idle_exec_status()

            # Move arms
            for joint_trajectory in motion_plan.trajectories:
                print joint_trajectory
                self.joint_trajectories_publisher.publish(joint_trajectory)
                #self.wait_for_busy_exec_status()
                rospy.sleep(1.0)
                self.wait_for_idle_exec_status()

            # Point Head
            self.point_head_client(motion_plan.head_direction)

            # Capture Image
            if motion_plan.capture_scene:
                self.capture_scene_client(motion_plan.capture_scene)

            rospy.logwarn("completed executing motion plan")
        
    def execute_work_order(self, work_order):
        try:
            rospy.logwarn("Here")
            res = self.get_motion_plan_client(work_order)
            print res
            rospy.logwarn("Here too")
            self.execute_motion_plan(res.motion_plan)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
            
    def get_robot_state(self, manip):
        try:
            res = self.get_robot_state_client(manip)
            return res.state_with_base
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

if __name__ == '__main__':

    controller = APCController('joint_trajectories', 'exec_status', 'perception/shelf_finder/shelf_pose.txt')
    rightjoints = controller.get_robot_state("rightarm_torso")
    leftjoints = controller.get_robot_state("leftarm_torso")
    
    start_pose = np.hstack([controller.robot_start_pose[:4], controller.robot_start_pose[-3:] + rightjoints.base_pos])
    
    work_order = BinWorkOrder('bin_G', 'all_combined', ['expo_dry_erase_board_eraser'],
                              'expo_dry_erase_board_eraser', rightjoints.joint_values, [0,1.57,0.2,1.57,-.8,2,-1.57,-1.57], start_pose, 'simple')
    controller.execute_work_order(work_order)
    
    
