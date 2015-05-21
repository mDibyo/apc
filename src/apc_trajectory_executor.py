#!/usr/bin/env python

from __future__ import division

import numpy as np

import roslib
roslib.load_manifest('apc')
import rospy
import tf
from std_msgs.msg import Header, Float32
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist, Point
#from pr2.manip import Manip
from pr2.arm import Arm

from apc.msg import ExecStatus, RobotStateBase
from ros_utils import ROSNode
from utils import LoopingThread 
from apc.srv import *

class APCTrajectoryExecutor(ROSNode):

    class ExecStatusContextManager(object):
        def __init__(self, topic):
            self.topic = topic

            self.status = ExecStatus.IDLE

            self.publisher = rospy.Publisher(topic, ExecStatus)
            self.publisher_thread = LoopingThread(target=self.publish,
                                                  rate=50)

        def __enter__(self):
            self.status = ExecStatus.BUSY
            return self

        def __exit__(self, exc_type, exc_val, exc_tb):
            self.status = ExecStatus.IDLE

        def publish(self):
            header = Header()
            header.stamp = rospy.Time.now()
            self.publisher.publish(ExecStatus(header, self.status))


    def __init__(self, joint_trajectories_topic, base_movement_topic, torso_height_topic, exec_status_topic, shelf_pose_file):
        super(APCTrajectoryExecutor, self).__init__('execute')

        self.joint_trajectories_topic = joint_trajectories_topic
        self.exec_status_topic = exec_status_topic
        self.base_movement_topic = base_movement_topic
        self.torso_height_topic = torso_height_topic
        self.exec_status = ExecStatus.IDLE

        #self.larm = Arm('left', default_speed=0.3)
        #self.larm_gripper_name = 'l_gripper_l_finger_joint'
        #self.rarm = Arm('right')
        # self.rarm_gripper_name = 'r_gripper_l_finger_joint'
        
        self.right = Arm("right", default_speed=0.15)
        self.left = Arm("left", default_speed=0.15)
        self.r_gripper_name = "r_gripper_l_finger_joint"
        self.l_gripper_name = "l_gripper_l_finger_joint"
        
        self.get_robot_state_client = rospy.ServiceProxy("/get_latest_robot_state", GetLatestRobotState)      

        self.joint_trajectories_subscriber = rospy.Subscriber(self.joint_trajectories_topic,
                                                              JointTrajectory,
                                                              self.execute_joint_trajectory)

        self.base_movement_subscriber = rospy.Subscriber(self.base_movement_topic,
                                                         Point,
                                                         self.move_base_linear)
                                                         
        self.torso_height_subscriber = rospy.Subscriber(self.torso_height_topic,
                                                        Float32,
                                                        self.move_torso)
        
        self.exec_status_publisher = rospy.Publisher(self.exec_status_topic, ExecStatus)
        self.exec_status_publisher_thread = LoopingThread(target=self.publish_exec_status,
                                                          rate=50)
        self.exec_status_publisher_thread.start()
        
        self.get_robot_state_client = rospy.ServiceProxy("get_latest_robot_state", GetLatestRobotState)     
        
        self.robot_start_pose = rave.poseFromMatrix(np.linalg.inv(np.loadtxt(shelf_pose_file)))
        self.robot_initial_odom = self.get_robot_state("").base_pos
    
    def get_robot_state(self, manip):
        try:
            res = self.get_robot_state_client(manip)
            return res.state_with_base
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
        
    def publish_exec_status(self):
        header = Header()
        header.stamp = rospy.Time.now()
        self.exec_status_publisher.publish(ExecStatus(header, self.exec_status))
     
    def get_robot_state(self, manip):
        try:
            res = self.get_robot_state_client(manip)
            return res.state_with_base
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e)) 
            
    def move_torso(self, target_height):
        while self.exec_status != ExecStatus.IDLE:
            rospy.sleep(0.5)   
            rospy.logwarn("not idle so waiting")
        try:
            arm = self.manip
            self.exec_status = ExecStatus.BUSY
            arm.move_torso_simple(target_height.data)
            self.exec_status = ExecStatus.IDLE
        except rospy.ROSException:
            self.exec_status = ExecStatus.ERROR
            
    def move_base_linear(self, base_target):
        while self.exec_status != ExecStatus.IDLE:
            rospy.sleep(0.5)   
            rospy.logwarn("not idle so waiting")         
        try:
            self.exec_status = ExecStatus.BUSY 
   
            cmd_vel_publisher = rospy.Publisher("base_controller/command", Twist)
            tf_listener = tf.TransformListener()
            
            tf_listener.waitForTransform("/base_footprint", "/odom_combined",
                                             rospy.Time(0), rospy.Duration(1))
                
            target = np.array([base_target.x, base_target.y, 0]) + self.robot_initial_odom                                  
            start = np.array(self.get_robot_state("").base_pos)

            pos, total_dist, dist_moved = start, np.linalg.norm(target - start), 0
            rate = rospy.Rate(10.)
                                                
            while np.linalg.norm(target - pos) > 1e-5 and dist_moved < total_dist:
                pos = self.get_robot_state("").base_pos
                                                 
                vel = Twist()
                v = (target - pos)
                v /= 20*np.linalg.norm(v)
                vel.linear.x, vel.linear.y = v[0], v[1]
                
                dist_moved = np.linalg.norm(pos - start)
                cmd_vel_publisher.publish(vel)                     
                rate.sleep()
                                                 
            self.exec_status = ExecStatus.IDLE
        except rospy.ROSException:
            self.exec_status = ExecStatus.ERROR
        
    def execute_joint_trajectory(self, trajectory):
        # TODO: Add handling for speed
        while self.exec_status != ExecStatus.IDLE:
            rospy.sleep(0.5)   
            rospy.logwarn("not idle so waiting")
        try:
            arm = self.manip
            """
            if joint_names[0][0] == 'l':
                arm = self.larm
            # elif joint_names[0][0] == 'r':
            #     arm = self.rarm
            else:  # TODO: Add handling for torso joint
                return """ 

            if len(trajectory.joint_names) > 1:
                print "move arm"
                # [1:] for no torso
                joint_names = trajectory.joint_names
                points = [point.positions for point in trajectory.points]
                
                self.exec_status = ExecStatus.BUSY
                arm.execute_joint_trajectory(points, block=True)               
                self.exec_status = ExecStatus.IDLE
         
            else:
                if trajectory.points[0].positions[0] == 0:
                    self.exec_status = ExecStatus.BUSY
                    arm.close_gripper(max_effort=60)
                    self.exec_status = ExecStatus.IDLE
                else:
                    self.exec_status = ExecStatus.BUSY
                    arm.open_gripper()
                    self.exec_status = ExecStatus.IDLE         
        except rospy.ROSException:
            self.exec_status = ExecStatus.ERROR

if __name__ == '__main__':
    executor = APCTrajectoryExecutor('joint_trajectories', 'base_movement', 'torso_height', 'exec_status', 'perception/shelf_finder/shelf_pose.txt')
    print "ready for execution"
    executor.spin()

