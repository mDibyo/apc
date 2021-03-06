#!/usr/bin/env python

from __future__ import division

import numpy as np
import roslib
roslib.load_manifest('apc')
import rospy
import tf
from std_msgs.msg import Header, Float32, String
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist, Point
#from pr2.manip import Manip
from pr2.arm import Arm
import openravepy as rave


from apc.msg import ExecStatus, RobotStateBase
from ros_utils import ROSNode
from utils import LoopingThread, FIX_YAW
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


    def __init__(self, joint_trajectories_topic, exec_status_topic):
        super(APCTrajectoryExecutor, self).__init__('execute')

        self.joint_trajectories_topic = joint_trajectories_topic
        self.exec_status_topic = exec_status_topic

        self.exec_status = ExecStatus.IDLE

        #self.larm = Arm('left', default_speed=0.3)
        #self.larm_gripper_name = 'l_gripper_l_finger_joint'
        #self.rarm = Arm('right')
        # self.rarm_gripper_name = 'r_gripper_l_finger_joint'
        
        self.right = Arm("right", default_speed=0.20)
        self.left = Arm("left", default_speed=0.20)
        self.r_gripper_name = "r_gripper_l_finger_joint"
        self.l_gripper_name = "l_gripper_l_finger_joint"
        self.left.close_gripper()
        
        self.get_robot_state_client = rospy.ServiceProxy("/get_latest_robot_state", GetLatestRobotState)      

        self.joint_trajectories_subscriber = rospy.Subscriber(self.joint_trajectories_topic,
                                                              JointTrajectory,
                                                              self.execute_joint_trajectory)

        self.base_movement_subscriber = rospy.Subscriber('base_movement',
                                                         Point,
                                                         self.move_base_linear)
                                                         
        self.torso_height_subscriber = rospy.Subscriber('torso_height',
                                                        Float32,
                                                        self.move_torso)
                                                        
        self.head_point_subscriber = rospy.Subscriber('head_point',
                                                      Point,
                                                      self.point_head_dir)
                                                      
        self.head_point_subscriber = rospy.Subscriber('capture_scene',
                                                      String,
                                                      self.capture_scene)
                                                      
        self.exec_status_publisher = rospy.Publisher(self.exec_status_topic, ExecStatus)
        self.exec_status_publisher_thread = LoopingThread(target=self.publish_exec_status,
                                                          rate=50)
        self.exec_status_publisher_thread.start()
        
        self.get_robot_state_client = rospy.ServiceProxy("/apc/get_latest_robot_state", GetLatestRobotState)     
        
        self.cmd_vel_publisher = rospy.Publisher("base_controller/command", Twist)
        
        self.point_head_client = rospy.ServiceProxy("/apc/point_head", PointHead)
        
        self.capture_scene_client = rospy.ServiceProxy("/apc/capture_scene", CaptureScene)
        
    def capture_scene(self, path):
        print "capturing scene"
        #try:
        self.exec_status = ExecStatus.BUSY
        self.capture_scene_client(path.data)
        self.exec_status = ExecStatus.IDLE
        #except rospy.ROSException:
        #    self.exec_status = ExecStatus.ERROR
        
    def point_head_dir(self, direction):
        print "pointing head at",direction
        #try:
        self.exec_status = ExecStatus.BUSY
        self.point_head_client(direction)
        self.exec_status = ExecStatus.IDLE
        #except rospy.ROSException:
        #    self.exec_status = ExecStatus.ERROR
            
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
        print "moving torso",target_height
        while self.exec_status != ExecStatus.IDLE:
            rospy.sleep(0.5)   
            rospy.logwarn("not idle so waiting")
        #try:
        arm = self.right
        self.exec_status = ExecStatus.BUSY
        arm.move_torso_simple(0.01 + target_height.data)
        self.exec_status = ExecStatus.IDLE
        #except rospy.ROSException:
        #    self.exec_status = ExecStatus.ERROR

     
    def move_base(self, base_pos):
        while self.exec_status != ExecStatus.IDLE:
            rospy.sleep(0.5)   
            rospy.logwarn("not idle so waiting")
        try:
            arm = self.right
            self.exec_status = ExecStatus.BUSY
            start = np.array(self.get_robot_state("base").base_pose[-3:])
            disp = np.array([base_pos[0] - start[0], base_pose[1] - start[1]])
            
            arm.move_base(disp)
            self.exec_status = ExecStatus.IDLE
        except rospy.ROSException:
            self.exec_status = ExecStatus.ERROR

         
    def move_base_linear(self, base_target):
        while self.exec_status != ExecStatus.IDLE:
            rospy.sleep(0.5)   
            rospy.logwarn("not idle so waiting")         
        #try:
        self.exec_status = ExecStatus.BUSY 
        
        vel = Twist()
        self.cmd_vel_publisher.publish(vel)    
        
        rospy.logwarn(base_target)
        rate = rospy.Rate(10.)
        tf_listener = tf.TransformListener()
        
        tf_listener.waitForTransform("/base_footprint", "/odom_combined",
                                         rospy.Time(0), rospy.Duration(1))
           
        if FIX_YAW:   
           
            start_quat = self.get_robot_state("base").base_pose[:4]
            angle = start_quat[-1]
            
            while abs(angle) > 0.0005:
                angle = self.get_robot_state("base").base_pose[3]
                rospy.loginfo("yaw: " + str(angle))
                vel = Twist()
                vel.angular.z = -0.025*np.sign(angle)
                self.cmd_vel_publisher.publish(vel)
                rate.sleep()
            
        start = self.get_robot_state("base").base_pose[-3:]
        end = np.array([base_target.x, base_target.y, 0])
        
        v, dist_moved = end - start, 0                    
        dist_to_move = np.linalg.norm(v)
        
        
        
        if dist_to_move > 0.001:
        
            v /= 20*np.linalg.norm(v)   
            print v
                       
            while dist_moved < dist_to_move:
                dist_moved = np.linalg.norm( np.array(self.get_robot_state("base").base_pose[-3:]) - start)
                rospy.loginfo(dist_moved)                           
                vel = Twist()
                vel.linear.x, vel.linear.y = v[0], v[1]
                
                self.cmd_vel_publisher.publish(vel)                     
                rate.sleep()
        else:
             rospy.logwarn("close enough")
                                                 
        self.exec_status = ExecStatus.IDLE
        #except rospy.ROSException:
        #    self.exec_status = ExecStatus.ERROR
        
    def execute_joint_trajectory(self, trajectory):
        # TODO: Add handling for speed
        while self.exec_status != ExecStatus.IDLE:
            rospy.sleep(0.5)   
            rospy.logwarn("not idle so waiting")
        try:
            if trajectory.joint_names[0][0] == 'l':
                arm = self.left
            elif trajectory.joint_names[0][0] == 'r':
                arm = self.right
                
            if len(trajectory.joint_names) > 1:            
                joint_names = trajectory.joint_names
                points = [point.positions for point in trajectory.points]
                
                self.exec_status = ExecStatus.BUSY
                arm.execute_joint_trajectory(points, block=True)               
                self.exec_status = ExecStatus.IDLE
         
            else:
                if trajectory.points[0].positions[0] == 0:
                    self.exec_status = ExecStatus.BUSY
                    arm.close_gripper()
                    self.exec_status = ExecStatus.IDLE
                else:
                    self.exec_status = ExecStatus.BUSY
                    arm.open_gripper()
                    self.exec_status = ExecStatus.IDLE         
        except rospy.ROSException:
            self.exec_status = ExecStatus.ERROR

if __name__ == '__main__':
    executor = APCTrajectoryExecutor('joint_trajectories', 'exec_status')
    print "ready for execution"
    executor.spin()

