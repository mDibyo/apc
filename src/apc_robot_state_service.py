#!/usr/bin/env python

from collections import deque
import os

import numpy as np
import roslib
roslib.load_manifest('apc')
import rospy
from ros_utils import ROSNode
import tf
import openravepy as rave

from sensor_msgs.msg import JointState, PointCloud2
from apc.msg import RobotStateBase
from apc.srv import *
from utils import APC_DIRECTORY, MODEL_DIR, SHELF_POSE_FILE

class APCRobotStateService(ROSNode):
    """ Subscribes to tf and joint_states and keep log of recent robot state.
        Call service with a manipulator name [left/right]arm[torso/] """
    
    torso_joint = 'torso_lift_joint'
    arm_joints= ['shoulder_pan_joint',
                 'shoulder_lift_joint',
                 'upper_arm_roll_joint',
                 'elbow_flex_joint',
                 'forearm_roll_joint',
                 'wrist_flex_joint',
                 'wrist_roll_joint']
    head_joints = ['head_pan_joint', 'head_tilt_joint']
                      
    T_rot = np.array([[ 0.,  0.,  1.,  0.],
                       [-1.,  0.,  0.,  0.],
                       [ 0., -1.,  0.,  0.],
                       [ 0.,  0.,  0.,  1.]])
                      
    T_tr = np.array([[ 1.  ,  0.  ,  0.  , -0.22],
                     [ 0.  ,  1.  ,  0.  , 0.025],
                     [ 0.  ,  0.  ,  1.  , 0.21 ],
                     [ 0.  ,  0.  ,  0.  ,  1.  ]])


    def __init__(self, shelf_pose_file):
        super(APCRobotStateService, self).__init__('robot_state')

        self.robot_start_mat = np.linalg.inv(np.loadtxt(shelf_pose_file)) # store world to robot
            
        self.tf_listener = tf.TransformListener()
        self.get_latest_state_service = rospy.Service('get_latest_robot_state',
                                                      GetLatestRobotState,
                                                      self.handle_get_latest_state)
                                                      
        self.get_camera_pose_service = rospy.Service('get_camera_pose',
                                                     GetCameraPose,
                                                     self.handle_get_camera_pose)
                                                                                                 
        self.joints = deque(maxlen=10)
        self.base = deque(maxlen=10)
        self.tf_all = []
        
        self.joints_subscriber = rospy.Subscriber("/joint_states",
                                                  JointState,
                                                  self.log_angles)
                                                  
        self.tf_subscriber = rospy.Subscriber("/tf",
                                              tf.msg.tfMessage,
                                              self.log_base)   
                                              
        rospy.logwarn("robot state info ready")
     
                                                                    
    def __enter__(self):
        self.env = rave.Environment()
        self.env.Load(os.path.join(MODEL_DIR, "pr2-beta-static.zae"))
        self.robot = self.env.GetRobot('pr2')
        print "ready"
        return self
                                                  
    def __exit__(self, exc_type, exc_val, exc_tb):
        rave.RaveDestroy
   
    def log_base(self, tf_msg):
        """ Log transform from /odom_combined to /base_footprint """
        for transform in tf_msg.transforms:
            self.tf_all.append(transform)
            if transform.header.frame_id == "/odom_combined" and transform.child_frame_id == "/base_footprint":
               T = transform.transform
               self.base.append(np.array([T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z,
                                          T.translation.x, T.translation.y, T.translation.z]))
                                                      
    def log_angles(self, joint_state):
        """ Log all joint angles. """
        state = {}
        for name,value in zip(joint_state.name, joint_state.position):
            state[name] = value
        self.joints.append(state)                                                  
    
    def handle_get_camera_pose(self, req):
        """ save T_camera_to_robot """
        joints = self.head_joints
        latest = self.joints[-1]        
        joint_angles = [latest[self.torso_joint]]
        for j in joints:
            joint_angles.append(latest[j])
        self.robot.SetDOFValues(joint_angles, [12,13,14])
        
        
        
        T = self.robot.GetLink("sensor_mount_link").GetTransform()
        mat = T.dot(self.T_tr.dot(self.T_rot))
        fname = osp.join(APC_DIRECTORY, 'src', 'perception', 'shelf_finder', 'transform.txt')
        np.savetxt(fname, mat)
        return os.path.abspath(fname)
                    
    def handle_get_latest_state(self, req):
        """ Return most recent joint angles and base position for given manipulator. """
        manip = req.manipulator
        joints = []
        if "torso" in manip:
            joints.append(self.torso_joint)
            
        if "right" in manip:
            joints.extend(["r_" + name for name in self.arm_joints])
        elif "left" in manip:
            joints.extend(["l_" + name for name in self.arm_joints])
        elif "head" in manip:
            joints.extend(self.head_joints)
            
        latest = self.joints[-1]
        
        joint_angles = []      
        for j in joints:
            joint_angles.append(latest[j])
        
        if self.robot_start_mat is None:
            try:
                self.robot_start_mat = np.linalg.inv(np.loadtxt(shelf_pose_file)) # store world to robot
            except Exception:
                self.robot_start_mat = None
                
        if self.robot_start_mat is None:
            start_mat = np.eye(4)
            rospy.logwarn("using default start mat")
        else:
            start_mat = self.robot_start_mat
            
        print "odometry at",self.base[-1][-3:]
        base_mat = rave.matrixFromPose(self.base[-1])
        base_pose = rave.poseFromMatrix(base_mat.dot(start_mat))
        msg = RobotStateBase(joint_angles, base_pose.tolist())
        return GetLatestRobotStateResponse(msg)
        
if __name__ == '__main__':
    with APCRobotStateService(SHELF_POSE_FILE) as self:
        self.spin()
        
