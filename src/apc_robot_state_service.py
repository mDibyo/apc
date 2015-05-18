#!/usr/bin/env python

from collections import deque

import roslib
roslib.load_manifest('apc')
import rospy
from ros_utils import ROSNode
import tf

from sensor_msgs.msg import JointState, PointCloud2
from apc.msg import RobotStateBase
from apc.srv import *

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

    def __init__(self, update_rate):
        super(APCRobotStateService, self).__init__('robot_state')
        
        self.tf_listener = tf.TransformListener()
        self.get_latest_state_service = rospy.Service('get_latest_robot_state',
                                                      GetLatestRobotState,
                                                      self.handle_get_latest_state)
                                                      
        self.joints = deque(maxlen=10)
        self.base = deque(maxlen=10)
        self.tf_all = []
        self.clouds = []
        self.joints_subscriber = rospy.Subscriber("joint_states",
                                                  JointState,
                                                  self.log_angles)
                                                  
        self.tf_subscriber = rospy.Subscriber("tf",
                                              tf.msg.tfMessage,
                                              self.log_base)        
                                              
        self.cloud_subscriber = rospy.Subscriber("camera/depth_registered/points",
                                                 PointCloud2,
                                                 self.log_cloud)
                                                                                 
    def __enter__(self):
        return self
                                                  
    def __exit__(self, exc_type, exc_val, exc_tb):
        pass
        
    def log_cloud(self, cloud_msg):
        self.clouds.append(cloud_msg)
        print len(self.clouds)
   
    def log_base(self, tf_msg):
        """ Log transform from /odom_combined to /base_footprint """
        for transform in tf_msg.transforms:
            self.tf_all.append(transform)
            if transform.header.frame_id == "/odom_combined" and transform.child_frame_id == "/base_footprint":
                T = transform.transform.translation
                self.base.append([T.x, T.y, T.z])
                                                      
    def log_angles(self, joint_state):
        """ Log all joint angles. """
        state = {}
        for name,value in zip(joint_state.name, joint_state.position):
            state[name] = value
        self.joints.append(state)                                                  
                                                       
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
        
        base_pos = self.base[-1]
        msg = RobotStateBase(joint_angles, base_pos)
        return GetLatestRobotStateResponse(msg)
        
if __name__ == '__main__':
    with APCRobotStateService(10) as node:
        node.spin()
        
