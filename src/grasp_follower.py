#!/usr/bin/env python
__author__ = 'dibyo'

import roslib
roslib.load_manifest('apc')

import rospy

from pr2 import arm as _arm

from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from apc.msg import Grasp as GraspMsg


class GraspFollower(object):
    
    def __init__(self, arm='right'):
        rospy.init_node('grasp_follower', anonymous=True)

        self.arm = _arm.Arm(arm)
        self.frame = 'base_link'
        
        # The name of the topic might need to be changed.
        rospy.Subscriber('gripper-moves-topic', GraspMsg, self.callback)

    def callback(self, graspMsg):

        target_pose = graspMsg.pose
        
        self.arm.go_to_pose(target_pose, block=True, speed=0.30)


if __name__ == '__main__':
    import sys
    arm = sys.argv[1] if len(sys.argv) > 1 else 'right'

    graspFollower = GraspFollower(arm)

    # Keep python from exiting until node is stopped
    rospy.spin()