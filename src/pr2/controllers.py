from __future__ import division

import roslib
roslib.load_manifest('apc')

import rospy
# import tf
# import actionlib
#
# import openravepy as rave
# import numpy as np

# from pr2_controllers_msgs.msg import *
# from trajectory_msgs.msg import JointTrajectory
# from sensor_msgs.msg import JointState

from ros_utils import ROSNode

__author__ = 'dibyo'

PR2_CONTROLLERS = [
    'head_traj_controller',
    'torso_controller',
    'base_controller',
    'l_arm_controller',
    'r_arm_controller',
    'r_gripper_controller',
    'l_gripper_controller',
]


class PR2Controller(ROSNode):
    def __init__(self, name, anonymous):
        super(PR2Controller, self).__init__(name,
                                            anonymous=anonymous)


class ArmController(PR2Controller):
    JOINT_NAMES = [
        "_shoulder_pan_joint",
        "_shoulder_lift_joint",
        "_upper_arm_roll_joint",
        "_elbow_flex_joint",
        "_forearm_roll_joint",
        "_wrist_flex_joint",
        "_wrist_roll_joint"
    ]
    
    def __init__(self, arm, anonymous=True):
        self.arm = arm

        super(ArmController, self).__init__('pr2_{}_arm_controller'.
                                            format(arm[0]), anonymous)
        self.JOINT_NAMES = [arm[0] + joint_name for joint_name
                            in self.JOINT_NAMES]


class LeftArmController(ArmController):
    JOINT_NAMES = None
    
    def __init__(self, anonymous=True):
        super(LeftArmController, self).__init__('left',
                                                anonymous)
        

class RightArmController(ArmController):
    def __init__(self, anonymous=True):
        super(RightArmController, self).__init__('right',
                                                 anonymous)


class GripperController(PR2Controller):
    def __init__(self, arm, anonymous=True):
        super(GripperController, self).__init__('pr2_{}_gripper_controller'.
                                                format(arm[0]), anonymous)


class LeftGripperController(PR2Controller):
    def __init__(self, anonymous=True):
        super(LeftGripperController, self).__init__('left',
                                                    anonymous)


class RightGripperController(PR2Controller):
    def __init__(self, anonymous=True):
        super(RightGripperController, self).__init__('right',
                                                     anonymous)


class HeadController(PR2Controller):
    def __init__(self, anonymous=True):
        super(HeadController, self).__init__('pr2_head_controller',
                                             anonymous)


class TorsoController(PR2Controller):
    def __init__(self, anonymous=True):
        super(TorsoController, self).__init__('pr2_torso_controller',
                                             anonymous)


class BaseController(PR2Controller):
    def __init__(self, anonymous=True):
        super(BaseController, self).__init__('pr2_base_controller',
                                             anonymous)

