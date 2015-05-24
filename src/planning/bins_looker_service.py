#!/usr/bin/env python

import roslib
roslib.load_manifest('apc')
import rospy

import numpy as np
import utils
from ros_utils import ROSNode
from apc.srv import *
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from apc.msg import MotionPlan

class BinLooker(ROSNode):
    def __init__(self):
        super(BinLooker, self).__init__('bins_looker_service', anonymous=True)

        self.look_at_bins_service = rospy.Service('look_at_bins', LookAtBins,
                                                  self.handle_look_at_bins)
                                                  
        self.robot_state_client = rospy.ServiceProxy('get_latest_robot_state',
                                                     GetLatestRobotState)

    def handle_look_at_bins(self, req):
        bin_list = req.bin_list
        if not bin_list:
            bin_list = utils.BINS

        current_pos = np.array(self.robot_state_client('base').state_with_base.base_pose[-3:])

        motion_plan_list = []
        for bin_N in bin_list:
            motion_plan = MotionPlan()
            motion_plan.strategy = "look"
            motion_plan.trajectories = []
            
            if bin_N[-1] in ["G","H","I"]:
                x = -1.33
                head_point = [1,-0.1,1]
            elif bin_N[-1] in ["J", "K", "L"]:
                x = -2
                head_point = [1,-0.1,1.1]
            
            # point head
            motion_plan.head_direction = Point(head_point[0], head_point[1], head_point[2])
            
            target_pos = np.array([x, utils.bin_pose[bin_N][-2], 0])
            dist_to_move = target_pos - current_pos
            current_pos = target_pos
            
            motion_plan.base_target = Point(dist_to_move[0], dist_to_move[1], 0)

            # Move torso
            motion_plan.torso_height = Float32(0)

            # Capture image
            motion_plan.capture_scene = bin_N

            motion_plan_list.append(motion_plan)
           

        return LookAtBinsResponse(motion_plan_list)


if __name__ == '__main__':
    bin_looker = BinLooker()
    bin_looker.spin()
