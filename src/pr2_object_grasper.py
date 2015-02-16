#!/usr/bin/env python

import roslib
roslib.load_manifest('apc')
roslib.load_manifest('tfx')

import rospy
import tfx
import tf
from geometry_msgs.msg import PoseStamped, Point, Quaternion

from pr2 import arm as _arm

from ros_utils import ROSNode

import yaml
from argparse import ArgumentParser


__author__ = 'dibyo'

class PR2ObjectGrasper(ROSNode):
    def __init__(self, gripper_transform_position, gripper_transform_orientation, 
                 name='pr2_object_grasper', arm='right',
                 object_marker='object_marker'):
        super(PR2ObjectGrasper, self).__init__(name)

        self.arm = _arm.Arm(arm)
        print '1'
        self.listener = tfx.TransformListener()

        self.object_marker = object_marker
        self.frame = 'base_link'
        
        # self.gripper_transform_position = gripper_transform_position
        # self.gripper_transform_orientation = gripper_transform_orientation
        self.gripper_pose = PoseStamped()


        self.gripper_pose.pose.position = Point(*gripper_transform_position)
        self.gripper_pose.pose.orientation = Quaternion(*gripper_transform_orientation)
        self.gripper_pose.header.frame_id = self.object_marker

        self.grasp_object()
    
    def get_gripper_pose(self, trans, rot):
        pass

    def grasp_object(self):
        done = False

        while not done:
            try:
                self.listener.waitForTransform(self.frame, self.object_marker,
                                               rospy.Time(0),
                                               rospy.Duration(1.0))
                done = True
            except tf.Exception:
                continue

        self.gripper_pose.header.stamp = rospy.Time(0)
        target_pose = self.listener.transformPose(self.frame, self.gripper_pose)

        p = target_pose.pose.position
        o = target_pose.pose.orientation
        target_pose = tfx.pose([p.x, p.y, p.z], [o.x, o.y, o.z, o.w],
                               frame=target_pose.header.frame_id)
        self.arm.go_to_pose(target_pose, block=True, speed=0.30)

        raw_input()
        self.arm.close_gripper(block=True)
        #
        # raw_input()
        # end_pose = tfx.pose([trans[0], trans[1], trans[2]+0.2], list(rot),
        #                     frame=self.frame)
        # self.arm.go_to_pose(end_pose, block=True, speed=0.30)


if __name__ == '__main__':
    # import sys
    
    # gripper_transform_position = yaml.load(sys.argv[1]) if len(sys.argv) > 1 else [0, 0, 0]
    # gripper_transform_orientation = yaml.load(sys.argv[2]) if len(sys.argv) > 2 else [0, 0, 0, 1]
    description = 'Move PR2 gripper to object position'
    parser = ArgumentParser(description=description)
    parser.add_argument('-p', '--position', default='[0, 0, 0]',
                        type=yaml.load)
    parser.add_argument('-o', '--orientation', default='[0, 0, 0, 1]',
                        type=yaml.load)
    # parser.add_argument('--gripper-name', default='gripper_marker')
    parser.add_argument('--object-marker', default='object_marker')
    args = parser.parse_args()


    PR2ObjectGrasper(gripper_transform_position=args.position,
                     gripper_transform_orientation=args.orientation,
                     object_marker=args.object_marker)
    # trans = [0.650, 0.000, 0.650]
    # rot = [0.000, 0.000, -0.085, 0.996]

    # trans = [0.768, -0.017, 0.639]
    # rot = [-0.38750825843847903,
    #        -0.59118907232473883,
    #        0.2816828623504114,
    #        0.64883556889544125]

    # target_pose


    # [0.020, 0.000, 0.127]
    # [0.325, 0.630, -0.328, 0.624]

    # [-0.003, 0.048, 0.055]
    # [0.013, 0.022, -0.509, 0.860]