#!/usr/bin/env python

from __future__ import division

import json

import roslib
roslib.load_manifest('apc')
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np

from apc.msg import Grasp, MotionPlan


__author__ = 'dibyo'


class JointTrajectoryWrapper(object):
    def __init__(self, joint_names, trajectory):
        self.joint_names = joint_names
        self.trajectory = trajectory

    def to_msg(self):
        points = []
        for waypoint in self.trajectory:
            point = JointTrajectoryPoint()
            point.positions = waypoint
            points.append(point)
        return JointTrajectory(Header(), self.joint_names, points)

    @classmethod
    def from_msg(cls, jt):
        trajectory = [point.positions for point in jt.points]
        return cls(jt.joint_names, np.array(trajectory))


class MotionPlanWrapper(object):
    def __init__(self, strategy, trajectories=None, joint_names=None):
        self.strategy = strategy
        self.trajectories = [] if trajectories is None else trajectories
        self.joint_names = [] if joint_names is None else joint_names

    def to_msg(self):
        return MotionPlan(self.strategy,
                          [JointTrajectoryWrapper(self.joint_names,
                                                  trajectory).to_msg()
                           for trajectory in self.trajectories])


class GraspWrapper(object):
    def __init__(self, gripper_pose, gripper_width=None, flag=False):
        self.gripper_pose = gripper_pose
        self.gripper_width = gripper_width
        self.flag = flag

    def to_msg(self):
        return Grasp(self.gripper_pose, self.gripper_width)

    def __str__(self):
        return str(self.to_msg())

    def to_json(self):
        p = self.gripper_pose.position
        o = self.gripper_pose.orientation

        d = {
            'gripper_pose': {
                'position': {'x': p.x, 'y': p.y, 'z': p.z},
                'orientation': {'x': o.x, 'y': o.y, 'z': o.z, 'w': o.w}
            },
            'gripper_width': self.gripper_width,
            'flag': self.flag
        }
        return d

    @classmethod
    def from_json(cls, d):
        pose = Pose()
        pose.position = Point(**d['gripper_pose']['position'])
        pose.orientation = Quaternion(**d['gripper_pose']['orientation'])

        return cls(pose, d['gripper_width'], d['flag'])

    @classmethod
    def grasps_from_file(cls, grasps_file):
        with open(grasps_file, 'r') as f:
            return [cls.from_json(grasp_json) for grasp_json in json.load(f)]

    @classmethod
    def grasps_to_file(cls, grasps, grasps_file):
        with open(grasps_file, 'w+') as f:
            json.dump([grasp.to_json() for grasp in grasps], f,
                      indent=4, separators=(', ', ': '))
