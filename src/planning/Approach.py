from __future__ import division

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from std_msgs.msg import Header

from message_wrappers import GraspWrapper, JointTrajectoryWrapper


class Approach(object):
    PREGRASP_POSE_OFFSET = -0.15
    GRASP_POSE_OFFSET = 0.045
    POSTGRASP_POSE_OFFSET = -0.2

    WAYPOINT_POSE = Pose(Point(0.35, 0.00739935381367, 1.2),
                         Quaternion(-0.00363820843569, -0.0145515141784,
                                    -0.242529037717, 0.970028186569))
    DROPZONE_POSE = Pose(Point(0.37946294703187866, 0.7371332656419127,
                               0.3694018168496691),
                         Quaternion(-0.5517710937068498, 0.3620950514477245,
                                    0.5284559388562138, 0.5340132531634529))

    def __init__(self, target_gripper_pose, target_gripper_width):
        self.gripper_width = target_gripper_width

        self.pregrasp_pose = self.get_offset_pose(target_gripper_pose,
                                                  self.PREGRASP_POSE_OFFSET)
        self.pregrasp_joints = None
        self.pregrasp_trajectory = None

        self.grasp_pose = self.get_offset_pose(target_gripper_pose,
                                               self.GRASP_POSE_OFFSET)
        self.grasp_joints = None
        self.grasp_trajectory = None

        p = self.grasp_pose.position
        q = self.grasp_pose.orientation

        self.postgrasp_pose = self.get_offset_pose(target_gripper_pose,
                                                   self.POSTGRASP_POSE_OFFSET)
        self.postgrasp_joints = None
        self.postgrasp_trajectory = None

        self.waypoint_pose = self.WAYPOINT_POSE
        self.waypoint_joints = None
        self.waypoint_trajectory = None

        self.dropzone_pose = self.DROPZONE_POSE
        self.dropzone_joints = None
        self.dropzone_trajectory = None

    @staticmethod
    def get_offset_pose(pose, offset):
        if not offset:
            return pose

        trans = np.array([pose.position.x,
                          pose.position.y,
                          pose.position.z])
        rot = np.array([pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w])

        T = tf.transformations.quaternion_matrix(rot)
        offset = T[:3, :3].dot([offset, 0, 0])

        return Pose(Point(*(trans + offset)), Quaternion(*rot))
