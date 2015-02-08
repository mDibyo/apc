#!/usr/bin/env python

"""
usage:
rosrun apc rviz_points_publisher.py \
    "/home/dibyo/workspace/amazon_picking_challenge/expo_dry_erase_board_eraser/meshes/poisson.stl" \
    "[0, 0, 0]" "[0, 0, 0, 1]"
"""


import yaml
import random

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion

import roslib
roslib.load_manifest('apc')
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def getch():
    import sys
    import tty
    import termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class RvizPointsPublisher(object):
    def __init__(self, pose, mesh_file=""):
        self.pub = rospy.Publisher('visualization_marker', Marker)
        rospy.init_node('rviz_points_publisher', anonymous=True)

        self.id = random.randrange(1000000)
        self.pose = pose
        self.mesh_file = mesh_file
        self.speed = 0.01
        self.scale = 1.0

        if self.mesh_file:
            self.refresh_marker_mesh()
        else:
            self.refresh_marker_arrow()

    move_map = {
        'w': ((1, 0, 0), (0, 0, 0)),
        'a': ((0, 1, 0), (0, 0, 0)),
        's': ((-1, 0, 0), (0, 0, 0)),
        'd': ((0, -1, 0), (0, 0, 0)),
        'i': ((0, 0, 1), (0, 0, 0)),
        'j': ((0, 0, -1), (0, 0, 0)),

        '2': ((0, 0, 0), (0, 1, 0)),
        '8': ((0, 0, 0), (0, -1, 0)),
        '4': ((0, 0, 0), (-1, 0, 0)),
        '6': ((0, 0, 0), (1, 0, 0)),
        '1': ((0, 0, 0), (0, 0, -1)),
        '3': ((0, 0, 0), (0, 0, 1)),
    }

    def update_pose(self, move):
        delta = self.move_map.get(move, ((0, 0, 0), (0, 0, 0)))
        self.pose.position.x += self.speed * delta[0][0]
        self.pose.position.y += self.speed * delta[0][1]
        self.pose.position.z += self.speed * delta[0][2]

        roll, pitch, yaw = euler_from_quaternion([self.pose.orientation.x,
                                                  self.pose.orientation.y,
                                                  self.pose.orientation.z,
                                                  self.pose.orientation.w])
        roll += self.speed * delta[1][0]
        pitch += self.speed * delta[1][1]
        yaw += self.speed * delta[1][2]
        self.pose.orientation = Quaternion(*quaternion_from_euler(roll, pitch, yaw))

    @staticmethod
    def create_marker_msg(pose, type, id=0, frame_id='base_link', ns='apc',
                          scale_x=1.0, scale_y=1.0, scale_z=1.0, color_a=1.0,
                          color_r=1.0, color_g=1.0, color_b=1.0, action=0,
                          mesh_resource=""):
        marker = Marker()

        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()

        marker.ns = ns
        marker.id = id

        marker.type = type
        marker.action = action
        marker.mesh_resource = mesh_resource

        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = scale_z
        marker.color.a = color_a
        marker.color.r = color_r
        marker.color.g = color_g
        marker.color.b = color_b

        marker.pose = pose
        marker.lifetime = rospy.Duration()

        return marker

    def refresh_marker_mesh(self):
        """
        """
        marker = self.create_marker_msg(pose=self.pose, type=Marker.MESH_RESOURCE,
                                        action=Marker.ADD, id=self.id,
                                        mesh_resource="file://{}".format(self.mesh_file),
                                        scale_x=self.scale, scale_y=self.scale,
                                        scale_z=self.scale)

        while not self.pub.get_num_connections():
            rospy.sleep(0.1)

        print "publishing mesh resource: \n{}".format(str(marker))
        self.pub.publish(marker)

    def refresh_marker_arrow(self):
        """
        :rtype: None
        """
        marker = self.create_marker_msg(pose=self.pose, type=Marker.ARROW,
                                        action=Marker.ADD, id=self.id,
                                        scale_x=0.2, scale_y=0.2, scale_z=0.2)

        while not self.pub.get_num_connections():
            rospy.sleep(0.1)

        print "publishing marker: \n{}".format(str(marker))
        self.pub.publish(marker)

    def enter_control_loop(self):
        string = "This was typed: "

        ch = getch()
        while ch != 'p':
            self.update_pose(ch)
            if self.mesh_file:
                self.refresh_marker_mesh()
            else:
                self.refresh_marker_arrow()

            string += ch
            ch = getch()

        return string


if __name__ == '__main__':
    import sys
    pose = Pose()
    pose.position = Point(0, 0, 0)
    pose.orientation = Quaternion(0, 0, 0, 1)
    mesh_location = ""

    if len(sys.argv) >= 2:
        mesh_location = sys.argv[1]
    if len(sys.argv) >= 3:
        pose.position = Point(*yaml.load(sys.argv[2]))
    if len(sys.argv) >= 4:
        pose.orientation = Quaternion(*yaml.load(sys.argv[3]))

    publisher = RvizPointsPublisher(pose, mesh_location)

    print publisher.enter_control_loop()
