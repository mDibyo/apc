#!/usr/bin/env python

"""
usage:
rosrun apc rviz_points_publisher.py \
    "/home/dibyo/workspace/amazon_picking_challenge/expo_dry_erase_board_eraser/meshes/poisson.stl" \
    "[0, 0, 0]" "[0, 0, 0, 1]"
"""


import yaml

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

    def refresh_marker_mesh(self):
        """
        """
        marker = Marker()

        marker.header.frame_id = 'base_link'
        marker.header.stamp = rospy.Time.now()

        marker.ns = 'apc'
        marker.id = 1

        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = \
            "file://{}".format(self.mesh_file)

        marker.scale.x = self.scale
        marker.scale.y = self.scale
        marker.scale.z = self.scale
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        marker.pose = self.pose

        marker.lifetime = rospy.Duration()

        while not self.pub.get_num_connections():
            rospy.sleep(0.1)

        print "publishing mesh resource: \n{}".format(str(marker))
        self.pub.publish(marker)

    def refresh_marker_arrow(self):
        """
        :rtype: None
        """
        marker = Marker()

        marker.header.frame_id = 'base_link'
        marker.header.stamp = rospy.Time.now()

        marker.ns = "apc"
        marker.id = 0

        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        marker.pose = self.pose

        marker.lifetime = rospy.Duration()

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
            # print "I got a {}".format(ch)
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
