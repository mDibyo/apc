#!/usr/bin/env python

"""
usage:
rosrun apc rviz_marker_publisher.py \
    "/home/dibyo/workspace/amazon_picking_challenge/expo_dry_erase_board_eraser/meshes/poisson.stl" \
    "[0, 0, 0]" "[0, 0, 0, 1]"
rosrun apc rviz_marker_publisher.py \
    -n "rviz_marker_publisher" -t 1 -i 12345
    -m "/home/dibyo/workspace/amazon_picking_challenge/expo_dry_erase_board_eraser/meshes/poisson.stl" \
    -p "[0, 0, 0]" -o "[0, 0, 0, 1]"
"""


import yaml
import random
from argparse import ArgumentParser

import roslib
roslib.load_manifest('apc')
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from ros_utils import ROSNode


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


class RvizPointsPublisher(ROSNode):
    def __init__(self, marker_type, name, pose, mesh_file="", id=-1,
                 remove=False):
        super(RvizPointsPublisher, self).__init__(name, anonymous=True)

        self.pub = rospy.Publisher('visualization_marker', Marker)
        self.br = tf.TransformBroadcaster()

        self.id = random.randrange(1000000) if id < 0 else id
        self.marker_type = marker_type
        self.pose = pose
        self.mesh_file = mesh_file
        self.remove = remove

        self.scale = 1.0

        self.speed = 0.01
        self.gripper_width = 0.06

        self.refresh_marker_map = {
            self.ARROW: self.refresh_marker_arrow,
            self.MESH: self.refresh_marker_mesh,
            self.GRIPPER: self.refresh_marker_gripper
        }

        if self.mesh_file:
            self.refresh_marker_mesh()
        else:
            self.refresh_marker_arrow()

    ARROW = 0
    MESH = 1
    GRIPPER = 2

    move_map = {
        # Position controls
        'w': ((1, 0, 0), (0, 0, 0), 0),
        'a': ((0, 1, 0), (0, 0, 0), 0),
        's': ((-1, 0, 0), (0, 0, 0), 0),
        'd': ((0, -1, 0), (0, 0, 0), 0),
        'i': ((0, 0, 1), (0, 0, 0), 0),
        'j': ((0, 0, -1), (0, 0, 0), 0),

        # Orientation controls
        '2': ((0, 0, 0), (0, 1, 0), 0),
        '8': ((0, 0, 0), (0, -1, 0), 0),
        '4': ((0, 0, 0), (-1, 0, 0), 0),
        '6': ((0, 0, 0), (1, 0, 0), 0),
        '1': ((0, 0, 0), (0, 0, -1), 0),
        '3': ((0, 0, 0), (0, 0, 1), 0),

        # Gripper width controls
        'g': ((0, 0, 0), (0, 0, 0), 1),
        'y': ((0, 0, 0), (0, 0, 0), -1)
    }

    def update_pose(self, move):
        delta = self.move_map.get(move, ((0, 0, 0), (0, 0, 0)))

        # Position change
        self.pose.position.x += self.speed * delta[0][0]
        self.pose.position.y += self.speed * delta[0][1]
        self.pose.position.z += self.speed * delta[0][2]

        # Orientation change
        roll, pitch, yaw = euler_from_quaternion([self.pose.orientation.x,
                                                  self.pose.orientation.y,
                                                  self.pose.orientation.z,
                                                  self.pose.orientation.w])
        roll += self.speed * delta[1][0]
        pitch += self.speed * delta[1][1]
        yaw += self.speed * delta[1][2]
        self.pose.orientation = Quaternion(*quaternion_from_euler(roll, pitch, yaw))

        # Gripper width change
        self.gripper_width += delta[2] * 0.001

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

    def publish_marker_msg(self, marker):
        while not self.pub.get_num_connections():
            rospy.sleep(0.1)

        print "publishing mesh resource: \n{}".format(str(marker))
        self.pub.publish(marker)

    def refresh_marker_mesh(self):
        """
        """
        marker = self.create_marker_msg(pose=self.pose, type=Marker.MESH_RESOURCE,
                                        action=Marker.ADD, id=self.id,
                                        mesh_resource="file://{}".format(self.mesh_file),
                                        scale_x=self.scale, scale_y=self.scale,
                                        scale_z=self.scale)

        self.publish_marker_msg(marker)

    def refresh_marker_arrow(self):
        """
        :rtype: None
        """
        marker = self.create_marker_msg(pose=self.pose, type=Marker.ARROW,
                                        action=Marker.ADD, id=self.id,
                                        scale_x=self.scale*0.2,
                                        scale_y=self.scale*0.2,
                                        scale_z=self.scale*0.2)

        self.publish_marker_msg(marker)

    def refresh_marker_gripper(self):
        marker = self.create_marker_msg(pose=self.pose, type=Marker.CUBE_LIST,
                                        action=Marker.ADD, id=self.id,
                                        scale_x=self.scale*0.05,
                                        scale_y=self.scale*0.01,
                                        scale_z=self.scale*0.02)

        marker.points = [
            Point(0, -self.gripper_width/2, 0),
            Point(0, self.gripper_width/2, 0)
        ]

        self.publish_marker_msg(marker)

    def delete_marker(self):
        marker = self.create_marker_msg(pose=self.pose, type=0,
                                        action=Marker.DELETE, id=self.id)

        self.publish_marker_msg(marker)

    def enter_control_loop(self):
        string = "This was typed: "

        ch = getch()
        while ch != 'p':
            self.update_pose(ch)
            self.refresh_marker_map[self.marker_type]()

            string += ch
            ch = getch()

        if self.remove:
            self.delete_marker()
        return string


if __name__ == '__main__':
    description = 'Publish and control markers to rviz environment'
    parser = ArgumentParser(description=description)
    parser.add_argument('-i', '--id', default=-1, type=int)
    parser.add_argument('-n', '--name', default='rviz_points_publisher')
    parser.add_argument('-r', '--remove', action='store_true',
                        help='whether the object is removed on node exit')
    parser.add_argument('-t', '--type', default='0', choices={0, 1, 2},
                        help='{0}: ARROW, {1}: MESH, {2}: GRIPPER'.
                        format(RvizPointsPublisher.ARROW,
                               RvizPointsPublisher.MESH,
                               RvizPointsPublisher.GRIPPER))
    parser.add_argument('-p', '--position', default='[0, 0, 0]',
                        type=yaml.load)
    parser.add_argument('-o', '--orientation', default='[0, 0, 0, 1]',
                        type=yaml.load)
    parser.add_argument('-m', '--mesh-file', default='')
    args = parser.parse_args()

    pose = Pose()
    pose.position = Point(*args.position)
    pose.orientation = Quaternion(*args.orientation)

    publisher = RvizPointsPublisher(marker_type=args.type, name=args.name,
                                    pose=pose, mesh_file=args.mesh_file,
                                    remove=args.remove)
    # import sys
    # pose = Pose()
    # pose.position = Point(0, 0, 0)
    # pose.orientation = Quaternion(0, 0, 0, 1)
    # mesh_location = ""
    #
    # if len(sys.argv) >= 2:
    #     mesh_location = sys.argv[1]
    # if len(sys.argv) >= 3:
    #     pose.position = Point(*yaml.load(sys.argv[2]))
    # if len(sys.argv) >= 4:
    #     pose.orientation = Quaternion(*yaml.load(sys.argv[3]))
    #
    # publisher = RvizPointsPublisher("rviz_points_publisher", pose, mesh_location)

    print publisher.enter_control_loop()
