#!/usr/bin/env python

"""
usage:
rosrun apc rviz_marker_publisher.py \
    -n "rviz_marker_publisher" -t 1 -r -u 50 -c "apc/marker_move"\
    -m "/home/dibyo/workspace/amazon_picking_challenge/expo_dry_erase_board_eraser/meshes/poisson.stl" \
    -p "[0, 0, 0]" -o "[0, 0, 0, 1]"
"""


import yaml
import random
from argparse import ArgumentParser
import threading

import roslib
roslib.load_manifest('apc')
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from ros_utils import ROSNode
from utils import LoopingThread, getch


class RvizMarkerPublisher(ROSNode):
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
        'g': ((0, 0, 0), (0, 0, 0), -1),
        'y': ((0, 0, 0), (0, 0, 0), 1)
    }

    def __init__(self, marker_type, name, pose, mesh_file="", id=-1,
                 control_topic=None, update_rate=0, delete=False):
        super(RvizMarkerPublisher, self).__init__(name, anonymous=True)

        self.publisher = rospy.Publisher('visualization_marker', Marker)
        self.subscriber = None
        self.br = tf.TransformBroadcaster()

        self.id = random.randrange(1000000) if id < 0 else id
        self.marker_type = marker_type
        self.pose = pose
        self.mesh_file = mesh_file
        self.control_topic = control_topic
        self.update_rate = update_rate
        self.delete = delete

        self.scale = 1.0

        self.speed = 0.01
        self.gripper_width = 0.06

        self.update_marker = {
            self.ARROW: self.update_marker_arrow,
            self.MESH: self.update_marker_mesh,
            self.GRIPPER: self.update_marker_gripper
        }[self.marker_type]

        self.update_marker()

    def __enter__(self):
        # Update constantly
        if self.update_rate:
            self.update_thread = LoopingThread(target=self.update_marker,
                                               rate=self.update_rate)
            self.update_thread.start()

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.update_rate:
            self.update_thread.stop()

        if self.delete:
            self.delete_marker()

    def update_pose(self, move):
        delta = self.move_map.get(move, ((0, 0, 0), (0, 0, 0), 0))

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
        self.gripper_width += delta[2] * 0.005

    @staticmethod
    def create_marker_msg(pose, type, id, frame_id='base_link', ns='apc',
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
        while not self.publisher.get_num_connections():
            rospy.sleep(0.1)

        self.publisher.publish(marker)

    def broadcast_transform(self):
        p = self.pose.position
        o = self.pose.orientation
        self.br.sendTransform((p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
                              rospy.Time.now(), self.name, "base_link")

    def update_marker_mesh(self):
        """
        """
        marker = self.create_marker_msg(pose=self.pose, type=Marker.MESH_RESOURCE,
                                        action=Marker.ADD, id=self.id,
                                        mesh_resource="file://{}".format(self.mesh_file),
                                        scale_x=self.scale, scale_y=self.scale,
                                        scale_z=self.scale)

        self.publish_marker_msg(marker)
        self.broadcast_transform()

    def update_marker_arrow(self):
        """
        :rtype: None
        """
        marker = self.create_marker_msg(pose=self.pose, type=Marker.ARROW,
                                        action=Marker.ADD, id=self.id,
                                        scale_x=self.scale*0.2,
                                        scale_y=self.scale*0.2,
                                        scale_z=self.scale*0.2)

        self.publish_marker_msg(marker)
        self.broadcast_transform()

    def update_marker_gripper(self):
        marker = self.create_marker_msg(pose=self.pose, type=Marker.CUBE_LIST,
                                        action=Marker.ADD, id=self.id,
                                        scale_x=self.scale*0.1,
                                        scale_y=self.scale*0.01,
                                        scale_z=self.scale*0.05)

        marker.points = [
            Point(0, -self.gripper_width/2, 0),
            Point(0, self.gripper_width/2, 0),
            Point(-0.075, 0, 0)
        ]

        self.publish_marker_msg(marker)
        self.broadcast_transform()

    def delete_marker(self):
        marker = self.create_marker_msg(pose=self.pose, type=0,
                                        action=Marker.DELETE, id=self.id)

        self.publish_marker_msg(marker)

    def enter_control_loop(self):
        if self.control_topic is not None:
            self.subscriber = rospy.Subscriber(self.control_topic, String,
                                               self.execute_move)
            self.spin()
        else:
            ch = getch()
            while ch != 'p':
                self.execute_move(ch)
                ch = getch()

    def execute_move(self, move):
        self.update_pose(move)
        self.update_marker()


if __name__ == '__main__':
    description = 'Publish and control markers in rviz environment'
    parser = ArgumentParser(description=description)
    parser.add_argument('-n', '--name', default='rviz_marker')
    parser.add_argument('-d', '--delete', action='store_true',
                        help='whether the object should be deleted on node exit')
    parser.add_argument('-t', '--type', default='0', choices={0, 1, 2},
                        help='{0}: ARROW, {1}: MESH, {2}: GRIPPER'.
                        format(RvizMarkerPublisher.ARROW,
                               RvizMarkerPublisher.MESH,
                               RvizMarkerPublisher.GRIPPER), type=int)
    parser.add_argument('-p', '--position', default='[0, 0, 0]',
                        type=yaml.load)
    parser.add_argument('-o', '--orientation', default='[0, 0, 0, 1]',
                        type=yaml.load)
    parser.add_argument('-m', '--mesh-file', default='')
    parser.add_argument('-c', '--control-topic', default=None)
    parser.add_argument('-u', '--update-rate', default='0', type=int,
                        help='the rate at which updates should be published')
    args = parser.parse_args()

    pose = Pose()
    pose.position = Point(*args.position)
    pose.orientation = Quaternion(*args.orientation)

    # publisher = RvizMarkerPublisher(marker_type=args.type, name=args.name,
    #                                 pose=pose, mesh_file=args.mesh_file,
    #                                 remove=args.remove)
    #
    # publisher.enter_control_loop()
    with RvizMarkerPublisher(marker_type=args.type, name=args.name,
                             pose=pose, mesh_file=args.mesh_file,
                             control_topic=args.control_topic,
                             update_rate=args.update_rate,
                             delete=args.delete) as publisher:
        publisher.enter_control_loop()