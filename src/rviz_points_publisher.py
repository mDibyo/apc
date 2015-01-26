#!/usr/bin/env python

import yaml

import roslib
roslib.load_manifest('rviz')

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion


class RvizPointsPublisher(object):
    def __init__(self):
        self.pub = rospy.Publisher('visualization_marker', Marker)
        rospy.init_node('rviz_points_publisher', anonymous=True)

    def publish(self, pose):
        """
        :param pose: The pose of the marker
        :type pose: Pose
        :rtype: None
        """
        marker = Marker()

        marker.header.frame_id = 'base_link'
        marker.header.stamp = rospy.Time.now()

        marker.ns = "apc"
        marker.id = 0

        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.scale.x = 1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        marker.pose = pose

        marker.lifetime = rospy.Duration()

        while not self.pub.get_num_connections():
            rospy.sleep(0.1)

        print "publishing marker: \n{}".format(str(marker))
        self.pub.publish(marker)


if __name__ == '__main__':
    print "hello    "

    import sys
    if len(sys.argv) < 3:
        raise RuntimeError("Not enough arguments")

    print sys.argv

    pose = Pose()
    pose.position = Point(*yaml.load(sys.argv[1]))
    pose.orientation = Quaternion(*yaml.load(sys.argv[2]))

    publisher = RvizPointsPublisher()
    publisher.publish(pose)
