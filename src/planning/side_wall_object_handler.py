#!/usr/bin/env python

import roslib
roslib.load_manifest('apc')
import rospy

__author__ = 'dibyo'


ORIENTATION = [0.5, 0.5, 0.5, 0.5]
POSITION = [0, 0.05, 0.08]


class SideWallObjectHandler(object):
    env = None
    robot = None
    ikmodel = None

    

