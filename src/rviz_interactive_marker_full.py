#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""
import math

import roslib
roslib.load_manifest("interactive_markers")

import rospy

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarker, Marker, \
    InteractiveMarkerControl, InteractiveMarkerFeedback
import tf


# server = None
# marker_pos = 0
# br = None
# counter = 0


class Server(object):
    def __init__(self):
        self.marker_pos = 0
        self.counter = 0

        rospy.init_node("basic_controls_apc")
        self.br = tf.TransformBroadcaster()

        # create a timer to update the published transforms
        rospy.Timer(rospy.Duration(0.05), self.frameCallback)

        self.server = InteractiveMarkerServer("apc")

        # make6DofMarker(False)
        self.make6DofMarker(True)

        # server.applyChanges()

    def frameCallback(self, msg):
            # global counter, br
            # print msg
            time = rospy.Time.now()
            self.br.sendTransform((0, 0, math.sin(self.counter/140.0)*2.0),
                                  (0, 0, 0, 1.0),
                                  rospy.Time.now(),
                                  "apc_object",
                                  "base_link")
            self.counter += 1

    def processFeedback(self, feedback):
        rospy.logdebug("hello")
        print "hello"
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo(s + ": button click" + mp + ".")
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo(s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + ".")
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo(s + ": pose changed")
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo(s + ": mouse down" + mp + ".")
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo(s + ": mouse up" + mp + ".")
        self.server.applyChanges()


    def alignMarker(self, feedback):
        pose = feedback.pose
        print "hello there"

        pose.position.x = round(pose.position.x-0.5)+0.5
        pose.position.y = round(pose.position.y-0.5)+0.5

        rospy.loginfo(feedback.marker_name + ": aligning position = " + str(feedback.pose.position.x) + "," +
                      str(feedback.pose.position.y) + "," + str(feedback.pose.position.z) + " to " +
                      str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.position.z))

        self.server.setPose(feedback.marker_name, pose)
        self.server.applyChanges()


    def makeBox(self, msg):
        marker = Marker()

        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        return marker


    def makeBoxControl(self, msg):
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.makeBox(msg))
        msg.controls.append(control)
        return control


# def saveMarker(int_marker):
#     server.insert(int_marker, processFeedback)


#####################################################################
# Marker Creation

    def make6DofMarker(self, fixed):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/base_link"
        int_marker.pose.position.y = -3.0 * self.marker_pos
        self.marker_pos += 1
        int_marker.scale = 0.1

        int_marker.name = "simple_6dof"
        # int_marker.description = "Simple 6-DOF Control"

        # insert a box
        self.makeBoxControl(int_marker)

        if fixed:
            int_marker.name += "_fixed"
            # int_marker.description += "\n(fixed orientation)"

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.processFeedback)
        self.server.applyChanges()


if __name__=="__main__":
    node = Server()

    rospy.spin()
