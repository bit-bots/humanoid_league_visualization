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
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from humanoid_league_msgs.msg import BallRelative
from geometry_msgs.msg import Pose, Point


server = None
menu_handler = MenuHandler()
publish_balls = True

def ball_feedback(feedback):
    global pose
    pose = feedback.pose
    server.applyChanges()



def makeSphere(msg):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * 0.13
    marker.scale.y = msg.scale * 0.13
    marker.scale.z = msg.scale * 0.13
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    return marker


def make_ball_marker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.pose.position = position
    int_marker.scale = 1

    int_marker.name = "ball"

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    int_marker.controls.append(copy.deepcopy(control))

    # make a box which also moves in the plane
    control.markers.append(makeSphere(int_marker))
    control.always_visible = True
    int_marker.controls.append(control)

    # we want to use our special callback function
    server.insert(int_marker, ball_feedback)


def pubBall(e):
    global pose, pub
    ball = BallRelative()
    ball.header.stamp = rospy.get_rostime()
    ball.header.frame_id = "map"
    ball.confidence = 1.0
    ball.ball_relative = pose.position
    if publish_balls:
        pub.publish(ball)

def menu_callback(feedback):
    global publish_balls
    item = feedback.menu_entry_id
    print(menu_handler.getCheckState(item))
    if menu_handler.getCheckState(item) == MenuHandler.CHECKED:
        menu_handler.setCheckState(item, MenuHandler.UNCHECKED)
        publish_balls = False
    else:
        publish_balls = True
        menu_handler.setCheckState(item, MenuHandler.CHECKED)


    menu_handler.reApply(server)
    server.applyChanges()


if __name__ == "__main__":
    global pub, pose, item
    rospy.init_node("humanoid_league_interactive_marker")

    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.1), pubBall)

    server = InteractiveMarkerServer("basic_controls")
    pub = rospy.Publisher("ball_relative", BallRelative, queue_size=1)
    pose = Pose()
    pose.position = Point(0, 0, 0)
    make_ball_marker(pose.position)
    item = menu_handler.insert("publish ball", callback=menu_callback)
    menu_handler.setCheckState(item, MenuHandler.CHECKED)
    menu_handler.apply(server, "ball")
    rospy.Timer(rospy.Duration(0.1), pubBall)

    server.applyChanges()

    rospy.spin()

