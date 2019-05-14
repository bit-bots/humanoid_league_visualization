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
import math

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from humanoid_league_msgs.msg import BallRelative, GoalRelative
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from tf.transformations import euler_from_quaternion

BALL_DIAMETER = 0.13
GOAL_WIDTH = 1.5
GOAL_HEIGHT = 1.1
POST_DIAMETER = 0.1


class RobocupInteractiveMarker(object):
    def __init__(self, server):
        self.server = server
        self.pose = Pose()
        self.publish = True
        self.make_marker()
        self.menu_handler = MenuHandler()
        item = self.menu_handler.insert("publish", callback=self.menu_callback)
        self.menu_handler.setCheckState(item, MenuHandler.CHECKED)
        self.menu_handler.apply(self.server, self.marker_name)

    def feedback(self, feedback):
        self.pose = feedback.pose
        self.server.applyChanges()

    def menu_callback(self, feedback):
        item = feedback.menu_entry_id
        if self.menu_handler.getCheckState(item) == MenuHandler.CHECKED:
            self.menu_handler.setCheckState(item, MenuHandler.UNCHECKED)
            self.publish = False
        else:
            self.publish = True
            self.menu_handler.setCheckState(item, MenuHandler.CHECKED)

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def make_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.pose = self.pose
        int_marker.scale = 1

        int_marker.name = self.marker_name

        control = InteractiveMarkerControl()
        control.orientation.w = math.sqrt(2) / 2
        control.orientation.x = 0
        control.orientation.y = math.sqrt(2) / 2
        control.orientation.z = 0
        control.interaction_mode = self.interaction_mode
        int_marker.controls.append(copy.deepcopy(control))

        # make a box which also moves in the plane
        markers = self.make_individual_markers(int_marker)
        for marker in markers:
            control.markers.append(marker)
        control.always_visible = True
        int_marker.controls.append(control)

        # we want to use our special callback function
        self.server.insert(int_marker, self.feedback)


class BallMarker(RobocupInteractiveMarker):
    def __init__(self, server):
        self.marker_name = "ball"
        self.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        self.publisher = rospy.Publisher("ball_relative", BallRelative, queue_size=1)
        super(BallMarker, self).__init__(server)

    def make_individual_markers(self, msg):
        marker = Marker()

        marker.type = Marker.SPHERE
        marker.scale.x = BALL_DIAMETER
        marker.scale.y = BALL_DIAMETER
        marker.scale.z = BALL_DIAMETER
        marker.pose.position.z = BALL_DIAMETER / 2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return (marker,)

    def publish_marker(self, e):
        # construct BallRelative message
        ball = BallRelative()
        ball.header.stamp = rospy.get_rostime()
        ball.header.frame_id = "map"
        ball.confidence = 1.0
        ball.ball_relative = self.pose.position

        # publish the new ball position
        if self.publish:
            self.publisher.publish(ball)


class GoalMarker(RobocupInteractiveMarker):
    def __init__(self, server):
        self.marker_name = "goal"
        self.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        self.publisher = rospy.Publisher("goal_relative", GoalRelative, queue_size=1)
        super(GoalMarker, self).__init__(server)

    def make_individual_markers(self, msg):
        lpost = Marker()
        lpost.type = Marker.CYLINDER
        lpost.scale = Vector3(POST_DIAMETER, POST_DIAMETER, GOAL_HEIGHT)
        lpost.color.r = 1.0
        lpost.color.g = 1.0
        lpost.color.b = 1.0
        lpost.color.a = 1.0
        lpost.pose.position = Point(0, GOAL_WIDTH / 2, GOAL_HEIGHT / 2)

        rpost = Marker()
        rpost.type = Marker.CYLINDER
        rpost.scale = Vector3(POST_DIAMETER, POST_DIAMETER, GOAL_HEIGHT)
        rpost.color.r = 1.0
        rpost.color.g = 1.0
        rpost.color.b = 1.0
        rpost.color.a = 1.0
        rpost.pose.position = Point(0, - GOAL_WIDTH / 2, GOAL_HEIGHT / 2)

        bar = Marker()
        bar.type = Marker.CYLINDER
        bar.scale = Vector3(POST_DIAMETER, POST_DIAMETER, GOAL_WIDTH)
        bar.color.r = 1.0
        bar.color.g = 1.0
        bar.color.b = 1.0
        bar.color.a = 1.0
        bar.pose.position = Point(0, 0, GOAL_HEIGHT)
        bar.pose.orientation = Quaternion(math.sqrt(2) / 2, 0, 0, math.sqrt(2) / 2)

        return (lpost, rpost, bar)

    def publish_marker(self, e):
        # construct GoalRelative message
        goal = GoalRelative()
        goal.header.stamp = rospy.get_rostime()
        goal.header.frame_id = "map"
        goal.confidence = 1.0
        # calculate the positions of the right and the left post
        orientation = euler_from_quaternion((self.pose.orientation.x, self.pose.orientation.y,
                                             self.pose.orientation.z, self.pose.orientation.w))
        angle = orientation[2]
        left_post = Point()
        left_post.x = self.pose.position.x - math.sin(angle) * GOAL_WIDTH / 2
        left_post.y = self.pose.position.y + math.cos(angle) * GOAL_WIDTH / 2

        right_post = Point()
        right_post.x =  self.pose.position.x + math.sin(angle) * GOAL_WIDTH / 2
        right_post.y = self.pose.position.y - math.cos(angle) * GOAL_WIDTH / 2

        goal.left_post = left_post
        goal.right_post = right_post

        # publish the new goal position
        if self.publish:
            self.publisher.publish(goal)


if __name__ == "__main__":
    rospy.init_node("humanoid_league_interactive_marker")

    # retrieve InteractiveMarkerServer and setup subscribers and publishers
    server = InteractiveMarkerServer("basic_controls")
    ball = BallMarker(server)
    goal = GoalMarker(server)

    server.applyChanges()

    # create a timer to update the published ball transform
    rospy.Timer(rospy.Duration(0.1), ball.publish_marker)
    rospy.Timer(rospy.Duration(0.1), goal.publish_marker)

    # run and block until finished
    rospy.spin()
