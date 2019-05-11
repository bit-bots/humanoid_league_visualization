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

from bitbots_msgs.msg import JointCommand
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from sensor_msgs.msg import JointState
from visualization_msgs.msg import *
from humanoid_league_msgs.msg import BallRelative, GoalRelative
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from tf.transformations import euler_from_quaternion

server = None
ball_menu_handler = MenuHandler()
goal_menu_handler = MenuHandler()
publish_balls = True
publish_goals = True

BALL_DIAMETER = 0.13
GOAL_WIDTH = 1.5
GOAL_HEIGHT = 1.1
POST_DIAMETER = 0.1


def ball_feedback(feedback):
    global ball_pose
    ball_pose = feedback.pose
    server.applyChanges()


def goal_feedback(feedback):
    global goal_pose
    goal_pose = feedback.pose
    server.applyChanges()


def make_sphere(msg):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x = BALL_DIAMETER
    marker.scale.y = BALL_DIAMETER
    marker.scale.z = BALL_DIAMETER
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    return marker


def make_goal(msg):
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
    bar.pose.orientation = Quaternion(0.77, 0, 0, 0.77)

    return (lpost, rpost, bar)


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
    control.markers.append(make_sphere(int_marker))
    control.always_visible = True
    int_marker.controls.append(control)

    # we want to use our special callback function
    server.insert(int_marker, ball_feedback)


def make_goal_marker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.pose.position = position
    int_marker.scale = 1

    int_marker.name = "goal"

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
    int_marker.controls.append(copy.deepcopy(control))

    lpost, rpost, bar = make_goal(int_marker)
    control.markers.append(lpost)
    control.markers.append(rpost)
    control.markers.append(bar)
    control.always_visible = True
    int_marker.controls.append(control)

    # we want to use our special callback function
    server.insert(int_marker, goal_feedback)


def pub_ball(e):
    global ball_pose, ball_pub

    # construct BallRelative message
    ball = BallRelative()
    ball.header.stamp = rospy.get_rostime()
    ball.header.frame_id = "map"
    ball.confidence = 1.0
    ball.ball_relative = ball_pose.position

    # publish the new ball position
    if publish_balls:
        ball_pub.publish(ball)


def pub_goal(e):
    global goal_pose, goal_pub

    # construct GoalRelative message
    goal = GoalRelative()
    goal.header.stamp = rospy.get_rostime()
    goal.header.frame_id = "map"
    goal.confidence = 1.0
    # calculate the positions of the right and the left post
    orientation = euler_from_quaternion((goal_pose.orientation.x, goal_pose.orientation.y,
                                         goal_pose.orientation.z, goal_pose.orientation.w))
    angle = orientation[2]
    left_post = Point()
    left_post.x = goal_pose.position.x - math.sin(angle) * GOAL_WIDTH / 2
    left_post.y = goal_pose.position.y + math.cos(angle) * GOAL_WIDTH / 2

    right_post = Point()
    right_post.x =  goal_pose.position.x + math.sin(angle) * GOAL_WIDTH / 2
    right_post.y = goal_pose.position.y - math.cos(angle) * GOAL_WIDTH / 2

    goal.left_post = left_post
    goal.right_post = right_post

    # publish the new goal position
    if publish_goals:
        goal_pub.publish(goal)


def ball_menu_callback(feedback):
    global publish_balls
    item = feedback.menu_entry_id
    if ball_menu_handler.getCheckState(item) == MenuHandler.CHECKED:
        ball_menu_handler.setCheckState(item, MenuHandler.UNCHECKED)
        publish_balls = False
    else:
        publish_balls = True
        ball_menu_handler.setCheckState(item, MenuHandler.CHECKED)

    ball_menu_handler.reApply(server)
    server.applyChanges()


def goal_menu_callback(feedback):
    global publish_goals
    item = feedback.menu_entry_id
    if goal_menu_handler.getCheckState(item) == MenuHandler.CHECKED:
        goal_menu_handler.setCheckState(item, MenuHandler.UNCHECKED)
        publish_goals = False
    else:
        publish_goals = True
        goal_menu_handler.setCheckState(item, MenuHandler.CHECKED)

    goal_menu_handler.reApply(server)
    server.applyChanges()


if __name__ == "__main__":
    global ball_pub, join_pos_pub, ball_pose, goal_pose
    rospy.init_node("humanoid_league_interactive_marker")

    # retrieve InteractiveMarkerServer and setup subscribers and publishers
    server = InteractiveMarkerServer("basic_controls")

    ball_pub = rospy.Publisher("ball_relative", BallRelative, queue_size=1)
    goal_pub = rospy.Publisher("goal_relative", GoalRelative, queue_size=1)

    ball_pose = Pose()
    ball_pose.position = Point(0, 0, 0)

    goal_pose = Pose()
    goal_pose.position = Point(0, 0, 0)

    make_ball_marker(ball_pose.position)
    make_goal_marker(goal_pose.position)

    item = ball_menu_handler.insert("publish ball", callback=ball_menu_callback)
    ball_menu_handler.setCheckState(item, MenuHandler.CHECKED)
    ball_menu_handler.apply(server, "ball")
    
    item = goal_menu_handler.insert("publish goal", callback=goal_menu_callback)
    goal_menu_handler.setCheckState(item, MenuHandler.CHECKED)
    goal_menu_handler.apply(server, "goal")

    server.applyChanges()

    # create a timer to update the published ball transform
    rospy.Timer(rospy.Duration(0.1), pub_ball)
    rospy.Timer(rospy.Duration(0.1), pub_goal)

    # run and block until finished
    rospy.spin()
