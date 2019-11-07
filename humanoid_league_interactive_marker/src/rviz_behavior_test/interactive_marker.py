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
import traceback

import rospy
import copy
import math

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from humanoid_league_msgs.msg import BallRelative, GoalRelative
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from tf2_geometry_msgs import PointStamped
from tf.transformations import euler_from_quaternion
import tf2_ros
import numpy as np

BALL_DIAMETER = 0.13
GOAL_WIDTH = 1.5
GOAL_HEIGHT = 1.1
POST_DIAMETER = 0.1

rospy.init_node("humanoid_league_interactive_marker")
tf_buffer = tf2_ros.Buffer(rospy.Duration(30))
tf_listener = tf2_ros.TransformListener(tf_buffer)


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
    def __init__(self, server, cam_info):
        self.cam_info = cam_info
        self.marker_name = "ball"
        self.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        self.absolute_publisher = rospy.Publisher("ball_absolute", BallRelative, queue_size=1)
        self.relative_publisher = rospy.Publisher("ball_relative", BallRelative, queue_size=1)
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
        # construct BallRelative message for map frame
        ball_absolute = BallRelative()
        ball_absolute.header.stamp = rospy.get_rostime()
        ball_absolute.header.frame_id = "map"
        ball_absolute.confidence = 1.0
        ball_absolute.ball_relative = self.pose.position

        # publish the new ball position
        if self.publish:
            self.absolute_publisher.publish(ball_absolute)

        # check if ball is also visible for the robot and publish on relative topic if this is the case
        # only works if camera info is provided
        if self.cam_info:
            try:
                ball_point_stamped = PointStamped()
                ball_point_stamped.header.stamp = rospy.Time.now()
                ball_point_stamped.header.frame_id = "map"
                ball_point_stamped.point = ball_absolute.ball_relative
                ball_in_camera_optical_frame = tf_buffer.transform(ball_point_stamped, self.cam_info["frame_id"],
                                                                   timeout=rospy.Duration(0.5))
                if ball_in_camera_optical_frame.point.z >= 0:
                    p = [ball_in_camera_optical_frame.point.x, ball_in_camera_optical_frame.point.y,
                         ball_in_camera_optical_frame.point.z]
                    k = np.reshape(self.cam_info["K"], (3, 3))
                    p_pixel = np.matmul(k, p)
                    p_pixel = p_pixel * (1 / p_pixel[2])

                    # make sure that the transformed pixel is inside the resolution and positive.
                    if 0 < p_pixel[0] <= self.cam_info["width"] and 0 < p_pixel[1] <= self.cam_info["height"]:
                        ball_relative = BallRelative()
                        ball_relative.header.stamp = ball_in_camera_optical_frame.header.stamp
                        ball_relative.header.frame_id = "base_footprint"
                        ball_in_footprint_frame = tf_buffer.transform(ball_in_camera_optical_frame, "base_footprint",
                                                                      timeout=rospy.Duration(0.5))
                        ball_relative.header = ball_in_footprint_frame.header
                        ball_relative.ball_relative = ball_in_footprint_frame.point
                        ball_relative.confidence = 1.0
                        self.relative_publisher.publish(ball_relative)
            except Exception as e:
                rospy.logwarn_throttle(10, "Necessary transforms for relative ball are not available yet.")


class GoalMarker(RobocupInteractiveMarker):
    def __init__(self, server, cam_info):
        self.cam_info = cam_info
        self.marker_name = "goal"
        self.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        self.absolute_publisher = rospy.Publisher("goal_absolute", GoalRelative, queue_size=1)
        self.relative_publisher = rospy.Publisher("goal_relative", GoalRelative, queue_size=1)
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
        goal_absolute = GoalRelative()
        goal_absolute.header.stamp = rospy.get_rostime()
        goal_absolute.header.frame_id = "map"
        goal_absolute.confidence = 1.0
        # calculate the positions of the right and the left post
        orientation = euler_from_quaternion((self.pose.orientation.x, self.pose.orientation.y,
                                             self.pose.orientation.z, self.pose.orientation.w))
        angle = orientation[2]
        left_post = Point()
        left_post.x = self.pose.position.x - math.sin(angle) * GOAL_WIDTH / 2
        left_post.y = self.pose.position.y + math.cos(angle) * GOAL_WIDTH / 2

        right_post = Point()
        right_post.x = self.pose.position.x + math.sin(angle) * GOAL_WIDTH / 2
        right_post.y = self.pose.position.y - math.cos(angle) * GOAL_WIDTH / 2

        goal_absolute.left_post = left_post
        goal_absolute.right_post = right_post

        # publish the new goal position
        if self.publish:
            self.absolute_publisher.publish(goal_absolute)

        # check if goal is also visible for the robot and publish on relative topic if this is the case
        # only works if camera info is provided
        if self.cam_info:
            try:
                goal_relative = GoalRelative()
                goal_relative.header.stamp = rospy.Time.now()
                goal_relative.header.frame_id = "base_footprint"

                goal_left_point_stamped = PointStamped()
                goal_left_point_stamped.header.stamp = goal_relative.header.stamp
                goal_left_point_stamped.header.frame_id = "map"
                goal_left_point_stamped.point = left_post
                left_post_in_camera_optical_frame = tf_buffer.transform(goal_left_point_stamped, self.cam_info["frame_id"],
                                                                        timeout=rospy.Duration(0.5))

                lpost_visible = False
                rpost_visible = False

                if left_post_in_camera_optical_frame.point.z >= 0:
                    p = [left_post_in_camera_optical_frame.point.x, left_post_in_camera_optical_frame.point.y,
                         left_post_in_camera_optical_frame.point.z]
                    k = np.reshape(self.cam_info["K"], (3, 3))
                    p_pixel = np.matmul(k, p)
                    p_pixel = p_pixel * (1 / p_pixel[2])

                    if 0 < p_pixel[0] <= self.cam_info["width"] and 0 < p_pixel[1] <= self.cam_info["height"]:
                        goal_relative.left_post = tf_buffer.transform(left_post_in_camera_optical_frame, "base_footprint",timeout=rospy.Duration(0.5)).point
                        lpost_visible = True

                goal_right_point_stamped = PointStamped()
                goal_right_point_stamped.header.stamp = goal_relative.header.stamp
                goal_right_point_stamped.header.frame_id = "map"
                goal_right_point_stamped.point = right_post
                right_post_in_camera_optical_frame = tf_buffer.transform(goal_right_point_stamped, self.cam_info["frame_id"],
                                                                         timeout=rospy.Duration(0.5))
                if right_post_in_camera_optical_frame.point.z >= 0:
                    p = [right_post_in_camera_optical_frame.point.x, right_post_in_camera_optical_frame.point.y,
                         right_post_in_camera_optical_frame.point.z]
                    k = np.reshape(self.cam_info["K"], (3, 3))
                    p_pixel = np.matmul(k, p)
                    p_pixel = p_pixel * (1 / p_pixel[2])

                    if 0 < p_pixel[0] <= self.cam_info["width"] and 0 < p_pixel[1] <= self.cam_info["height"]:
                        goal_relative.right_post = tf_buffer.transform(right_post_in_camera_optical_frame, "base_footprint",
                                                              timeout=rospy.Duration(0.5)).point
                        rpost_visible = True

                if rpost_visible or lpost_visible:
                    if not lpost_visible:
                        goal_relative.left_post = goal_relative.right_post
                    elif not rpost_visible:
                        goal_relative.right_post = goal_relative.left_post
                    goal_relative.confidence = 1
                    self.relative_publisher.publish(goal_relative)
            except Exception as e:
                rospy.logwarn_throttle(10, "Necessary transforms for relative goal are not available yet.")


if __name__ == "__main__":

    # get camera info
    cam_info = rospy.get_param("/camera_info", None)
    if not cam_info:
        rospy.logwarn("No camera info provided from parameter server. Will only publish absolute messages!")
    # retrieve InteractiveMarkerServer and setup subscribers and publishers
    server = InteractiveMarkerServer("basic_controls")
    ball = BallMarker(server, cam_info)
    goal = GoalMarker(server, cam_info)

    server.applyChanges()

    # create a timer to update the published ball transform
    rospy.Timer(rospy.Duration(0.1), ball.publish_marker)
    rospy.Timer(rospy.Duration(0.1), goal.publish_marker)

    # run and block until finished
    rospy.spin()
