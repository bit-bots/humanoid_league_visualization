#!/usr/bin/env python3
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

import abc
import copy
import math

import numpy as np
import tf2_ros

import rclpy
from rclpy import logging
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from tf2_geometry_msgs import PointStamped
from tf_transformations import euler_from_quaternion

from soccer_vision_3d_msgs.msg import Ball, BallArray, Goalpost, GoalpostArray, Obstacle, ObstacleArray
from soccer_vision_attribute_msgs.msg import Confidence
from soccer_vision_attribute_msgs.msg import Goalpost as GoalpostAttr

BALL_DIAMETER = 0.13
GOAL_WIDTH = 1.5
GOAL_HEIGHT = 1.1
POST_DIAMETER = 0.1
OBSTACLE_NUMBER = 1
OBSTACLE_HEIGHT = 0.8
OBSTACLE_DIAMETER = 0.2

#todo this should not be hardcoded
CAMERA_INFO = {  # Updated to jacks camera info from wide angle camera
    'frame_id': "camera_optical_frame",
    'height': 1536,
    'width': 2048,
    'K': [
        1338.64532, 0.        , 1026.12387,
        0.        , 1337.89746, 748.42213 ,
        0.        , 0.        , 1.         ,
        ],
}

NODE_NAME = 'humanoid_league_interactive_marker'

logger = logging.get_logger(NODE_NAME)


class AbstractRobocupInteractiveMarker():

    def __init__(self, server, tf_buffer, marker_name, interaction_mode):
        self.server = server
        self.tf_buffer = tf_buffer
        self.marker_name = marker_name
        self.interaction_mode = interaction_mode

        self.pose = Pose()
        self.publish = True
        self.interactive_marker = None
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
        self.interactive_marker = InteractiveMarker()
        self.interactive_marker.header.frame_id = "map"
        self.interactive_marker.pose = self.pose
        self.interactive_marker.scale = 1.0

        self.interactive_marker.name = self.marker_name

        control = InteractiveMarkerControl()
        control.orientation.w = math.sqrt(2) / 2
        control.orientation.x = 0.0
        control.orientation.y = math.sqrt(2) / 2
        control.orientation.z = 0.0
        control.interaction_mode = self.interaction_mode
        self.interactive_marker.controls.append(copy.deepcopy(control))

        # make a box which also moves in the plane
        markers = self.make_individual_markers(self.interactive_marker)
        for marker in markers:
            control.markers.append(marker)
        control.always_visible = True
        self.interactive_marker.controls.append(control)

        # we want to use our special callback function
        self.server.insert(self.interactive_marker,
                           feedback_callback=self.feedback)

    @abc.abstractmethod
    def make_individual_markers(self, msg):
        ...


class BallMarker(AbstractRobocupInteractiveMarker):

    def __init__(self, server, tf_buffer, node):
        self.node = node
        self.absolute_publisher = self.node.create_publisher(
            BallArray, "balls_absolute", qos_profile=1)
        self.relative_publisher = self.node.create_publisher(
            BallArray, "balls_relative", qos_profile=1)
        super().__init__(server, tf_buffer, "ball",
                         InteractiveMarkerControl.MOVE_PLANE)
        self.pose.position.x = 1.0

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
        marker.color.a = 0.4

        return (marker, )

    def publish_marker(self):
        # construct PoseWithCertaintyArray() message for map frame
        ball_absolute = Ball()
        ball_absolute.center.x = self.pose.position.x
        ball_absolute.center.y = self.pose.position.y
        ball_absolute.center.z = self.pose.position.z
        ball_absolute.confidence = Confidence()

        balls_absolute = BallArray()
        balls_absolute.header.stamp = self.node.get_clock().now().to_msg()
        balls_absolute.header.frame_id = "map"
        balls_absolute.balls = [ball_absolute]

        # publish the new ball position
        if self.publish:
            self.absolute_publisher.publish(balls_absolute)

        # check if ball is also visible for the robot and publish on relative topic if this is the case
        try:
            ball_point_stamped = PointStamped()
            ball_point_stamped.header.stamp = self.node.get_clock().now().to_msg()
            ball_point_stamped.header.frame_id = "map"
            ball_point_stamped.point = ball_absolute.center
            ball_in_camera_optical_frame = self.tf_buffer.transform(
                ball_point_stamped,
                CAMERA_INFO["frame_id"],
                timeout=Duration(nanoseconds=500_000_000))  # Half a second
            if ball_in_camera_optical_frame.point.z >= 0:
                point = [
                    ball_in_camera_optical_frame.point.x,
                    ball_in_camera_optical_frame.point.y,
                    ball_in_camera_optical_frame.point.z
                ]
                k = np.reshape(CAMERA_INFO["K"], (3, 3))
                point_pixel = np.matmul(k, point)
                point_pixel = point_pixel * (1 / point_pixel[2])

                # make sure that the transformed pixel is inside the resolution and positive.
                if (0 < point_pixel[0] <= CAMERA_INFO["width"]
                        and 0 < point_pixel[1] <= CAMERA_INFO["height"]):
                    ball_in_footprint_frame = self.tf_buffer.transform(
                        ball_in_camera_optical_frame,
                        "base_footprint",
                        timeout=Duration(nanoseconds=500_000_000))  # Half a second
                    ball_relative = Ball()
                    ball_relative.center = ball_in_footprint_frame.point
                    ball_relative.confidence = Confidence()

                    balls_relative = BallArray()
                    balls_relative.header = ball_in_footprint_frame.header
                    balls_relative.balls = [ball_relative]
                    self.relative_publisher.publish(balls_relative)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as ex:
            logger.warning(str(ex), throttle_duration_sec=10.0)
            return


class GoalMarker(AbstractRobocupInteractiveMarker):

    def __init__(self, server, tf_buffer, node):
        self.node = node
        self.absolute_publisher = self.node.create_publisher(
            GoalpostArray, "goal_absolute", qos_profile=1)
        self.relative_publisher = self.node.create_publisher(
            GoalpostArray, "goal_relative", qos_profile=1)
        self.relative_posts_publisher = self.node.create_publisher(
            GoalpostArray, "goal_posts_relative", qos_profile=1)
        super().__init__(server, tf_buffer, "goal",
                         InteractiveMarkerControl.MOVE_ROTATE)
        self.pose.position.x = 3.0

    def make_individual_markers(self, msg):
        left_post = Marker()
        left_post.type = Marker.CYLINDER
        left_post.scale = Vector3(x=POST_DIAMETER,
                                  y=POST_DIAMETER,
                                  z=GOAL_HEIGHT)
        left_post.color.r = 1.0
        left_post.color.g = 1.0
        left_post.color.b = 1.0
        left_post.color.a = 0.4
        left_post.pose.position = Point(x=0.0,
                                        y=GOAL_WIDTH / 2,
                                        z=GOAL_HEIGHT / 2)

        right_post = Marker()
        right_post.type = Marker.CYLINDER
        right_post.scale = Vector3(x=POST_DIAMETER,
                                   y=POST_DIAMETER,
                                   z=GOAL_HEIGHT)
        right_post.color.r = 1.0
        right_post.color.g = 1.0
        right_post.color.b = 1.0
        right_post.color.a = 0.4
        right_post.pose.position = Point(x=0.0,
                                         y=-GOAL_WIDTH / 2,
                                         z=GOAL_HEIGHT / 2)

        bar = Marker()
        bar.type = Marker.CYLINDER
        bar.scale = Vector3(x=POST_DIAMETER, y=POST_DIAMETER, z=GOAL_WIDTH)
        bar.color.r = 1.0
        bar.color.g = 1.0
        bar.color.b = 1.0
        bar.color.a = 0.4
        bar.pose.position = Point(x=0.0, y=0.0, z=GOAL_HEIGHT)
        bar.pose.orientation = Quaternion(x=math.sqrt(2) / 2,
                                          y=0.0,
                                          z=0.0,
                                          w=math.sqrt(2) / 2)
        return (left_post, right_post, bar)

    def publish_marker(self):
        # construct GoalRelative message
        goal_absolute = GoalpostArray()
        goal_absolute.header.stamp = self.node.get_clock().now().to_msg()
        goal_absolute.header.frame_id = "map"
        # calculate the positions of the right and the left post
        orientation = euler_from_quaternion(
            (self.pose.orientation.x, self.pose.orientation.y,
             self.pose.orientation.z, self.pose.orientation.w))
        angle = orientation[2]
        left_post = Goalpost()
        left_post.bb.center.position.x = self.pose.position.x - math.sin(
            angle) * GOAL_WIDTH / 2
        left_post.bb.center.position.y = self.pose.position.y + math.cos(
            angle) * GOAL_WIDTH / 2
        left_post.attributes.side = GoalpostAttr.SIDE_LEFT
        left_post.confidence = Confidence()

        right_post = Goalpost()
        right_post.bb.center.position.x = self.pose.position.x + math.sin(
            angle) * GOAL_WIDTH / 2
        right_post.bb.center.position.y = self.pose.position.y - math.cos(
            angle) * GOAL_WIDTH / 2
        right_post.confidence = Confidence()
        right_post.attributes.side = GoalpostAttr.SIDE_RIGHT
        goal_absolute.posts.append(left_post)
        goal_absolute.posts.append(right_post)

        # publish the new goal position
        if self.publish:
            self.absolute_publisher.publish(goal_absolute)

        # check if goal is also visible for the robot and publish on relative topic if this is the case
        try:
            goal_relative = GoalpostArray()
            goal_relative.header.stamp = self.node.get_clock().now().to_msg()
            goal_relative.header.frame_id = "base_footprint"

            goal_left_point_stamped = PointStamped()
            goal_left_point_stamped.header.stamp = goal_relative.header.stamp
            goal_left_point_stamped.header.frame_id = "map"
            goal_left_point_stamped.point = left_post.bb.center.position
            left_post_in_camera_optical_frame = self.tf_buffer.transform(
                goal_left_point_stamped,
                CAMERA_INFO["frame_id"],
                timeout=Duration(nanoseconds=500_000_000))  # Half a second

            left_post_visible = False
            right_post_visible = False

            if left_post_in_camera_optical_frame.point.z >= 0:
                point = [
                    left_post_in_camera_optical_frame.point.x,
                    left_post_in_camera_optical_frame.point.y,
                    left_post_in_camera_optical_frame.point.z
                ]
                k = np.reshape(CAMERA_INFO["K"], (3, 3))
                point_pixel = np.matmul(k, point)
                point_pixel = point_pixel * (1 / point_pixel[2])

                if 0 < point_pixel[0] <= CAMERA_INFO[
                        "width"] and 0 < point_pixel[1] <= CAMERA_INFO[
                            "height"]:
                    left_post = Goalpost()
                    left_post.bb.center.position = self.tf_buffer.transform(
                        left_post_in_camera_optical_frame,
                        "base_footprint",
                        timeout=Duration(
                            nanoseconds=500_000_000)).point  # Half a second
                    goal_relative.posts.append(left_post)
                    left_post_visible = True

            goal_right_point_stamped = PointStamped()
            goal_right_point_stamped.header.stamp = goal_relative.header.stamp
            goal_right_point_stamped.header.frame_id = "map"
            goal_right_point_stamped.point = right_post.bb.center.position
            right_post_in_camera_optical_frame = self.tf_buffer.transform(
                goal_right_point_stamped,
                CAMERA_INFO["frame_id"],
                timeout=Duration(nanoseconds=500_000_000))  # Half a second
            if right_post_in_camera_optical_frame.point.z >= 0:
                point = [
                    right_post_in_camera_optical_frame.point.x,
                    right_post_in_camera_optical_frame.point.y,
                    right_post_in_camera_optical_frame.point.z
                ]
                k = np.reshape(CAMERA_INFO["K"], (3, 3))
                point_pixel = np.matmul(k, point)
                point_pixel = point_pixel * (1 / point_pixel[2])

                if 0 < point_pixel[0] <= (
                        CAMERA_INFO["width"]
                        and 0 < point_pixel[1] <= CAMERA_INFO["height"]):
                    right_post = Goalpost()
                    right_post.bb.center.position = self.tf_buffer.transform(
                        right_post_in_camera_optical_frame,
                        "base_footprint",
                        timeout=Duration(
                            nanoseconds=500_000_000)).point  # Half a second
                    goal_relative.posts.append(right_post)
                    right_post_visible = True

            # publish goal relative msg
            if right_post_visible or left_post_visible:
                self.relative_publisher.publish(goal_relative)
                self.relative_posts_publisher.publish(goal_relative)

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as ex:
            logger.warning(str(ex), throttle_duration_sec=10.0)
            return


class ObstacleMarker(AbstractRobocupInteractiveMarker):

    def __init__(self, server, tf_buffer, node, name):
        self.node = node
        self.type = 0  # unknown
        self.player_number = 0
        self.confidence = 1.0
        super().__init__(server, tf_buffer, name,
                         InteractiveMarkerControl.MOVE_PLANE)
        sub_menu_handle = self.menu_handler.insert("Color")
        h_mode_last = self.menu_handler.insert("red",
                                               parent=sub_menu_handle,
                                               callback=self.color_callback)
        h_mode_last = self.menu_handler.insert("blue",
                                               parent=sub_menu_handle,
                                               callback=self.color_callback)
        h_mode_last = self.menu_handler.insert("unknown",
                                               parent=sub_menu_handle,
                                               callback=self.color_callback)
        self.menu_handler.setCheckState(h_mode_last, MenuHandler.CHECKED)
        self.menu_handler.apply(self.server, self.marker_name)
        self.pose.position.x = 2.0

    def color_callback(self, feedback):
        item = feedback.menu_entry_id
        if self.menu_handler.getCheckState(item) == MenuHandler.CHECKED:
            # Unchecking something should lead to unknown color
            self.type = 0
            self.menu_handler.setCheckState(item, MenuHandler.UNCHECKED)
            self.menu_handler.setCheckState(5, MenuHandler.CHECKED)
            self.interactive_marker.controls[1].markers[0].type.r = 0.0
            self.interactive_marker.controls[1].markers[0].type.g = 0.0
            self.interactive_marker.controls[1].markers[0].type.b = 0.0
        else:
            if item == 3:
                self.type = 2
                self.menu_handler.setCheckState(4, MenuHandler.UNCHECKED)
                self.menu_handler.setCheckState(5, MenuHandler.UNCHECKED)
                self.interactive_marker.controls[1].markers[0].type.r = 1.0
                self.interactive_marker.controls[1].markers[0].type.g = 0.0
                self.interactive_marker.controls[1].markers[0].type.b = 0.0
            elif item == 4:
                self.type = 3
                self.menu_handler.setCheckState(3, MenuHandler.UNCHECKED)
                self.menu_handler.setCheckState(5, MenuHandler.UNCHECKED)
                self.interactive_marker.controls[1].markers[0].type.r = 0.0
                self.interactive_marker.controls[1].markers[0].type.g = 0.0
                self.interactive_marker.controls[1].markers[0].type.b = 1.0
            elif item == 5:
                self.type = 0
                self.menu_handler.setCheckState(3, MenuHandler.UNCHECKED)
                self.menu_handler.setCheckState(4, MenuHandler.UNCHECKED)
                self.interactive_marker.controls[1].markers[0].type.r = 0.0
                self.interactive_marker.controls[1].markers[0].type.g = 0.0
                self.interactive_marker.controls[1].markers[0].type.b = 0.0
            self.menu_handler.setCheckState(item, MenuHandler.CHECKED)

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def make_individual_markers(self, msg):
        marker = Marker()

        marker.type = Marker.CYLINDER
        marker.scale = Vector3(x=OBSTACLE_DIAMETER,
                               y=OBSTACLE_DIAMETER,
                               z=OBSTACLE_HEIGHT)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.4
        marker.pose.position = Point(x=0.0, y=0.0, z=OBSTACLE_HEIGHT / 2)
        return (marker, )

    def get_absolute_message(self):
        msg = Obstacle()
        msg.bb.center.position = self.pose.position
        msg.bb.size.z = OBSTACLE_HEIGHT
        msg.bb.size.x = OBSTACLE_DIAMETER/2
        msg.bb.size.y = OBSTACLE_DIAMETER/2
        msg.confidence = Confidence()
        return msg

    def get_relative_msg(self):
        # check if obstacle is also visible for the robot and only then return
        try:
            obstacle_point_stamped = PointStamped()
            obstacle_point_stamped.header.stamp = self.node.get_clock().now(
            ).to_msg()
            obstacle_point_stamped.header.frame_id = "map"
            obstacle_point_stamped.point = self.pose.position
            obstacle_in_camera_optical_frame = self.tf_buffer.transform(
                obstacle_point_stamped,
                CAMERA_INFO["frame_id"],
                timeout=Duration(nanoseconds=500_000_000))  # Half a second
            if obstacle_in_camera_optical_frame.point.z >= 0:
                point = [
                    obstacle_in_camera_optical_frame.point.x,
                    obstacle_in_camera_optical_frame.point.y,
                    obstacle_in_camera_optical_frame.point.z
                ]
                k = np.reshape(CAMERA_INFO["K"], (3, 3))
                point_pixel = np.matmul(k, point)
                point_pixel = point_pixel * (1 / point_pixel[2])

                # make sure that the transformed pixel is inside the resolution and positive.
                if (0 < point_pixel[0] <= CAMERA_INFO["width"]
                        and 0 < point_pixel[1] <= CAMERA_INFO["height"]):
                    obstacle_relative = Obstacle()
                    ball_in_footprint_frame = self.tf_buffer.transform(
                        obstacle_in_camera_optical_frame,
                        "base_footprint",
                        timeout=Duration(
                            nanoseconds=500_000_000))  # Half a second
                    obstacle_relative.bb.center.position = ball_in_footprint_frame.point
                    obstacle_relative.confidence = Confidence()
                    return obstacle_relative
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as ex:
            logger.warning(str(ex), throttle_duration_sec=10.0)
            return


class ObstacleMarkerArray:

    def __init__(self, server, tf_buffer, node):
        self.node = node
        self.absolute_publisher = self.node.create_publisher(
            ObstacleArray, "obstacles_absolute", qos_profile=1)
        self.relative_publisher = self.node.create_publisher(
            ObstacleArray, "obstacles_relative", qos_profile=1)
        self.obstacles = []
        for i in range(0, OBSTACLE_NUMBER):
            self.obstacles.append(
                ObstacleMarker(server, tf_buffer, node, f"obstacle_{i}"))

    def publish_marker(self):
        absolute_msg = ObstacleArray()
        absolute_msg.header.stamp = self.node.get_clock().now().to_msg()
        absolute_msg.header.frame_id = "map"
        absolute_obstacles = []
        relative_msg = ObstacleArray()
        relative_msg.header.stamp = self.node.get_clock().now().to_msg()
        relative_msg.header.frame_id = "base_footprint"
        relative_obstacles = []

        for obstacle in self.obstacles:
            # always publish on absolute
            absolute_obstacles.append(obstacle.get_absolute_message())

            rel_msg = obstacle.get_relative_msg()
            if rel_msg:
                relative_obstacles.append(rel_msg)

        absolute_msg.obstacles = absolute_obstacles
        relative_msg.obstacles = relative_obstacles

        self.absolute_publisher.publish(absolute_msg)
        self.relative_publisher.publish(relative_msg)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(NODE_NAME)
    server = InteractiveMarkerServer(node, "robocup")
    multi_executor = MultiThreadedExecutor()
    multi_executor.add_node(node)
    tf_buffer = tf2_ros.Buffer(Duration(seconds=30))
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)  # pylint: disable=unused-variable

    ball = BallMarker(server, tf_buffer, node)
    goal = GoalMarker(server, tf_buffer, node)
    obstacles = ObstacleMarkerArray(server, tf_buffer, node)
    
    server.applyChanges()

    # Create a timer to update the published ball transform
    node.create_timer(1.0, ball.publish_marker)
    node.create_timer(1.0, goal.publish_marker)
    node.create_timer(1.0, obstacles.publish_marker)

    try:
        multi_executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
