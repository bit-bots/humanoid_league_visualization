#!/usr/bin/env python2

import rospy
import time
import math

import tf2_ros as tf2
from tf2_geometry_msgs import PointStamped
import tf_conversions
from geometry_msgs.msg import Pose, Vector3, Pose2D
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from humanoid_league_msgs.msg import BallRelative, GoalRelative, ObstaclesRelative, TeamData, Position2D


class TeamDataVisualizer:

    def __init__(self):
        rospy.init_node("show_robocup_objects")
        self.marker_publisher = rospy.Publisher("/team_data_markers", MarkerArray, queue_size=10)

        rospy.Subscriber("/team_data", TeamData, self.team_data_callback, queue_size=10)

        rospy.spin()

    def team_data_callback(self, td_msg):
        # type: (TeamData) -> None
        marker_array = MarkerArray()
        for i in range(len(td_msg.robot_ids)):
            marker_list = list()

            # handle robot detections
            robot_pose = td_msg.robot_positions[i]
            if robot_pose.x > 20:
                continue
            if i < len(td_msg.ball_relative) and td_msg.ball_relative[i].x < 20:
                marker_list.append(self.render_ball_marker(self.transform_to_pose2d(td_msg.ball_relative[i], robot_pose)))
            for r in ['a', 'b', 'c', 'd']:
                robot_detections = td_msg.__getattribute__('opponent_robot_' + r)
                if not i < len(robot_detections):
                    break
                robot_detection = robot_detections[i]
                if robot_detection.pose.x < 20:
                    marker_list.append(self.render_opponent_marker(self.transform_to_pose2d(robot_detection, robot_pose)))
            for r in ['a', 'b', 'c']:
                robot_detections = td_msg.__getattribute__('team_robot_' + r)
                if not i < len(robot_detections):
                    break
                robot_detection = robot_detections[i]
                if robot_detection.pose.x < 20:
                    marker_list.append(self.render_mate_marker(self.transform_to_pose2d(robot_detection, robot_pose)))

            for m in range(len(marker_list)):
                marker = marker_list[m]
                marker.ns = 'robot_' + str(i)
                marker.id = i * 100 + m
            marker_array.markers += marker_list

        self.marker_publisher.publish(marker_array)

    def render_opponent_marker(self, pose2d):
        # type: (Pose2D) -> Marker
        return self.render_robot_marker(pose2d, ColorRGBA(1, 0, 0, .8))

    def render_mate_marker(self, pose2d):
        # type: (Pose2D) -> Marker
        return self.render_robot_marker(pose2d, ColorRGBA(0, 1, 0, .8))

    def render_robot_marker(self, pose2d, color):
        # type: (Pose2D, ColorRGBA) -> Marker
        marker = Marker()
        marker.color = color
        marker.header.frame_id = 'map'
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .8
        marker.pose.position.z = marker.scale.z / 2.0
        marker.type = Marker.CUBE
        marker.lifetime = rospy.rostime.Duration(.2)
        marker.action = Marker.ADD
        marker.pose.position.x = pose2d.x
        marker.pose.position.y = pose2d.y

        return marker

    def render_ball_marker(self, pose2d):
        # type: (Pose2D) -> Marker
        marker = Marker()
        marker.color = ColorRGBA(1, 1, 1, .8)
        marker.header.frame_id = 'map'
        marker.scale.x = .13
        marker.scale.y = .13
        marker.scale.z = .13
        marker.pose.position.z = marker.scale.z / 2.0
        marker.type = Marker.SPHERE
        marker.lifetime = rospy.rostime.Duration(.2)
        marker.action = Marker.ADD
        marker.pose.position.x = pose2d.x
        marker.pose.position.y = pose2d.y

        return marker

    def transform_to_pose2d(self, detection, observer):
        # type: (Position2D, Pose2D) -> Pose2D
        pose = Pose2D()
        # detection to planar coordinate
        r = math.sqrt(detection.pose.x ** 2 + detection.pose.y ** 2)
        d = math.atan2(detection.pose.y, detection.pose.x)
        # add angle of detecting player
        d += observer.theta
        if d < -math.pi:
            d += 2 * math.pi
        if d > math.pi:
            d -= 2 * math.pi
        # back to Cartesian coordinates...
        pose.x = r * math.cos(d)
        pose.y = r * math.sin(d)
        # add observer coordinates
        pose.x += observer.x
        pose.y += observer.y

        return pose

if __name__ == "__main__":
    marker_node = TeamDataVisualizer()
