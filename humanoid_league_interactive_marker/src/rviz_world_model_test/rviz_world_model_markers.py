#!/usr/bin/env python

import rospy
import copy

import math
import random
import numpy as np

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from humanoid_league_msgs.msg import BallRelative, TeamData, Position2D
from geometry_msgs.msg import Pose2D, Pose, Point, PoseStamped, Vector3


class WorldModelMarkerTest:
    def __init__(self):
        self.server = InteractiveMarkerServer("basic_controls")
        self.team_data_pub = rospy.Publisher("team_data", TeamData, queue_size=1)
        self.player_marker_pub = rospy.Publisher("player_marker", Marker, queue_size=1)

        self.menu_handler = MenuHandler()
        self.publish_ball = True
        # no difference in filtering obstacles or opponents
        self.publish_obstacle = True
        self.obstacle_count = 1
        self.ball_pose = Pose()  # everything should be 0
        self.obstacle_poses = list()
        self.mate_poses = list()  # the mates include the observing player!

        mate_1 = Pose()
        mate_1.position = Point(0, 0, 0)
        self.mate_poses.append(mate_1)

        for i in range(self.obstacle_count):
            obstacle_pose = Pose()
            obstacle_pose.position.x = 0.1 + 0.1 * i
            self.obstacle_poses.append(obstacle_pose)

        #self.spawn_ball_marker()
        self.spawn_obstacle_markers()
        self.server.applyChanges()


        # create a timer to update the published ball transform
        rospy.Timer(rospy.Duration(0.1), self.pub_timer_callback)

        # run and block until finished
        rospy.spin()

    def pub_timer_callback(self, evt):
        self.publish_player_markers()
        detection_rate = .8
        td_msg = TeamData()

        # creating a dummy pose as placeholder in the team data message
        dummy_pose = Pose()
        dummy_pose.position.x = 1000
        dummy_pose.position.y = 1000
        dummy_pose.position.z = 1000

        letters = ('a', 'b', 'c', 'd')
        td_msg.robot_ids = list()
        for mate_id in range(len(self.mate_poses)):
            # add stuff in sight

            td_msg.robot_ids.append(mate_id)

            noisy_mate_pose = add_noise(self.mate_poses[mate_id], 5, 5, 5)  # TODO set noise everywhere to useful values

            td_msg.robot_positions.append(pose_to_pose2d(noisy_mate_pose))

            '''
            # dumb idea
            # TODO: broadcast pose with noise (other frames as the local filter!)
            t = geometry_msgs.msg.TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = 'mate_' + str(mate_id)
            t.transform.translation.x = mate_pose.position.x
            t.transform.translation.y = mate_pose.position.y
            t.transform.translation.z = 0.0
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, mate_pose.orientation.z)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            br.sendTransform(t)
            '''
            # TODO: ball!!!

            # TODO: transform poses to relative
            # publish obstacles as opponents
            for obstacle_id in range(self.obstacle_count):
                if randomly_in_sight(self.mate_poses[mate_id], self.obstacle_poses[obstacle_id], detection_rate):
                    obstacle_map_pose = self.obstacle_poses[obstacle_id]
                    obstacle_rel_pose = Pose()
                    obstacle_rel_pose.position.x = obstacle_map_pose.position.x - noisy_mate_pose.position.x
                    obstacle_rel_pose.position.y = obstacle_map_pose.position.y - noisy_mate_pose.position.y
                    obstacle_rel_pose.orientation.z = obstacle_map_pose.orientation.z - noisy_mate_pose.orientation.z
                    td_msg.__getattribute__('opponent_robot_' + letters[obstacle_id]).append(pose_to_position2d(add_noise(obstacle_rel_pose, 1, 1, 1)))
                else:
                    td_msg.__getattribute__('opponent_robot_' + letters[obstacle_id]).append(pose_to_position2d(dummy_pose))
            # publish mates
            mate_seen_count = 0
            for detected_mate_id in range(len(self.mate_poses)):
                # ignore ourselves
                if detected_mate_id == mate_id:
                    continue
                if randomly_in_sight(self.mate_poses[mate_id], self.mate_poses[detected_mate_id], detection_rate):
                    mate_map_pose = self.mate_poses[mate_id]
                    mate_rel_pose = Pose()
                    mate_rel_pose.position.x = mate_map_pose.position.x - noisy_mate_pose.position.x
                    mate_rel_pose.position.y = mate_map_pose.position.y - noisy_mate_pose.position.y
                    mate_rel_pose.orientation.z = mate_map_pose.orientation.z - self.mate_poses[mate_id].orientation.z
                    td_msg.__getattribute__('opponent_robot_' + letters[mate_seen_count]).append(pose_to_position2d(add_noise(mate_rel_pose, 1, 1, 1)))
                else:
                    td_msg.__getattribute__('opponent_robot_' + letters[mate_seen_count]).append(pose_to_position2d(dummy_pose))
                mate_seen_count += 1
        self.team_data_pub.publish(td_msg)

        # TODO: publish the markers directly as TeamData message! Apply noise!

    def spawn_ball_marker(self, position):
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
        self.server.insert(int_marker, self.ball_feedback_callback)

    def spawn_obstacle_markers(self):
        for i in range(self.obstacle_count):
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = "map"
            int_marker.pose = self.obstacle_poses[i]
            int_marker.scale = 1

            int_marker.name = "obstacle_" + str(i)

            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 1
            control.orientation.z = 0
            control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
            int_marker.controls.append(copy.deepcopy(control))

            # make a box which also moves in the plane
            control.markers.append(make_cube(int_marker))
            control.always_visible = True
            int_marker.controls.append(control)

            # we want to use our special callback function
            self.server.insert(int_marker, lambda f: self.obstacle_feedback_callback(f, i))

    def ball_feedback_callback(self, feedback):
        self.ball_pose = feedback.pose
        self.server.applyChanges()

    def obstacle_feedback_callback(self, feedback, obstacle_id):
        self.obstacle_poses[obstacle_id] = feedback.pose
        self.server.applyChanges()

    def publish_player_markers(self):
        distance = 9
        marker_msg = Marker()
        marker_msg.type = Marker.TRIANGLE_LIST
        marker_msg.action = Marker.ADD
        marker_msg.color.a = .6
        marker_msg.color.g = 1
        marker_msg.header.frame_id = 'map'
        marker_msg.scale = Vector3(1, 1, 1)

        for player in self.mate_poses:
            marker_msg.points.append(player.position)
            for k in [1, -1]:
                angle = player.orientation.z + k * math.radians(50)
                if angle < -math.pi:
                    angle += 2 * math.pi
                if angle > math.pi:
                    angle -= 2 * math.pi

                p = Point()
                p.x = player.position.x + distance * math.cos(angle)
                p.y = player.position.y + distance * math.sin(angle)
                marker_msg.points.append(p)



        self.player_marker_pub.publish(marker_msg)


def randomly_in_sight(observer_pose, object_pose, detection_chance):
    # type: (Pose, Pose, float) -> bool
    return in_sight(observer_pose, object_pose) if random.random() <= detection_chance else False


def in_sight(observer_pose, object_pose):
    # type: (Pose, Pose) -> bool

    # all angles in radian!
    fov = math.radians(100)  # setting the FOV to 100 degrees

    # calculate relative position of the object
    x_dist = object_pose.position.x - observer_pose.position.x
    y_dist = object_pose.position.y - observer_pose.position.y
    angle_global = math.atan2(y_dist, x_dist)
    angle_relative = angle_global - observer_pose.orientation.z  # this is wrong, but it isn't.
    if angle_relative < -math.pi:
        angle_relative += 2 * math.pi
    if angle_relative > math.pi:
        angle_relative -= 2 * math.pi

    return abs(angle_relative) <= fov / 2.0


def distance(pose_a, pose_b):
    # type: (Pose, Pose) -> float
    # assuming z=0
    x_dist = pose_a.position.x - pose_b.position.x
    y_dist = pose_a.position.y - pose_b.position.y
    return math.sqrt(x_dist ** 2 + y_dist ** 2)


def add_noise(in_pose, sigma_x, sigma_y, sigma_theta):
    # type: (Pose, float, float, float) -> Pose
    pose = copy.deepcopy(in_pose)
    x_offset = .1 * np.random.normal(0, sigma_x, 1)
    y_offset = .1 * np.random.normal(0, sigma_y, 1)
    theta_offset = .1 * np.random.normal(0, sigma_theta, 1)
    pose.position.x += x_offset
    pose.position.y += y_offset
    pose.orientation.z += theta_offset
    return pose


def make_sphere(msg):
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


def make_cube(msg):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.2
    marker.scale.y = msg.scale * 0.2
    marker.scale.z = msg.scale * 1
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    return marker


def stamp_now(pose, frame):
    # type: (Pose, str) -> PoseStamped
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = frame
    pose_stamped.header.stamp = rospy.get_rostime()
    return pose_stamped


def pose_to_pose2d(pose):
    # type: (Pose) -> Pose2D
    pose2d = Pose2D()
    pose2d.x = pose.position.x
    pose2d.y = pose.position.y
    pose2d.theta = pose.orientation.z
    return pose2d


def pose_to_position2d(pose):
    # type: (Pose) -> Position2D
    position2d = Position2D()
    position2d.confidence = 1
    position2d.pose = pose_to_pose2d(pose)
    return position2d


def menu_callback(feedback):
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
    rospy.init_node("humanoid_league_interactive_marker")

    WorldModelMarkerTest()
