#!/usr/bin/env python2.7
import copy
import threading
from collections import OrderedDict

import cv2
import os
import rospy
from dynamic_reconfigure.server import Server
from humanoid_league_msgs.msg import BallInImage, BallsInImage
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
#from humanoid_league_image_marker.cfg import image_marker_paramsConfig


def draw_ball(cv_img, ball):
    """Draws a circle on the image where the ball was seen."""
    i = [0, 0, 0]
    i[0] = int(ball.center.x)
    i[1] = int(ball.center.y)
    i[2] = int(ball.diameter / 2.0)
    c = (255, 0, 0)
    # outer circle
    cv2.circle(cv_img, (i[0], i[1]), i[2], c, 2)
    # circle center
    cv2.circle(cv_img, (i[0], i[1]), 2, (0, 0, 255), 3)


def draw_ball_candidates(cv_img, candidates):
    """Draws a list of ball candidates. The color is depending on the confidence."""
    if len(candidates) > 0:
        for can in candidates:
            i = [0, 0, 0]
            i[0] = int(can.center.x)
            i[1] = int(can.center.y)
            i[2] = int(can.diameter / 2.0)

            if can.confidence >= 0.5:
                c = (0, 255, 0)
            else:
                c = (0, 0, 255)
                # print(p)
                # draw the outer circle
            cv2.circle(cv_img, (i[0], i[1]), i[2], c, 2)
            # draw the center of the circle
            cv2.circle(cv_img, (i[0], i[1]), 2, (0, 0, 255), 3)


class ImageMarker:
    """This class starts a ROS node which takes images and draws recognized RoboCup Soccer objects on to it.
    Dynamic reconfigure is used to activate and deactive certain drawings."""

    def __init__(self):
        # todo implement final ball, goal posts and lines
        rospy.init_node("humanoid_league_image_marker")

        self.bridge = CvBridge()
        self.images = OrderedDict()
        self.ball_candidates = OrderedDict()
        self.balls = OrderedDict()

        self.img_lock = threading.Lock()
        self.can_lock = threading.Lock()

        self.ball_roi_active = True
        self.candidates_active = True
        self.ball_active = True
        self.goal_roi_active = True
        self.goal_posts_active = True
        self.goals_active = True
        self.obstacle_roi_active = True
        self.obstacles_active = True
        self.lines_roi_active = True
        self.line = True

        #self.server = Server(image_marker_paramsConfig, self.reconfigure)
        self.viz_publisher = rospy.Publisher("image_marker_image", Image, queue_size=10)
        rospy.Subscriber("image_raw", Image, self._image_cb, queue_size=10)
        rospy.Subscriber("ball_candidates", BallsInImage, self._candidates_cb, queue_size=10)

        self.run()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.img_lock:
                images = copy.deepcopy(self.images)

            for t in images.keys():  # imgages who are wating
                img = images.pop(t)  # get image from queue
                cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
                with self.can_lock:
                    if t in self.ball_candidates.keys():  # Check if all data to draw is there
                        print("test3")
                        candidates = self.ball_candidates.pop(t)
                        # ball = self.balls.pop(t)

                        if self.candidates_active:
                            rospy.logwarn("1")
                            draw_ball_candidates(cv_img, candidates)
                        # if self.ball_active:
                        #    draw_ball(cv_img, ball)

                        out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                        self.viz_publisher.publish(out_msg)
            rate.sleep()

    def _image_cb(self, msg):
        with self.img_lock:
            self.images[msg.header.stamp] = msg

            if len(self.images) >= 10:
                self.images.popitem(last=False)

    def _candidates_cb(self, msg):
        with self.can_lock:
            self.ball_candidates[msg.header.stamp] = msg.candidates
            if len(self.ball_candidates) > 50:
                self.ball_candidates.popitem(last=False)

    def reconfigure(self, config, level):

        self.ball_roi_active = config["ball_ROI"]
        self.candidates_active = config["ball_candidates"]
        self.ball_active = config["ball"]

        self.goal_roi_active = config["goal_post_ROI"]
        self.goal_posts_active = config["goal_posts"]
        self.goals_active = config["goals"]

        self.obstacle_roi_active = config["obstacle_ROI"]
        self.obstacles_active = config["obstacles"]

        self.lines_roi_active = config["line_ROI"]
        self.line = config["lines"]
        return config


if __name__ == "__main__":
    viz = ImageMarker()
