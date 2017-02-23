import rospy
import time

from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from humanoid_league_msgs.msg import BallRelative


class ShowRobocupObjects:
    def __init__(self):
        self.marker_publisher = rospy.Publisher("/robocup_markers", Marker, queue_size=10)

        # initilize message objects
        self.marker_ball_rel = Marker()  # type: Marker
        self.marker_ball_rel.header.frame_id = "base_link"
        self.marker_ball_rel.ns = "rel_ball"
        self.marker_ball_rel.id = 0
        self.marker_ball_rel.type = Marker.SPHERE
        self.marker_ball_rel.action = Marker.MODIFY
        self.ball_pose = Pose()
        scale = Vector3(1, 1, 1)
        self.marker_ball_rel.scale = scale
        color = ColorRGBA()
        color.r = 1.0
        color.a = 1.0
        self.marker_ball_rel.color = color
        self.marker_ball_rel.lifetime = 1

        # todo also display data from world model
        rospy.Subscriber("/ball_relative", self.ball_cb, queue_size=10)
        #rospy.Subscriber("/goal_relative", self.goal_cb, queue_size=10)
        #rospy.Subscriber("/obstacles_relative", self.obstacle_cb, queue_size=10)

        # we do everything in the callbacks
        rospy.spin()

    def ball_cb(self, msg: BallRelative):
        self.marker_ball_rel.header.stamp = rospy.Time.from_sec(time.time())

        self.ball_pose.position = msg.ball_relative
        self.marker_ball_rel.pose = self.ball_pose
