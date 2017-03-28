import os
import rospkg
from PyQt5 import Qt
from PyQt5 import QtCore
from PyQt5 import QtGui

import math
import rospy
from PyQt5.QtCore import QObject
from PyQt5.QtCore import QSize
from PyQt5.QtGui import QBrush
from PyQt5.QtGui import QColor
from PyQt5.QtGui import QIcon
from PyQt5.QtGui import QPen
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QGraphicsEllipseItem
from PyQt5.QtWidgets import QGraphicsPixmapItem
from PyQt5.QtWidgets import QGraphicsRectItem
from PyQt5.QtWidgets import QGraphicsScene
from PyQt5.QtWidgets import QWidget
from python_qt_binding import loadUi

from rqt_gui_py.plugin import Plugin

from humanoid_league_msgs.msg import BallRelative
from humanoid_league_msgs.msg import GoalRelative

from dynamic_reconfigure.server import Server


# from humanoid_league_msgs.cfg import field_rqt_params


class HumanoidLeagueRelativeRqt(Plugin):
    """ This class provides a rqt plugin which shows a 2d view of a RoboCup Soccer field.
     Different objects can be shown. Their position changing is handled in the respective callbacks."""

    def __init__(self, context):
        super(HumanoidLeagueRelativeRqt, self).__init__(context)
        self.setObjectName('2dField')

        # image values
        self.image_width = 1200.0
        self.image_height = 1200.0
        self.meter_length_img = 50
        self.scale = 1  # scaling factor between meters in reality and pixels on display

        # object values
        self.obstacle_size = 75
        self.obstacle_pen_width = 5
        self.ball_size = 50
        self.opacity = 0.75
        self.post_size = 50
        self.center_size = 30

        self.ball_active = True
        self.goal_active = True

        # initialize the UI
        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('humanoid_league_relative_rqt'), 'resource', 'relative.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('2dFieldUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        self._widget.resize_push_button.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.resize_push_button.pressed.connect(self.resize_field)

        # set drawing space
        self.view = self._widget.graphics_view
        self.view.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.view.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(QColor(255, 255, 255))

        # radar object
        rp = rospkg.RosPack()
        image_path = rp.get_path('humanoid_league_relative_rqt') + "/resource/radar.png"
        field_image = QPixmap(image_path)
        self.radar = QGraphicsPixmapItem(field_image)
        self.radar.setPos(0, 0)
        self._scene.addItem(self.radar)

        # ball
        self.ball_pen = QPen(QColor(255, 125, 0))
        self.ball_pen.setWidth(2)

        self.ball_brush = QBrush(QColor(255, 125, 0))
        self.ball = QGraphicsEllipseItem(0, 0, self.ball_size, self.ball_size, self.radar)
        self.ball.setPen(self.ball_pen)
        self.ball.setBrush(self.ball_brush)
        self.ball.setVisible(False)
        self.ball.setOpacity(self.opacity)

        # goal posts
        self.post_pen = QPen(QColor(200, 200, 0))
        self.post_pen.setWidth(2)
        self.post_brush = QBrush(QColor(200, 200, 0))

        self.left_post = QGraphicsEllipseItem(0, 0, self.post_size, self.post_size, self.radar)
        self.left_post.setPen(self.post_pen)
        self.left_post.setBrush(self.post_brush)
        self.left_post.setVisible(False)
        self.left_post.setOpacity(self.opacity)

        self.right_post = QGraphicsEllipseItem(0, 0, self.post_size, self.post_size, self.radar)
        self.right_post.setPen(self.post_pen)
        self.right_post.setBrush(self.post_brush)
        self.right_post.setVisible(False)
        self.right_post.setOpacity(self.opacity)

        # goal center
        self.center_pen = QPen(QColor(150, 150, 0))
        self.center_pen.setWidth(2)
        self.center_brush = QBrush(QColor(150, 150, 0))

        self.center = QGraphicsEllipseItem(0, 0, self.center_size, self.center_size, self.radar)
        self.center.setPen(self.center_pen)
        self.center.setBrush(self.center_brush)
        self.center.setVisible(False)
        self.center.setOpacity(self.opacity)

        # set the right positions and sizes
        self.resize_field()
        self.view.setScene(self._scene)

        # self.dyn_reconf = Server(field_rqt_params, self.reconfigure)

        rospy.Subscriber("ball_relative", BallRelative, self.ball_cb, queue_size=100)
        rospy.Subscriber("goal_relative", GoalRelative, self.goal_cb, queue_size=100)

        context.add_widget(self._widget)

    def reconfigure(self, config, level):
        # set visibilities accordingly
        self.ball_active = config["ball"]
        self.goal_active = config["goal"]

        # todo
        # config["obstacles"]
        # config["lines"]

    def resize_field(self):
        # fits the field into current window size
        size = self._widget.size()
        x_scale = size.width() / self.image_width
        y_scale = size.height() / self.image_height
        self.scale = min(x_scale, y_scale)
        self.radar.setScale(self.scale)
        self.view.centerOn(size.width() / 2, size.height() / 2)
        self.radar.offset()

    def set_scaled_position(self, item, x, y, height, width):
        item_scale = min(self.scale + 0.5, 1)
        item.setScale(item_scale)
        x *= self.meter_length_img  # scaling from meters to pixels on original image
        x += self.image_width / 2  # transform from upper left corner (qt coord system) to center point (robocup coord sys)
        x -= width * item_scale / 2
        x = max(min(x, self.image_width - 50), 0)  # dont let it get outside of the window

        y *= self.meter_length_img
        y += self.image_height / 2
        y -= height * item_scale / 2
        y = max(min(y, self.image_height - 50), 0)

        item.setX(y)
        item.setY(x)

    def ball_cb(self, msg):
        if msg.ball_relative is not None:
            self.set_scaled_position(self.ball, -1 * msg.ball_relative.x, -1 * msg.ball_relative.y, self.ball_size,
                                     self.ball_size)
        self.ball.setVisible(msg.confidence > 0 and self.ball_active)
        self.ball.setOpacity(msg.confidence)

    def goal_cb(self, msg):
        self.set_scaled_position(self.left_post, -1 * msg.left_post.x, -1 * msg.left_post.y, self.post_size,
                                 self.post_size)
        self.left_post.setVisible(msg.confidence > 0 and self.goal_active)
        self.left_post.setOpacity(msg.confidence)

        self.set_scaled_position(self.right_post, -1 * msg.right_post.x, -1 * msg.right_post.y, self.post_size,
                                 self.post_size)
        self.right_post.setVisible(msg.confidence > 0 and self.goal_active)
        self.right_post.setOpacity(msg.confidence)

        self.set_scaled_position(self.center, -1 * msg.center_direction.x, -1 * msg.center_direction.y, self.center_size,
                                 self.center_size)
        self.center.setOpacity(msg.confidence)
        self.center.setVisible(msg.confidence > 0 and self.goal_active)
