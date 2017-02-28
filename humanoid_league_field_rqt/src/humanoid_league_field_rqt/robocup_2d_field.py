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
from humanoid_league_msgs.msg import Position


class RoboCup2dField(Plugin):
    def __init__(self, context):
        super(RoboCup2dField, self).__init__(context)
        self.setObjectName('2dField')

        # field values
        self.field_length = 9
        self.field_width = 6
        self.image_width = 1100.0
        self.image_height = 800.0
        self.field_length_img = 900
        self.field_width_img = 600
        self.scale = 1  # scaling factor between meters in reality and pixels on display

        # object values
        self.robot_size = 75
        self.robot_pen_width = 5
        self.ball_size = 50

        # initialize the UI
        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('humanoid_league_field_rqt'), 'resource', '2dField.ui')
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

        # field object
        rp = rospkg.RosPack()
        image_path = rp.get_path('humanoid_league_field_rqt') + "/resource/field.png"
        field_image = QPixmap(image_path)
        self.field = QGraphicsPixmapItem(field_image)
        self.field.setPos(0, 0)
        self._scene.addItem(self.field)
        # robot
        self.robot = QGraphicsEllipseItem(0, 0, self.robot_size, self.robot_size, self.field)
        self.robot_brush = QBrush(QColor(255, 0, 0))
        self.robot.setBrush(self.robot_brush)
        self.robot_pen = QPen()
        self.robot_pen.setWidth(self.robot_pen_width)
        self.robot.setPen(self.robot_pen)
        self.robot.setVisible(False)
        #self._scene.addItem(self.robot)

        # ball
        self.ball = QGraphicsEllipseItem(0, 0, self.ball_size, self.ball_size, self.field)
        self.ball_pen = QPen(QColor(255, 165, 0))
        self.ball_pen.setWidth(2)
        self.ball.setPen(self.ball_pen)
        self.ball_brush = QBrush(QColor(255, 165, 0))
        self.ball.setBrush(self.ball_brush)
        self.ball.setVisible(False)
        #self._scene.addItem(self.ball)

        # set the right positions and sizes
        self.resize_field()
        self.view.setScene(self._scene)

        # todo implement the messages in the architecture (speak with wolves)
        # rospy.Subscriber("/local_model", LocalModel, self.local_model_update, queue_size=100)
        # rospy.Subscriber("/global_model", GlobalModel, self.global_model_update, queue_size=100)

        rospy.Subscriber("/local_position", Position, self.position_cb, queue_size=10)

        context.add_widget(self._widget)

    def get_center_point_xy(self):
        # to make it always fit to the current scale
        return (self.field_length / 2 * self.scale,
                self.field_width / 2 * self.scale)

    def resize_field(self):
        # fits the field into current window size
        size = self._widget.size()
        x_scale = size.width() / self.image_width
        y_scale = size.height() / self.image_height
        self.scale = min(x_scale, y_scale)
        self.field.setScale(self.scale)
        self.view.centerOn(size.width() / 2, size.height() / 2)
        self.field.offset()

    def set_scaled_position(self, item, x, y, height, width):
        item_scale = min(self.scale + 0.5, 1)
        item.setScale(item_scale)
        x *= self.field_length_img / self.field_length  # scaling from meters to pixels on original image
        x += self.image_width / 2  # transform from upper left corner (qt coord system) to center point (robocup coord sys)
        x -= width * item_scale / 2
        x = max(min(x, self.image_width - 50), 0)  # dont let it get outside of the window

        y *= self.field_width_img / self.field_width
        y += self.image_height / 2
        y -= height * item_scale / 2
        y = max(min(y, self.image_height - 50), 0)

        item.setX(x)
        item.setY(y)

        return x, y

    def position_cb(self, msg):
        self.set_scaled_position(self.robot, msg.pose.x, msg.pose.y, self.robot_size, self.robot_size)
        angle = math.degrees(msg.pose.theta) * 16  # to get to qt coordinate system
        # set orientation
        self.robot.setStartAngle(angle)
        self.robot.setSpanAngle(360 * 16 - 1)
        factor = msg.confidence * -1 + 1
        self.robot_brush.setColor(QColor(255, 200 * factor, 200 * factor))
        self.robot.setBrush(self.robot_brush)
        self.robot.setVisible(True)
