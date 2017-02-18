import os
import rospkg
from PyQt5 import Qt
from PyQt5 import QtCore
from PyQt5 import QtGui

import rospy
from PyQt5.QtCore import QObject
from PyQt5.QtCore import QSize
from PyQt5.QtGui import QColor
from PyQt5.QtGui import QIcon
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QGraphicsEllipseItem
from PyQt5.QtWidgets import QGraphicsPixmapItem
from PyQt5.QtWidgets import QGraphicsRectItem
from PyQt5.QtWidgets import QGraphicsScene
from PyQt5.QtWidgets import QWidget
from python_qt_binding import loadUi

from rqt_gui_py.plugin import Plugin


class RoboCup2dField(Plugin):
    # todo add buttons to control what shall be displayed on field
    # todo display arroaws pointing outside of the window if an object can not be displayed on it

    def __init__(self, context):
        super(RoboCup2dField, self).__init__(context)
        self.setObjectName('2dField')

        # load params
        self.field_length = rospy.get_param("/2d_field/field_length", 9)
        self.field_width = rospy.get_param("/2d_field/field_with", 6)
        # todo will be dynamic due to resize
        self.scale = 50  # scaling factor between meters in reality and pixels on display
        self.image_width = 1131.0
        self.image_height = 790.0

        # initialize the UI
        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('robocup_2d_field'), 'resource', '2dField.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('2dFieldUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # prepare field objects
        self.view = self._widget.graphics_view
        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(QColor(255, 255, 255))
        field_image = QPixmap("/home/keks/repositories/humanoid_league_rqt/robocup_2d_field/resource/field.png")
        self.field = QGraphicsPixmapItem(field_image)
        self.field.setPos(10, 10)
        self._scene.addItem(self.field)

        self.ball = QGraphicsEllipseItem(0, 0, 10, 10)
        # self._scene.addItem(self.ball)

        self.view.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.view.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.view.setScene(self._scene)

        # set the right positions and sizes

        self.resize_field()

        self._widget.resize_push_button.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.resize_push_button.pressed.connect(self.resize_field)

        # todo implement the messages in the architecture (speak with wolves)
        # rospy.Subscriber("/local_model", LocalModel, self.local_model_update, queue_size=100)
        # rospy.Subscriber("/global_model", GlobalModel, self.global_model_update, queue_size=100)

        context.add_widget(self._widget)

    def get_center_point_xy(self):
        # to make it always fit to the current scale
        return (self.field_length / 2 * self.scale,
                self.field_width / 2 * self.scale)

    def resize_field(self):
        # fits the field into current window size
        size = self._widget.size()
        x_scale = (size.width() - 40) / self.image_width
        y_scale = (size.height() - 60) / self.image_height
        self.scale = min(x_scale, y_scale)
        rospy.logwarn(self.scale)
        self.field.setScale(self.scale)
        self.view.centerOn(size.width() / 2, size.height() / 2)

    def set_robot_position(self):