# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'LocationAddView.ui'
##
## Created by: Qt User Interface Compiler version 6.6.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QGraphicsView, QHBoxLayout, QLabel,
    QLineEdit, QPushButton, QSizePolicy, QTextEdit,
    QVBoxLayout, QWidget)
import res_rc

class Ui_LocationAddView(object):
    def setupUi(self, LocationAddView):
        if not LocationAddView.objectName():
            LocationAddView.setObjectName(u"LocationAddView")
        LocationAddView.resize(1024, 600)
        LocationAddView.setAcceptDrops(False)
        LocationAddView.setStyleSheet(u"\u314a")
        self.main_layout = QVBoxLayout(LocationAddView)
        self.main_layout.setSpacing(0)
        self.main_layout.setObjectName(u"main_layout")
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.content_widget = QWidget(LocationAddView)
        self.content_widget.setObjectName(u"content_widget")
        self.content_widget.setStyleSheet(u"background-color: #FFFFFF;")
        self.map_view = QGraphicsView(self.content_widget)
        self.map_view.setObjectName(u"map_view")
        self.map_view.setGeometry(QRect(20, 109, 991, 471))
        self.map_view.setMinimumSize(QSize(400, 400))
        self.map_view.setStyleSheet(u"QGraphicsView {\n"
"    background-color: #FFFFFF;\n"
"    border: 1px solid #333333;\n"
"    border-radius: 12px;\n"
"}")
        self.status_widget = QWidget(self.content_widget)
        self.status_widget.setObjectName(u"status_widget")
        self.status_widget.setGeometry(QRect(10, 10, 991, 91))
        self.callbell_button = QPushButton(self.status_widget)
        self.callbell_button.setObjectName(u"callbell_button")
        self.callbell_button.setGeometry(QRect(10, 10, 161, 45))
        self.callbell_button.setMinimumSize(QSize(120, 45))
        font = QFont()
        font.setFamilies([u"Noto Sans KR Black"])
        font.setBold(True)
        self.callbell_button.setFont(font)
        self.callbell_button.setCursor(QCursor(Qt.PointingHandCursor))
        self.callbell_button.setStyleSheet(u"QPushButton {\n"
"    background-color: #2cbb5d;\n"
"    color: white;\n"
"	font-family: \"Noto Sans KR Black\";\n"
"    border: none;\n"
"    border-radius: 8px;\n"
"    padding: 8px 16px;\n"
"    font-size: 16px;\n"
"    font-weight: bold;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #34cc6a;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #249d4e;\n"
"}\n"
"")
        self.destination_name_input = QLineEdit(self.status_widget)
        self.destination_name_input.setObjectName(u"destination_name_input")
        self.destination_name_input.setGeometry(QRect(180, 10, 191, 51))
        self.destination_name_input.setStyleSheet(u"QLineEdit {\n"
"    background-color: white;\n"
"    color: black;\n"
"    border: 1px solid #444444;\n"
"    border-radius: 8px;\n"
"    padding: 12px;\n"
"    font-size: 14px;\n"
"}")
        self.apply_area_btn = QPushButton(self.content_widget)
        self.apply_area_btn.setObjectName(u"apply_area_btn")
        self.apply_area_btn.setGeometry(QRect(830, 10, 80, 80))
        self.apply_area_btn.setStyleSheet(u"QPushButton {\n"
"    background-color: #2A2A2A;\n"
"    color: white;\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"    font-size: 18px;\n"
"	font-family: \"Noto Sans KR Black\";\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.cancel_area_btn = QPushButton(self.content_widget)
        self.cancel_area_btn.setObjectName(u"cancel_area_btn")
        self.cancel_area_btn.setGeometry(QRect(920, 10, 80, 80))
        self.cancel_area_btn.setStyleSheet(u"QPushButton {\n"
"    background-color: #2A2A2A;\n"
"    color: white;\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"    font-size: 18px;\n"
"	font-family: \"Noto Sans KR Black\";\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.map_control_widget = QWidget(self.content_widget)
        self.map_control_widget.setObjectName(u"map_control_widget")
        self.map_control_widget.setGeometry(QRect(750, 320, 261, 261))
        self.map_control_widget.setStyleSheet(u"QWidget {\n"
"    background-color: rgba(0, 0, 0, 0);\n"
"}\n"
"")
        self.object_zoom_in = QPushButton(self.map_control_widget)
        self.object_zoom_in.setObjectName(u"object_zoom_in")
        self.object_zoom_in.setGeometry(QRect(10, 90, 80, 80))
        self.object_zoom_in.setStyleSheet(u"QPushButton {\n"
"    background-image: url(:/file/MapControl/zoom_in.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgba(58, 58, 58, 200); /* hover \uc2dc \ud22c\uba85\ub3c4 */\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: rgba(51, 51, 51, 220); /* \ud074\ub9ad \uc2dc \ud22c\uba85\ub3c4 */\n"
"}\n"
"")
        self.map_right = QPushButton(self.map_control_widget)
        self.map_right.setObjectName(u"map_right")
        self.map_right.setGeometry(QRect(170, 170, 80, 80))
        self.map_right.setStyleSheet(u"QPushButton {\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    color: white;\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"    font-size: 18px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.map_left = QPushButton(self.map_control_widget)
        self.map_left.setObjectName(u"map_left")
        self.map_left.setGeometry(QRect(10, 170, 80, 80))
        self.map_left.setStyleSheet(u"QPushButton {\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    color: white;\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"    font-size: 18px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.map_up = QPushButton(self.map_control_widget)
        self.map_up.setObjectName(u"map_up")
        self.map_up.setGeometry(QRect(90, 90, 80, 80))
        self.map_up.setStyleSheet(u"QPushButton {\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    color: white;\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"    font-size: 18px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.map_down = QPushButton(self.map_control_widget)
        self.map_down.setObjectName(u"map_down")
        self.map_down.setGeometry(QRect(90, 170, 80, 80))
        self.map_down.setStyleSheet(u"QPushButton {\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    color: white;\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"    font-size: 18px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.map_move_refresh = QPushButton(self.map_control_widget)
        self.map_move_refresh.setObjectName(u"map_move_refresh")
        self.map_move_refresh.setGeometry(QRect(170, 10, 80, 80))
        self.map_move_refresh.setStyleSheet(u"QPushButton {\n"
"	background-image: url(:/file/Registration/map_view_refresh.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;    \n"
"	background-color: rgba(42, 42, 42, 180); \n"
"    color: white;\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"    font-size: 18px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.object_zoom_out = QPushButton(self.map_control_widget)
        self.object_zoom_out.setObjectName(u"object_zoom_out")
        self.object_zoom_out.setGeometry(QRect(170, 90, 80, 80))
        self.object_zoom_out.setStyleSheet(u"QPushButton {\n"
"	background-image: url(:/file/MapControl/zoom_out.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.label = QLabel(self.map_control_widget)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(10, 10, 101, 21))
        self.label.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 16px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.marker_control_widget = QWidget(self.content_widget)
        self.marker_control_widget.setObjectName(u"marker_control_widget")
        self.marker_control_widget.setGeometry(QRect(740, 380, 281, 211))
        self.marker_control_widget.setStyleSheet(u"QWidget {\n"
"    background-color: rgba(0, 0, 0, 0);\n"
"}\n"
"")
        self.robot_move_down = QPushButton(self.marker_control_widget)
        self.robot_move_down.setObjectName(u"robot_move_down")
        self.robot_move_down.setGeometry(QRect(100, 110, 80, 80))
        self.robot_move_down.setStyleSheet(u"QPushButton {\n"
"    background-image: url(:/file/MarkerMove/marker_move_down_50.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgba(58, 58, 58, 200); /* hover \uc2dc \ud22c\uba85\ub3c4 */\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: rgba(51, 51, 51, 220); /* \ud074\ub9ad \uc2dc \ud22c\uba85\ub3c4 */\n"
"}\n"
"")
        self.robot_move_right = QPushButton(self.marker_control_widget)
        self.robot_move_right.setObjectName(u"robot_move_right")
        self.robot_move_right.setGeometry(QRect(180, 110, 80, 80))
        self.robot_move_right.setStyleSheet(u"QPushButton {\n"
"    background-image: url(:/file/MarkerMove/marker_move_right_50.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgba(58, 58, 58, 200); /* hover \uc2dc \ud22c\uba85\ub3c4 */\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: rgba(51, 51, 51, 220); /* \ud074\ub9ad \uc2dc \ud22c\uba85\ub3c4 */\n"
"}\n"
"")
        self.robot_move_left = QPushButton(self.marker_control_widget)
        self.robot_move_left.setObjectName(u"robot_move_left")
        self.robot_move_left.setGeometry(QRect(20, 110, 80, 80))
        self.robot_move_left.setStyleSheet(u"QPushButton {\n"
"    background-image: url(:/file/MarkerMove/marker_move_left_50.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgba(58, 58, 58, 200); /* hover \uc2dc \ud22c\uba85\ub3c4 */\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: rgba(51, 51, 51, 220); /* \ud074\ub9ad \uc2dc \ud22c\uba85\ub3c4 */\n"
"}\n"
"")
        self.robot_move_up = QPushButton(self.marker_control_widget)
        self.robot_move_up.setObjectName(u"robot_move_up")
        self.robot_move_up.setGeometry(QRect(100, 30, 80, 80))
        self.robot_move_up.setStyleSheet(u"QPushButton {\n"
"    background-image: url(:/file/MarkerMove/marker_move_up_50.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgba(58, 58, 58, 200); /* hover \uc2dc \ud22c\uba85\ub3c4 */\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: rgba(51, 51, 51, 220); /* \ud074\ub9ad \uc2dc \ud22c\uba85\ub3c4 */\n"
"}\n"
"")
        self.robot_rotate_right = QPushButton(self.marker_control_widget)
        self.robot_rotate_right.setObjectName(u"robot_rotate_right")
        self.robot_rotate_right.setGeometry(QRect(180, 30, 80, 80))
        self.robot_rotate_right.setStyleSheet(u"QPushButton {\n"
"	background-image: url(:/file/MarkerMove/marker_rotate_right_50.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;    \n"
"	background-color: rgba(42, 42, 42, 180); \n"
"    color: white;\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"    font-size: 18px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.robot_rotate_left = QPushButton(self.marker_control_widget)
        self.robot_rotate_left.setObjectName(u"robot_rotate_left")
        self.robot_rotate_left.setGeometry(QRect(20, 30, 80, 80))
        self.robot_rotate_left.setStyleSheet(u"QPushButton {\n"
"	background-image: url(:/file/MarkerMove/marker_rotate_left_50.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;    \n"
"	background-color: rgba(42, 42, 42, 180); \n"
"    color: white;\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"    font-size: 18px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.label_2 = QLabel(self.marker_control_widget)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(20, 0, 121, 21))
        self.label_2.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 16px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.draw_area_widget = QWidget(self.content_widget)
        self.draw_area_widget.setObjectName(u"draw_area_widget")
        self.draw_area_widget.setGeometry(QRect(30, 480, 231, 91))
        self.draw_area_widget.setStyleSheet(u"QWidget {\n"
"    background-color: rgba(0, 0, 0, 0);\n"
"}\n"
"")
        self.restricted_area_button = QPushButton(self.draw_area_widget)
        self.restricted_area_button.setObjectName(u"restricted_area_button")
        self.restricted_area_button.setGeometry(QRect(0, 20, 70, 70))
        self.restricted_area_button.setStyleSheet(u"QPushButton {\n"
"	background-image: url(:/file/RegistSelect/restricted_50.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;\n"
"    background-color: #2A2A2A;\n"
"	background-color: rgba(255, 255, 255, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"	background-color: rgba(58, 58, 58, 200); /* hover \uc2dc \ud22c\uba85\ub3c4 */\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: rgba(51, 51, 51, 220); /* \ud074\ub9ad \uc2dc \ud22c\uba85\ub3c4 */\n"
"}\n"
"QPushButton:checked {\n"
"    background-color: rgba(255, 0, 0, 220); /* \ud074\ub9ad \uc2dc \ud22c\uba85\ub3c4 */\n"
"}")
        self.restricted_area_button.setCheckable(True)
        self.priority_path_area_button = QPushButton(self.draw_area_widget)
        self.priority_path_area_button.setObjectName(u"priority_path_area_button")
        self.priority_path_area_button.setGeometry(QRect(80, 20, 70, 70))
        self.priority_path_area_button.setStyleSheet(u"QPushButton {\n"
"	background-image: url(:/file/RegistSelect/priority_path_50.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;\n"
"    background-color: #2A2A2A;\n"
"	background-color: rgba(255, 255, 255, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"	background-color: rgba(58, 58, 58, 200); /* hover \uc2dc \ud22c\uba85\ub3c4 */\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: rgba(51, 51, 51, 220); /* \ud074\ub9ad \uc2dc \ud22c\uba85\ub3c4 */\n"
"}\n"
"QPushButton:checked {\n"
"    background-color: rgba(0, 0, 255, 220); /* \ud074\ub9ad \uc2dc \ud22c\uba85\ub3c4 */\n"
"}")
        self.priority_path_area_button.setCheckable(True)
        self.map_control_button = QPushButton(self.draw_area_widget)
        self.map_control_button.setObjectName(u"map_control_button")
        self.map_control_button.setGeometry(QRect(160, 20, 70, 70))
        self.map_control_button.setStyleSheet(u"QPushButton {\n"
"	background-image: url(:/file/MapControl/map_control.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;\n"
"    background-color: #2A2A2A;\n"
"	background-color: rgba(255, 255, 255, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"	background-color: rgba(58, 58, 58, 200); /* hover \uc2dc \ud22c\uba85\ub3c4 */\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: rgba(51, 51, 51, 220); /* \ud074\ub9ad \uc2dc \ud22c\uba85\ub3c4 */\n"
"}\n"
"QPushButton:checked {\n"
"    background-color: rgba(70, 130, 180, 220); /* \ud074\ub9ad \uc2dc \ud22c\uba85\ub3c4 */\n"
"}")
        self.map_control_button.setCheckable(True)
        self.label_3 = QLabel(self.draw_area_widget)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(0, 0, 71, 21))
        self.label_3.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 16px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.label_5 = QLabel(self.draw_area_widget)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(160, 0, 41, 21))
        self.label_5.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 16px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.move_area_widget = QWidget(self.content_widget)
        self.move_area_widget.setObjectName(u"move_area_widget")
        self.move_area_widget.setGeometry(QRect(580, 380, 431, 201))
        self.move_area_widget.setStyleSheet(u"QWidget {\n"
"    background-color: rgba(0, 0, 0, 0);\n"
"}\n"
"")
        self.object_move_up = QPushButton(self.move_area_widget)
        self.object_move_up.setObjectName(u"object_move_up")
        self.object_move_up.setGeometry(QRect(260, 30, 80, 80))
        self.object_move_up.setStyleSheet(u"QPushButton {\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    color: white;\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"    font-size: 18px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.object_move_down = QPushButton(self.move_area_widget)
        self.object_move_down.setObjectName(u"object_move_down")
        self.object_move_down.setGeometry(QRect(260, 110, 80, 80))
        self.object_move_down.setStyleSheet(u"QPushButton {\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    color: white;\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"    font-size: 18px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.object_move_left = QPushButton(self.move_area_widget)
        self.object_move_left.setObjectName(u"object_move_left")
        self.object_move_left.setGeometry(QRect(180, 110, 80, 80))
        self.object_move_left.setStyleSheet(u"QPushButton {\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    color: white;\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"    font-size: 18px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.object_move_right = QPushButton(self.move_area_widget)
        self.object_move_right.setObjectName(u"object_move_right")
        self.object_move_right.setGeometry(QRect(340, 110, 80, 80))
        self.object_move_right.setStyleSheet(u"QPushButton {\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    color: white;\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"    font-size: 18px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.grow_vertical = QPushButton(self.move_area_widget)
        self.grow_vertical.setObjectName(u"grow_vertical")
        self.grow_vertical.setGeometry(QRect(10, 30, 80, 80))
        self.grow_vertical.setStyleSheet(u"QPushButton {\n"
"	background-image: url(:/file/SetArea/grow_vertical.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"	background-color: rgba(58, 58, 58, 200); /* hover \uc2dc \ud22c\uba85\ub3c4 */\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.shrink_vertical = QPushButton(self.move_area_widget)
        self.shrink_vertical.setObjectName(u"shrink_vertical")
        self.shrink_vertical.setGeometry(QRect(10, 110, 80, 80))
        self.shrink_vertical.setStyleSheet(u"QPushButton {\n"
"	background-image: url(:/file/SetArea/shrink_vertical.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"	background-color: rgba(58, 58, 58, 200); /* hover \uc2dc \ud22c\uba85\ub3c4 */\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.grow_horizontal = QPushButton(self.move_area_widget)
        self.grow_horizontal.setObjectName(u"grow_horizontal")
        self.grow_horizontal.setGeometry(QRect(90, 30, 80, 80))
        self.grow_horizontal.setStyleSheet(u"QPushButton {\n"
"	background-image: url(:/file/SetArea/grow_horizontal.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;\n"
"    background-color: #2A2A2A;\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"	background-color: rgba(58, 58, 58, 200); /* hover \uc2dc \ud22c\uba85\ub3c4 */\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.shrink_horizontal = QPushButton(self.move_area_widget)
        self.shrink_horizontal.setObjectName(u"shrink_horizontal")
        self.shrink_horizontal.setGeometry(QRect(90, 110, 80, 80))
        self.shrink_horizontal.setStyleSheet(u"QPushButton {\n"
"	background-image: url(:/file/SetArea/shrink_horizontal.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;\n"
"    background-color: rgba(42, 42, 42, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"	background-color: rgba(58, 58, 58, 200); /* hover \uc2dc \ud22c\uba85\ub3c4 */\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.label_4 = QLabel(self.move_area_widget)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(10, 0, 121, 21))
        self.label_4.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 16px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.map_data_debug_TextEdit = QTextEdit(self.content_widget)
        self.map_data_debug_TextEdit.setObjectName(u"map_data_debug_TextEdit")
        self.map_data_debug_TextEdit.setGeometry(QRect(30, 110, 210, 51))
        self.map_data_debug_TextEdit.setStyleSheet(u"QTextEdit {\n"
"    background-color: transparent;\n"
"	border: None;\n"
"    color: Red;\n"
"    font-family: 'ial';\n"
"    font-size: 10px;\n"
"	font-family: \"Noto Sans KR Black\";\n"
"}")
        self.map_data_debug_TextEdit.setReadOnly(True)
        self.set_area_button = QPushButton(self.content_widget)
        self.set_area_button.setObjectName(u"set_area_button")
        self.set_area_button.setGeometry(QRect(740, 10, 80, 80))
        self.set_area_button.setMinimumSize(QSize(80, 45))
        self.set_area_button.setFont(font)
        self.set_area_button.setStyleSheet(u"QPushButton {\n"
"    background-color: #2A2A2A;\n"
"    color: white;\n"
"	font-family: \"Noto Sans KR Black\";\n"
"    font-size: 16px;\n"
"    font-weight: 500;\n"
"    border-radius: 8px;\n"
"    height: 45px;\n"
"    border: 1px solid #444444;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"    border: 1px solid #666666;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.set_area_button.setCheckable(True)
        self.register_button = QPushButton(self.content_widget)
        self.register_button.setObjectName(u"register_button")
        self.register_button.setGeometry(QRect(830, 10, 80, 80))
        self.register_button.setMinimumSize(QSize(0, 0))
        self.register_button.setFont(font)
        self.register_button.setStyleSheet(u"QPushButton {\n"
"    background-color: #2A2A2A;\n"
"    color: white;\n"
"	font-family: \"Noto Sans KR Black\";\n"
"    font-size: 16px;\n"
"    font-weight: 500;\n"
"    border-radius: 8px;\n"
"    height: 45px;\n"
"    border: none;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #2A2A2A;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #0080DD;\n"
"}")
        self.cancel_button = QPushButton(self.content_widget)
        self.cancel_button.setObjectName(u"cancel_button")
        self.cancel_button.setGeometry(QRect(920, 10, 80, 80))
        self.cancel_button.setMinimumSize(QSize(80, 80))
        self.cancel_button.setFont(font)
        self.cancel_button.setStyleSheet(u"QPushButton {\n"
"    background-color: #2A2A2A;\n"
"    color: white;\n"
"	font-family: \"Noto Sans KR Black\";\n"
"    font-size: 16px;\n"
"    font-weight: 500;\n"
"    border-radius: 8px;\n"
"    height: 45px;\n"
"    border: 1px solid #444444;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"    border: 1px solid #666666;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #333333;\n"
"}")
        self.horizontalLayoutWidget = QWidget(self.content_widget)
        self.horizontalLayoutWidget.setObjectName(u"horizontalLayoutWidget")
        self.horizontalLayoutWidget.setGeometry(QRect(20, 70, 801, 31))
        self.area_list_layout = QHBoxLayout(self.horizontalLayoutWidget)
        self.area_list_layout.setObjectName(u"area_list_layout")
        self.area_list_layout.setContentsMargins(0, 0, 0, 0)
        self.navigation_control_widget = QWidget(self.content_widget)
        self.navigation_control_widget.setObjectName(u"navigation_control_widget")
        self.navigation_control_widget.setGeometry(QRect(30, 480, 161, 91))
        self.navigation_control_widget.setStyleSheet(u"QWidget {\n"
"    background-color: rgba(0, 0, 0, 0);\n"
"}\n"
"")
        self.marker_control_type_button = QPushButton(self.navigation_control_widget)
        self.marker_control_type_button.setObjectName(u"marker_control_type_button")
        self.marker_control_type_button.setGeometry(QRect(0, 20, 70, 70))
        self.marker_control_type_button.setStyleSheet(u"QPushButton {\n"
"	background-image: url(:/file/Registration/select_marker_70.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;\n"
"    background-color: #2A2A2A;\n"
"	background-color: rgba(255, 255, 255, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"	background-color: rgba(58, 58, 58, 200); /* hover \uc2dc \ud22c\uba85\ub3c4 */\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: rgba(70, 130, 180, 180); /* \ud074\ub9ad \uc2dc \ud22c\uba85\ub3c4 */\n"
"}\n"
"QPushButton:checked {\n"
"    background-color: rgba(70, 130, 180, 180); /* \ud074\ub9ad \uc2dc \ud22c\uba85\ub3c4 */\n"
"}")
        self.marker_control_type_button.setCheckable(True)
        self.map_control_type_button = QPushButton(self.navigation_control_widget)
        self.map_control_type_button.setObjectName(u"map_control_type_button")
        self.map_control_type_button.setGeometry(QRect(80, 20, 70, 70))
        self.map_control_type_button.setStyleSheet(u"QPushButton {\n"
"	background-image: url(:/file/Registration/select_map_70.png);\n"
"    background-repeat: no-repeat;\n"
"    background-position: center;\n"
"    background-color: #2A2A2A;\n"
"	background-color: rgba(255, 255, 255, 180); /* \ud22c\uba85\ub3c4 \ucd94\uac00 */\n"
"    border: 1px solid #444444;\n"
"    border-radius: 6px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #3A3A3A;\n"
"	background-color: rgba(58, 58, 58, 200); /* hover \uc2dc \ud22c\uba85\ub3c4 */\n"
"    border: 1px solid #666666;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: rgba(70, 130, 180, 180); /* \ud074\ub9ad \uc2dc \ud22c\uba85\ub3c4 */\n"
"}\n"
"QPushButton:checked {\n"
"    background-color: rgba(70, 130, 180, 180); /* \ud074\ub9ad \uc2dc \ud22c\uba85\ub3c4 */\n"
"}")
        self.map_control_type_button.setCheckable(True)
        self.label_6 = QLabel(self.navigation_control_widget)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setGeometry(QRect(0, 0, 71, 21))
        self.label_6.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 16px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.label_7 = QLabel(self.navigation_control_widget)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setGeometry(QRect(80, 0, 71, 21))
        self.label_7.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 16px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.not_called_button = QPushButton(self.content_widget)
        self.not_called_button.setObjectName(u"not_called_button")
        self.not_called_button.setGeometry(QRect(120, 140, 161, 71))
        self.not_called_button.setMinimumSize(QSize(0, 0))
        self.not_called_button.setMaximumSize(QSize(1000, 1000))
        font1 = QFont()
        font1.setFamilies([u"Noto Sans KR Black"])
        self.not_called_button.setFont(font1)
        self.not_called_button.setStyleSheet(u"QPushButton {\n"
"    border-radius: 12px;                /* \ub465\uadfc \ud14c\ub450\ub9ac */\n"
"    background-color: white;          /* \ubc1d\uc740 \ud68c\uc0c9 \ubc30\uacbd */\n"
"    padding: 6px 12px;                  /* \ub0b4\ubd80 \uc5ec\ubc31 */\n"
"    text-align: left;                   /* \ud14d\uc2a4\ud2b8 \uc67c\ucabd \uc815\ub82c */\n"
"    padding-left: 10px;                 /* \uc544\uc774\ucf58 \uacf5\uac04 \ud655\ubcf4 */\n"
"	font-family: \"Noto Sans KR Black\";\n"
"    font-size: 18px;\n"
"    color: #000000;                     /* \uac80\uc815\uc0c9 \uae00\uc790 */\n"
"    qproperty-icon: url(:/file/CallBell/bell_off.png);\n"
"    qproperty-iconSize: 65px 65px;\n"
"}\n"
"")
        self.callbell_activated_button = QPushButton(self.content_widget)
        self.callbell_activated_button.setObjectName(u"callbell_activated_button")
        self.callbell_activated_button.setGeometry(QRect(120, 230, 181, 71))
        self.callbell_activated_button.setMinimumSize(QSize(0, 0))
        self.callbell_activated_button.setMaximumSize(QSize(1000, 1000))
        self.callbell_activated_button.setFont(font1)
        self.callbell_activated_button.setStyleSheet(u"QPushButton {\n"
"    border-radius: 12px;                /* \ub465\uadfc \ud14c\ub450\ub9ac */\n"
"    background-color: white;          /* \ubc1d\uc740 \ud68c\uc0c9 \ubc30\uacbd */\n"
"    padding: 6px 12px;                  /* \ub0b4\ubd80 \uc5ec\ubc31 */\n"
"    text-align: left;                   /* \ud14d\uc2a4\ud2b8 \uc67c\ucabd \uc815\ub82c */\n"
"    padding-left: 10px;                 /* \uc544\uc774\ucf58 \uacf5\uac04 \ud655\ubcf4 */\n"
"	font-family: \"Noto Sans KR Black\";\n"
"    font-size: 18px;\n"
"    color: #000000;                     /* \uac80\uc815\uc0c9 \uae00\uc790 */\n"
"    qproperty-icon: url(:/file/CallBell/bell_on_red.png);\n"
"    qproperty-iconSize: 65px 65px;\n"
"}\n"
"")
        self.status_widget.raise_()
        self.map_view.raise_()
        self.apply_area_btn.raise_()
        self.cancel_area_btn.raise_()
        self.map_control_widget.raise_()
        self.marker_control_widget.raise_()
        self.draw_area_widget.raise_()
        self.move_area_widget.raise_()
        self.map_data_debug_TextEdit.raise_()
        self.set_area_button.raise_()
        self.register_button.raise_()
        self.cancel_button.raise_()
        self.horizontalLayoutWidget.raise_()
        self.navigation_control_widget.raise_()
        self.not_called_button.raise_()
        self.callbell_activated_button.raise_()

        self.main_layout.addWidget(self.content_widget)


        self.retranslateUi(LocationAddView)

        QMetaObject.connectSlotsByName(LocationAddView)
    # setupUi

    def retranslateUi(self, LocationAddView):
        LocationAddView.setWindowTitle(QCoreApplication.translate("LocationAddView", u"Dentium", None))
        self.callbell_button.setText("")
        self.destination_name_input.setInputMask(QCoreApplication.translate("LocationAddView", u"q1", None))
        self.destination_name_input.setPlaceholderText(QCoreApplication.translate("LocationAddView", u"\uc774\ub984\uc744 \uc785\ub825\ud558\uc138\uc694", None))
        self.apply_area_btn.setText("")
        self.cancel_area_btn.setText("")
        self.object_zoom_in.setText("")
        self.map_right.setText("")
        self.map_left.setText("")
        self.map_up.setText("")
        self.map_down.setText("")
        self.map_move_refresh.setText("")
        self.object_zoom_out.setText("")
        self.label.setText(QCoreApplication.translate("LocationAddView", u"map control", None))
        self.robot_move_down.setText("")
        self.robot_move_right.setText("")
        self.robot_move_left.setText("")
        self.robot_move_up.setText("")
        self.robot_rotate_right.setText("")
        self.robot_rotate_left.setText("")
        self.label_2.setText(QCoreApplication.translate("LocationAddView", u"marker control", None))
        self.restricted_area_button.setText("")
        self.priority_path_area_button.setText("")
        self.map_control_button.setText("")
        self.label_3.setText(QCoreApplication.translate("LocationAddView", u"set area", None))
        self.label_5.setText(QCoreApplication.translate("LocationAddView", u"map", None))
        self.object_move_up.setText("")
        self.object_move_down.setText("")
        self.object_move_left.setText("")
        self.object_move_right.setText("")
        self.grow_vertical.setText("")
        self.shrink_vertical.setText("")
        self.grow_horizontal.setText("")
        self.shrink_horizontal.setText("")
        self.label_4.setText(QCoreApplication.translate("LocationAddView", u"area control", None))
        self.map_data_debug_TextEdit.setHtml(QCoreApplication.translate("LocationAddView", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Noto Sans KR Black'; font-size:10px; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'Gulim'; font-size:9pt; color:#00a3ff;\">X :</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'Gulim'; font-size:9pt; color:#00a3ff;\">Y :</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'Gulim'; font-size:9pt; color:#00a3ff;\">Yaw :</span></p></body></html>", None))
        self.set_area_button.setText("")
        self.register_button.setText("")
        self.cancel_button.setText("")
        self.marker_control_type_button.setText("")
        self.map_control_type_button.setText("")
        self.label_6.setText(QCoreApplication.translate("LocationAddView", u"marker", None))
        self.label_7.setText(QCoreApplication.translate("LocationAddView", u"map", None))
        self.not_called_button.setText(QCoreApplication.translate("LocationAddView", u"Not\n"
"Called", None))
        self.callbell_activated_button.setText(QCoreApplication.translate("LocationAddView", u"CallBell\n"
"Activated", None))
    # retranslateUi

