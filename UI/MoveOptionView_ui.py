# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'MoveOptionView.ui'
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
from PySide6.QtWidgets import (QApplication, QCheckBox, QFrame, QLabel,
    QPushButton, QScrollArea, QSizePolicy, QSlider,
    QTextEdit, QVBoxLayout, QWidget)
import res_rc

class Ui_MoveOptionView(object):
    def setupUi(self, MoveOptionView):
        if not MoveOptionView.objectName():
            MoveOptionView.setObjectName(u"MoveOptionView")
        MoveOptionView.resize(1024, 600)
        MoveOptionView.setStyleSheet(u"background-color: #141414;")
        self.verticalLayout = QVBoxLayout(MoveOptionView)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.status_bar_placeholder = QWidget(MoveOptionView)
        self.status_bar_placeholder.setObjectName(u"status_bar_placeholder")
        self.status_bar_placeholder.setMinimumSize(QSize(0, 60))

        self.verticalLayout.addWidget(self.status_bar_placeholder)

        self.scrollArea = QScrollArea(MoveOptionView)
        self.scrollArea.setObjectName(u"scrollArea")
        self.scrollArea.setStyleSheet(u"QScrollArea {\n"
"    background-color: #141414;\n"
"    border: none;\n"
"}\n"
"QScrollBar:vertical {\n"
"    background-color: #222222;\n"
"    width: 10px;\n"
"    margin: 0px;\n"
"}\n"
"QScrollBar::handle:vertical {\n"
"    background-color: #666666;\n"
"    min-height: 20px;\n"
"    border-radius: 5px;\n"
"}\n"
"QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {\n"
"    height: 0px;\n"
"}")
        self.scrollArea.setFrameShape(QFrame.NoFrame)
        self.scrollArea.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.scrollArea.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scrollArea.setWidgetResizable(True)
        self.scrollAreaWidgetContents = QWidget()
        self.scrollAreaWidgetContents.setObjectName(u"scrollAreaWidgetContents")
        self.scrollAreaWidgetContents.setGeometry(QRect(0, -260, 1014, 1200))
        self.scrollAreaWidgetContents.setMinimumSize(QSize(0, 1200))
        self.scrollAreaWidgetContents.setStyleSheet(u"background-color: #FFFFFF;")
        self.title_label = QLabel(self.scrollAreaWidgetContents)
        self.title_label.setObjectName(u"title_label")
        self.title_label.setGeometry(QRect(360, 0, 286, 70))
        font = QFont()
        font.setFamilies([u"Noto Sans KR Black"])
        font.setBold(True)
        self.title_label.setFont(font)
        self.title_label.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 22px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.subtitle_label = QLabel(self.scrollAreaWidgetContents)
        self.subtitle_label.setObjectName(u"subtitle_label")
        self.subtitle_label.setGeometry(QRect(10, 60, 281, 35))
        self.subtitle_label.setFont(font)
        self.subtitle_label.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 22px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.subtitle_label.setAlignment(Qt.AlignCenter)
        self.robot_velo_set_label = QLabel(self.scrollAreaWidgetContents)
        self.robot_velo_set_label.setObjectName(u"robot_velo_set_label")
        self.robot_velo_set_label.setGeometry(QRect(10, 200, 142, 35))
        self.robot_velo_set_label.setFont(font)
        self.robot_velo_set_label.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 22px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.robot_velo_set_label.setAlignment(Qt.AlignCenter)
        self.linear_speed_usr_textEdit = QTextEdit(self.scrollAreaWidgetContents)
        self.linear_speed_usr_textEdit.setObjectName(u"linear_speed_usr_textEdit")
        self.linear_speed_usr_textEdit.setGeometry(QRect(860, 390, 151, 51))
        self.linear_speed_usr_textEdit.setStyleSheet(u"QTextEdit {\n"
"    background-color: transparent;\n"
"	border: None;\n"
"    color: black;\n"
"    font-size: 10px;\n"
"	font-family: \"Noto Sans KR Black\";\n"
"}")
        self.linear_speed_usr_textEdit.setReadOnly(True)
        self.accel_rate_label_6 = QLabel(self.scrollAreaWidgetContents)
        self.accel_rate_label_6.setObjectName(u"accel_rate_label_6")
        self.accel_rate_label_6.setGeometry(QRect(220, 400, 41, 32))
        self.accel_rate_label_6.setFont(font)
        self.accel_rate_label_6.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 22px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.accel_rate_label_6.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.linear_speed_usr_Slider = QSlider(self.scrollAreaWidgetContents)
        self.linear_speed_usr_Slider.setObjectName(u"linear_speed_usr_Slider")
        self.linear_speed_usr_Slider.setGeometry(QRect(280, 400, 501, 41))
        self.linear_speed_usr_Slider.setStyleSheet(u"QSlider::groove:horizontal {\n"
"    background: #eeeeee;\n"
"    height: 10px;              /* \ud2b8\ub799\uc740 \uc587\uac8c \uc720\uc9c0 */\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal {\n"
"    background: #3a86ff;\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::add-page:horizontal {\n"
"    background: #eeeeee;\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"    background: #3a86ff;\n"
"    border: none;\n"
"    width: 32px;               /* \uac00\ub85c \ud06c\uae30 */\n"
"    height: 32px;              /* \uc138\ub85c \ud06c\uae30 \uc99d\uac00 */\n"
"    margin: -11px 0px;         /* \uc704\uc544\ub798 margin \uc74c\uc218 \u2192 \ud2b8\ub799 \uc911\uc559 \uc815\ub82c */\n"
"    border-radius: 16px;       /* \uc644\uc804\ud55c \uc6d0\ud615 */\n"
"}\n"
"")
        self.linear_speed_usr_Slider.setOrientation(Qt.Horizontal)
        self.accel_rate_label_7 = QLabel(self.scrollAreaWidgetContents)
        self.accel_rate_label_7.setObjectName(u"accel_rate_label_7")
        self.accel_rate_label_7.setGeometry(QRect(790, 400, 51, 32))
        self.accel_rate_label_7.setFont(font)
        self.accel_rate_label_7.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 22px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.accel_rate_label_7.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.decel_rate_label_2 = QLabel(self.scrollAreaWidgetContents)
        self.decel_rate_label_2.setObjectName(u"decel_rate_label_2")
        self.decel_rate_label_2.setGeometry(QRect(20, 400, 151, 32))
        self.decel_rate_label_2.setFont(font)
        self.decel_rate_label_2.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 22px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.decel_rate_label_2.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.angular_speed_usr_textEdit = QTextEdit(self.scrollAreaWidgetContents)
        self.angular_speed_usr_textEdit.setObjectName(u"angular_speed_usr_textEdit")
        self.angular_speed_usr_textEdit.setGeometry(QRect(860, 460, 151, 51))
        self.angular_speed_usr_textEdit.setStyleSheet(u"QTextEdit {\n"
"    background-color: transparent;\n"
"	border: None;\n"
"    color: black;\n"
"    font-size: 10px;\n"
"	font-family: \"Noto Sans KR Black\";\n"
"}")
        self.angular_speed_usr_textEdit.setReadOnly(True)
        self.angular_speed_usr_Slider = QSlider(self.scrollAreaWidgetContents)
        self.angular_speed_usr_Slider.setObjectName(u"angular_speed_usr_Slider")
        self.angular_speed_usr_Slider.setGeometry(QRect(280, 470, 491, 41))
        self.angular_speed_usr_Slider.setStyleSheet(u"QSlider::groove:horizontal {\n"
"    background: #eeeeee;\n"
"    height: 10px;              /* \ud2b8\ub799\uc740 \uc587\uac8c \uc720\uc9c0 */\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal {\n"
"    background: #3a86ff;\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::add-page:horizontal {\n"
"    background: #eeeeee;\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"    background: #3a86ff;\n"
"    border: none;\n"
"    width: 32px;               /* \uac00\ub85c \ud06c\uae30 */\n"
"    height: 32px;              /* \uc138\ub85c \ud06c\uae30 \uc99d\uac00 */\n"
"    margin: -11px 0px;         /* \uc704\uc544\ub798 margin \uc74c\uc218 \u2192 \ud2b8\ub799 \uc911\uc559 \uc815\ub82c */\n"
"    border-radius: 16px;       /* \uc644\uc804\ud55c \uc6d0\ud615 */\n"
"}\n"
"")
        self.angular_speed_usr_Slider.setOrientation(Qt.Horizontal)
        self.accel_rate_label_8 = QLabel(self.scrollAreaWidgetContents)
        self.accel_rate_label_8.setObjectName(u"accel_rate_label_8")
        self.accel_rate_label_8.setGeometry(QRect(790, 470, 51, 32))
        self.accel_rate_label_8.setFont(font)
        self.accel_rate_label_8.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 22px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.accel_rate_label_8.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.decel_rate_label_3 = QLabel(self.scrollAreaWidgetContents)
        self.decel_rate_label_3.setObjectName(u"decel_rate_label_3")
        self.decel_rate_label_3.setGeometry(QRect(20, 470, 161, 32))
        self.decel_rate_label_3.setFont(font)
        self.decel_rate_label_3.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 22px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.decel_rate_label_3.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.accel_rate_label_9 = QLabel(self.scrollAreaWidgetContents)
        self.accel_rate_label_9.setObjectName(u"accel_rate_label_9")
        self.accel_rate_label_9.setGeometry(QRect(220, 470, 41, 32))
        self.accel_rate_label_9.setFont(font)
        self.accel_rate_label_9.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 22px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.accel_rate_label_9.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.accel_rate_label_2 = QLabel(self.scrollAreaWidgetContents)
        self.accel_rate_label_2.setObjectName(u"accel_rate_label_2")
        self.accel_rate_label_2.setGeometry(QRect(190, 260, 51, 32))
        self.accel_rate_label_2.setFont(font)
        self.accel_rate_label_2.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 22px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.accel_rate_label_2.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.accel_rate_Slider = QSlider(self.scrollAreaWidgetContents)
        self.accel_rate_Slider.setObjectName(u"accel_rate_Slider")
        self.accel_rate_Slider.setGeometry(QRect(250, 260, 401, 41))
        self.accel_rate_Slider.setStyleSheet(u"QSlider::groove:horizontal {\n"
"    background: #eeeeee;\n"
"    height: 10px;              /* \ud2b8\ub799\uc740 \uc587\uac8c \uc720\uc9c0 */\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal {\n"
"    background: #3a86ff;\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::add-page:horizontal {\n"
"    background: #eeeeee;\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"    background: #3a86ff;\n"
"    border: none;\n"
"    width: 32px;               /* \uac00\ub85c \ud06c\uae30 */\n"
"    height: 32px;              /* \uc138\ub85c \ud06c\uae30 \uc99d\uac00 */\n"
"    margin: -11px 0px;         /* \uc704\uc544\ub798 margin \uc74c\uc218 \u2192 \ud2b8\ub799 \uc911\uc559 \uc815\ub82c */\n"
"    border-radius: 16px;       /* \uc644\uc804\ud55c \uc6d0\ud615 */\n"
"}\n"
"")
        self.accel_rate_Slider.setOrientation(Qt.Horizontal)
        self.accel_rate_label = QLabel(self.scrollAreaWidgetContents)
        self.accel_rate_label.setObjectName(u"accel_rate_label")
        self.accel_rate_label.setGeometry(QRect(10, 260, 131, 32))
        self.accel_rate_label.setFont(font)
        self.accel_rate_label.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 22px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.accel_rate_label.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.accel_rate_textEdit = QTextEdit(self.scrollAreaWidgetContents)
        self.accel_rate_textEdit.setObjectName(u"accel_rate_textEdit")
        self.accel_rate_textEdit.setGeometry(QRect(860, 250, 151, 51))
        self.accel_rate_textEdit.setStyleSheet(u"QTextEdit {\n"
"    background-color: transparent;\n"
"	border: None;\n"
"    color: black;\n"
"    font-size: 10px;\n"
"	font-family: \"Noto Sans KR Black\";\n"
"}")
        self.accel_rate_textEdit.setReadOnly(True)
        self.accel_rate_label_3 = QLabel(self.scrollAreaWidgetContents)
        self.accel_rate_label_3.setObjectName(u"accel_rate_label_3")
        self.accel_rate_label_3.setGeometry(QRect(660, 260, 52, 32))
        self.accel_rate_label_3.setFont(font)
        self.accel_rate_label_3.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 22px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.accel_rate_label_3.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.decel_rate_textEdit = QTextEdit(self.scrollAreaWidgetContents)
        self.decel_rate_textEdit.setObjectName(u"decel_rate_textEdit")
        self.decel_rate_textEdit.setGeometry(QRect(860, 320, 151, 51))
        self.decel_rate_textEdit.setStyleSheet(u"QTextEdit {\n"
"    background-color: transparent;\n"
"	border: None;\n"
"    color: black;\n"
"    font-size: 10px;\n"
"	font-family: \"Noto Sans KR Black\";\n"
"}")
        self.decel_rate_textEdit.setReadOnly(True)
        self.decel_rate_label = QLabel(self.scrollAreaWidgetContents)
        self.decel_rate_label.setObjectName(u"decel_rate_label")
        self.decel_rate_label.setGeometry(QRect(10, 330, 111, 32))
        self.decel_rate_label.setFont(font)
        self.decel_rate_label.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 22px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.decel_rate_label.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.accel_rate_label_5 = QLabel(self.scrollAreaWidgetContents)
        self.accel_rate_label_5.setObjectName(u"accel_rate_label_5")
        self.accel_rate_label_5.setGeometry(QRect(220, 320, 16, 32))
        self.accel_rate_label_5.setFont(font)
        self.accel_rate_label_5.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 22px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.accel_rate_label_5.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.decel_rate_Slider = QSlider(self.scrollAreaWidgetContents)
        self.decel_rate_Slider.setObjectName(u"decel_rate_Slider")
        self.decel_rate_Slider.setGeometry(QRect(250, 320, 401, 41))
        self.decel_rate_Slider.setStyleSheet(u"QSlider::groove:horizontal {\n"
"    background: #eeeeee;\n"
"    height: 10px;              /* \ud2b8\ub799\uc740 \uc587\uac8c \uc720\uc9c0 */\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal {\n"
"    background: #3a86ff;\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::add-page:horizontal {\n"
"    background: #eeeeee;\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"    background: #3a86ff;\n"
"    border: none;\n"
"    width: 32px;               /* \uac00\ub85c \ud06c\uae30 */\n"
"    height: 32px;              /* \uc138\ub85c \ud06c\uae30 \uc99d\uac00 */\n"
"    margin: -11px 0px;         /* \uc704\uc544\ub798 margin \uc74c\uc218 \u2192 \ud2b8\ub799 \uc911\uc559 \uc815\ub82c */\n"
"    border-radius: 16px;       /* \uc644\uc804\ud55c \uc6d0\ud615 */\n"
"}\n"
"")
        self.decel_rate_Slider.setOrientation(Qt.Horizontal)
        self.accel_rate_label_4 = QLabel(self.scrollAreaWidgetContents)
        self.accel_rate_label_4.setObjectName(u"accel_rate_label_4")
        self.accel_rate_label_4.setGeometry(QRect(660, 320, 52, 32))
        self.accel_rate_label_4.setFont(font)
        self.accel_rate_label_4.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 22px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.accel_rate_label_4.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.option_label = QLabel(self.scrollAreaWidgetContents)
        self.option_label.setObjectName(u"option_label")
        self.option_label.setGeometry(QRect(10, 120, 91, 41))
        self.option_label.setFont(font)
        self.option_label.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 22px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.option_label.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.callbell_option_checkBox = QCheckBox(self.scrollAreaWidgetContents)
        self.callbell_option_checkBox.setObjectName(u"callbell_option_checkBox")
        self.callbell_option_checkBox.setGeometry(QRect(120, 120, 191, 51))
        self.callbell_option_checkBox.setFont(font)
        self.callbell_option_checkBox.setStyleSheet(u"QCheckBox {\n"
"    spacing: 12px;                   /* \ud14d\uc2a4\ud2b8\uc640 \uccb4\ud06c\ubc15\uc2a4 \uac04 \uac04\uaca9 */\n"
"	font-family: 'Noto Sans KR Black';\n"
"    font-size: 20px;                 /* \uae00\uc528 \ud06c\uae30 */\n"
"    font-weight: bold;              /* \uae00\uc528 \uad75\uae30 (\ud5e4\ub354 \ub290\ub08c) */\n"
"    color: #1A1A1A;                 /* \uc9c4\ud55c \ud14d\uc2a4\ud2b8 \uc0c9 */\n"
"    padding: 10px 4px;              /* \uc5ec\ubc31 \ucd94\uac00 (\uc2dc\uac01\uc801\uc73c\ub85c \ub354 \ud07c\uc9c1\ud558\uac8c) */\n"
"}\n"
"\n"
"QCheckBox::indicator {\n"
"    width: 32px;                    /* \uccb4\ud06c \uc544\uc774\ucf58 \uac00\ub85c \ud06c\uae30 */\n"
"    height: 32px;                   /* \uccb4\ud06c \uc544\uc774\ucf58 \uc138\ub85c \ud06c\uae30 */\n"
"}\n"
"\n"
"/* \uccb4\ud06c \ud574\uc81c \uc0c1\ud0dc */\n"
"QCheckBox::indicator:unchecked {\n"
"    image: url(:/file/CheckBox/uncheck.png);\n"
"}\n"
"\n"
"/* \uccb4\ud06c\ub41c \uc0c1\ud0dc */\n"
"QCheckBox::i"
                        "ndicator:checked {\n"
"    image: url(:/file/CheckBox/checked.png);\n"
"}\n"
"")
        self.callbell_option_checkBox.setChecked(True)
        self.save_button = QPushButton(self.scrollAreaWidgetContents)
        self.save_button.setObjectName(u"save_button")
        self.save_button.setGeometry(QRect(410, 600, 200, 50))
        self.save_button.setMinimumSize(QSize(200, 50))
        self.save_button.setMaximumSize(QSize(200, 50))
        font1 = QFont()
        font1.setBold(True)
        self.save_button.setFont(font1)
        self.save_button.setStyleSheet(u"QPushButton {\n"
"    background-color: #00A3FF;\n"
"    color: white;\n"
"    font-size: 20px;\n"
"    font-weight: 600;\n"
"    border-radius: 8px;\n"
"    height: 50px;\n"
"    border: none;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #33B5FF;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #0080DD;\n"
"}")
        self.callbell_option_checkBox_2 = QCheckBox(self.scrollAreaWidgetContents)
        self.callbell_option_checkBox_2.setObjectName(u"callbell_option_checkBox_2")
        self.callbell_option_checkBox_2.setGeometry(QRect(150, 520, 41, 51))
        self.callbell_option_checkBox_2.setFont(font)
        self.callbell_option_checkBox_2.setStyleSheet(u"QCheckBox {\n"
"    spacing: 12px;                   /* \ud14d\uc2a4\ud2b8\uc640 \uccb4\ud06c\ubc15\uc2a4 \uac04 \uac04\uaca9 */\n"
"	font-family: 'Noto Sans KR Black';\n"
"    font-size: 20px;                 /* \uae00\uc528 \ud06c\uae30 */\n"
"    font-weight: bold;              /* \uae00\uc528 \uad75\uae30 (\ud5e4\ub354 \ub290\ub08c) */\n"
"    color: #1A1A1A;                 /* \uc9c4\ud55c \ud14d\uc2a4\ud2b8 \uc0c9 */\n"
"    padding: 10px 4px;              /* \uc5ec\ubc31 \ucd94\uac00 (\uc2dc\uac01\uc801\uc73c\ub85c \ub354 \ud07c\uc9c1\ud558\uac8c) */\n"
"}\n"
"\n"
"QCheckBox::indicator {\n"
"    width: 32px;                    /* \uccb4\ud06c \uc544\uc774\ucf58 \uac00\ub85c \ud06c\uae30 */\n"
"    height: 32px;                   /* \uccb4\ud06c \uc544\uc774\ucf58 \uc138\ub85c \ud06c\uae30 */\n"
"}\n"
"\n"
"/* \uccb4\ud06c \ud574\uc81c \uc0c1\ud0dc */\n"
"QCheckBox::indicator:unchecked {\n"
"    image: url(:/file/CheckBox/uncheck.png);\n"
"}\n"
"\n"
"/* \uccb4\ud06c\ub41c \uc0c1\ud0dc */\n"
"QCheckBox::i"
                        "ndicator:checked {\n"
"    image: url(:/file/CheckBox/checked.png);\n"
"}\n"
"")
        self.callbell_option_checkBox_2.setChecked(True)
        self.option_label_2 = QLabel(self.scrollAreaWidgetContents)
        self.option_label_2.setObjectName(u"option_label_2")
        self.option_label_2.setGeometry(QRect(10, 520, 131, 41))
        self.option_label_2.setFont(font)
        self.option_label_2.setStyleSheet(u"QLabel {\n"
"    color: #000000;\n"
"    font-size: 22px;\n"
"    font-weight: 500;\n"
"    font-family: 'Noto Sans KR Black';\n"
"    background: transparent;\n"
"}")
        self.option_label_2.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)
        self.scrollArea.setWidget(self.scrollAreaWidgetContents)

        self.verticalLayout.addWidget(self.scrollArea)


        self.retranslateUi(MoveOptionView)

        QMetaObject.connectSlotsByName(MoveOptionView)
    # setupUi

    def retranslateUi(self, MoveOptionView):
        MoveOptionView.setWindowTitle(QCoreApplication.translate("MoveOptionView", u"Dentium", None))
        self.title_label.setText(QCoreApplication.translate("MoveOptionView", u"\uc774\ub3d9 \uc635\uc158 \uc124\uc815", None))
        self.subtitle_label.setText(QCoreApplication.translate("MoveOptionView", u"\uc774\ub3d9 \uc911 \ub2e4\ub978 \ud638\ucd9c\ubca8 \uc751\ub2f5 \uc124\uc815", None))
        self.robot_velo_set_label.setText(QCoreApplication.translate("MoveOptionView", u"\ub85c\ubd07 \uc18d\ub3c4 \uc124\uc815", None))
        self.linear_speed_usr_textEdit.setHtml(QCoreApplication.translate("MoveOptionView", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Noto Sans KR Black'; font-size:10px; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:22pt;\">0</span></p></body></html>", None))
        self.accel_rate_label_6.setText(QCoreApplication.translate("MoveOptionView", u"0.1", None))
        self.accel_rate_label_7.setText(QCoreApplication.translate("MoveOptionView", u"1.57", None))
        self.decel_rate_label_2.setText(QCoreApplication.translate("MoveOptionView", u"\uc9c1\uc9c4  \uc18d\ub3c4(m/s)", None))
        self.angular_speed_usr_textEdit.setHtml(QCoreApplication.translate("MoveOptionView", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Noto Sans KR Black'; font-size:10px; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:22pt;\">0</span></p></body></html>", None))
        self.accel_rate_label_8.setText(QCoreApplication.translate("MoveOptionView", u"7.85", None))
        self.decel_rate_label_3.setText(QCoreApplication.translate("MoveOptionView", u"\ud68c\uc804 \uc18d\ub3c4(rad/s)", None))
        self.accel_rate_label_9.setText(QCoreApplication.translate("MoveOptionView", u"0.5", None))
        self.accel_rate_label_2.setText(QCoreApplication.translate("MoveOptionView", u"0.11", None))
        self.accel_rate_label.setText(QCoreApplication.translate("MoveOptionView", u"\uac00\uc18d\ub3c4(ms\u00b2)", None))
        self.accel_rate_textEdit.setHtml(QCoreApplication.translate("MoveOptionView", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Noto Sans KR Black'; font-size:10px; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:22pt;\">0</span></p></body></html>", None))
        self.accel_rate_label_3.setText(QCoreApplication.translate("MoveOptionView", u"1.68", None))
        self.decel_rate_textEdit.setHtml(QCoreApplication.translate("MoveOptionView", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Noto Sans KR Black'; font-size:10px; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:22pt;\">0</span></p></body></html>", None))
        self.decel_rate_label.setText(QCoreApplication.translate("MoveOptionView", u"\uac10\uc18d(ms\u00b2)", None))
        self.accel_rate_label_5.setText(QCoreApplication.translate("MoveOptionView", u"0", None))
        self.accel_rate_label_4.setText(QCoreApplication.translate("MoveOptionView", u"1023", None))
        self.option_label.setText(QCoreApplication.translate("MoveOptionView", u"\uc124\uc815 \uc635\uc158", None))
        self.callbell_option_checkBox.setText(QCoreApplication.translate("MoveOptionView", u"\ub2e4\ub978 \ud638\ucd9c\ubca8 \ubb34\uc2dc", None))
        self.save_button.setText(QCoreApplication.translate("MoveOptionView", u"\uc800\uc7a5", None))
        self.callbell_option_checkBox_2.setText("")
        self.option_label_2.setText(QCoreApplication.translate("MoveOptionView", u"\ube0c\ub808\uc774\ud06c \ubaa8\ub4dc", None))
    # retranslateUi

