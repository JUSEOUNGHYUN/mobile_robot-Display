import os
import json
import math
import uuid
import random
import subprocess
from datetime import datetime
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                              QLabel, QLineEdit, QGraphicsView, QGraphicsScene, 
                              QGraphicsPixmapItem, QGraphicsRectItem, QGraphicsEllipseItem,
                              QGraphicsLineItem, QGraphicsTextItem, QGraphicsPathItem,
                              QGraphicsPolygonItem, QScrollArea, QFrame, QStackedWidget,
                              QButtonGroup, QRadioButton, QCheckBox, QTextEdit, QMenu)
from PySide6.QtCore import Qt, Signal, QPointF, QRectF, QLineF, QTimer, QTime, QSize, QEvent
from PySide6.QtGui import QPixmap, QPen, QBrush, QColor, QPainter, QPainterPath, QPolygonF, QTransform, QFont
from Utils.MapManager import MapManager
from Utils.ButtonStyle import RoundButtonStyle
from Utils.Toast import ToastManager
from Utils.MessageBox import MessageBox
from PySide6.QtUiTools import QUiLoader
from Utils.RosTopic import KeepoutZonePublisher, DeleteKeepoutZonePublisher, PreferredAreaPublisher  # RosTopic 클래스 추가
from Utils.KeyboardLineEidt import KeyboardLineEdit  # KeyboardLineEdit 클래스 가져오기


class LocationAddView(QWidget):
    # 저장 및 취소 시그널
    save_signal = Signal()  # 파라미터 없음
    cancel_signal = Signal()
    position_clicked = Signal(float, float)  # x, y 좌표를 전달하는 시그널
    
    # 맵 컨트롤 시그널 추가
    map_zoom_in_signal = Signal()
    map_zoom_out_signal = Signal()
    map_move_up_signal = Signal()
    map_move_down_signal = Signal()
    map_move_left_signal = Signal()
    map_move_right_signal = Signal()
    
    # 로봇 컨트롤 시그널 추가
    robot_move_up_signal = Signal()
    robot_move_down_signal = Signal()
    robot_move_left_signal = Signal()
    robot_move_right_signal = Signal()
    robot_rotate_right_signal = Signal()
    robot_rotate_left_signal = Signal()

    # 금지구역 이동 시그널 추가
    object_move_up_signal = Signal()
    object_move_down_signal = Signal()
    object_move_left_signal = Signal()
    object_move_right_signal = Signal()

    # 영역 설정 상태 변경 시그널 추가
    area_setting_changed = Signal(bool)  # 영역 설정 모드 상태 변경 시그널

    # 마커(로봇) 이동 스텝 크기 (미터 단위)
    ROBOT_MOVE_STEP = 0.1
    # 마커(로봇) 회전 스텝 크기 (라디안 단위)
    ROBOT_ROTATE_STEP = 0.15

    # 영역 설정 관련 변수
    is_area_drawing_mode = False
    is_restricted_area_drawing = False
    is_two_point_drawing = False
    is_two_point_line_drawing = False  # 선 그리기 모드 추가
    current_rect = None
    current_circle = None
    current_line = None  # 선 객체 추가
    start_point = None
    first_click_point = None
    rect_size = 1.0  # 기본 사각형 크기 (미터 단위)
    circle_size = 0.5  # 원의 크기 (미터 단위)
    show_area_index = False  # 영역 인덱스 표시 여부

    def __init__(self, parent=None):
        """초기화"""
        super().__init__(parent)
        
        try:
            # 금지 구역 발행자 초기화
            self.keepout_zone_publisher = KeepoutZonePublisher()
            self.delete_keepout_zone_publisher = DeleteKeepoutZonePublisher()
            
            # 우선 경로 영역 발행자 초기화
            self.preferred_area_publisher = PreferredAreaPublisher()
        except Exception as e:
            print(f"ROS 발행자 초기화 중 오류: {e}")
        
        self.style_button_size = 80  # 버튼 크기 (픽셀 단위)
        self.button_opacity = 180  # 아이콘 크기 (픽셀 단위)

        # 호출벨 관련 변수 추가
        self.last_detected_callbell_id = None
        self.is_callbell_active = False
        
        # 기본 변수 초기화
        self.ui = None
        self.map_scene = None
        self.map_view = None
        self.map_manager = None
        self.map_widget = None
        self.status_widget = None
        self.register_button = None
        self.cancel_button = None
        self.name_input = None
        self.destination_name_input = None
        self.map_data_debug_TextEdit = None
        
        # 호출벨 버튼 변수 추가
        self.not_called_button = None
        self.callbell_activated_button = None
        self.updating_call_button = None
        
        # 드래그 관련 변수 추가
        self.is_dragging = False
        self.drag_start_pos = None
        self.last_drag_pos = None
        
        # 로봇 위치 관련 변수
        self.clicked_x = None  # 사용자가 클릭한 위치 (명시적으로 클릭하기 전까지는 None)
        self.clicked_y = None
        self.robot_yaw = 0.0  # 기본 방향 0도
        
        # 사용자가 명시적으로 클릭했는지 여부
        self.user_has_clicked = False
        
        # 마커 기능 활성화 여부
        self.is_marker_enabled = True
        
        self.setup_ui()
        self.setup_map()
        self.setup_connections()
        
        # 현재 모드 (추가 또는 수정)
        self.is_edit_mode = False
        
        # 위치 데이터 로딩 상태 플래그
        self.loading_location = False

        # 로봇 방향
        self.robot_yaw = 0.0

        # 영역 설정 관련 변수 초기화
        self.is_area_drawing_mode = False
        self.is_restricted_area_drawing = False
        self.is_two_point_drawing = False
        self.is_two_point_line_drawing = False  # 선 그리기 모드 추가
        self.current_rect = None
        self.current_circle = None
        self.current_line = None  # 선 객체 추가
        self.start_point = None
        self.first_click_point = None
        
        # 영역 설정 버튼 연결
        self.center_based_draw_button = self.ui.findChild(QPushButton, "center_based_draw_button")
        self.two_point_draw_button = self.ui.findChild(QPushButton, "two_point_draw_button")
        self.two_point_line_draw_button = self.ui.findChild(QPushButton, "two_point_line_draw_button")  # 선 그리기 버튼 추가
        if self.center_based_draw_button:
            self.center_based_draw_button.clicked.connect(self.toggle_center_based_drawing)
        if self.two_point_draw_button:
            self.two_point_draw_button.clicked.connect(self.toggle_two_point_drawing)
        if self.two_point_line_draw_button:
            self.two_point_line_draw_button.clicked.connect(self.toggle_two_point_line_drawing)  # 선 그리기 모드 전환 연결
        
        # 영역 목록 레이아웃 초기화
        self.area_list_container = self.ui.findChild(QWidget, "horizontalLayoutWidget")
        self.area_list_layout = self.ui.findChild(QHBoxLayout, "area_list_layout")
        self.area_list_label = self.ui.findChild(QLabel, "area_list_label")
        # area_list_label이 레이아웃에 없으면 추가
        if self.area_list_label and self.area_list_layout.indexOf(self.area_list_label) == -1:
            self.area_list_layout.addWidget(self.area_list_label)
        
        # 영역 목록 저장
        self.area_buttons = []
        
        # 금지구역 색상 빨간색으로 고정
        self.area_colors = [QColor(255, 0, 0)]
        self.available_colors = self.area_colors.copy()

        # area_list_layout 생성 후 정렬 지정
        self.area_list_layout.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        # 영역 설정 위젯 추가
        self.restricted_area_widget = self.ui.findChild(QWidget, "restricted_area_widget")
        self.priority_path_area_widget = self.ui.findChild(QWidget, "priority_path_area_widget")
        
        # 초기 위젯 상태 설정
        self.setup_area_widgets()

        self.three_point_line_draw_button = self.ui.findChild(QPushButton, "three_point_line_draw_button")
        if self.three_point_line_draw_button:
            self.three_point_line_draw_button.clicked.connect(self.toggle_three_point_line_drawing)
        
        # 세 점 선 그리기 관련 변수
        self.is_three_point_line_drawing = False
        self.three_point_line_points = []
        self.three_point_line_circles = []
        self.three_point_line_lines = []

        # __init__ 또는 setup_ui 등 클래스 초기화 부분에 추가
        self.is_marker_enabled = True
        self.set_area_button.setCheckable(True)

        self.grow_vertical_button = self.ui.findChild(QPushButton, "grow_vertical")
        RoundButtonStyle.apply_icon_transfer_button_style(self.grow_vertical_button, width=80, height=80, icon_path=":/file/SetArea/grow_vert.png",icon_size=80,border_radius=15, bg_color=(255, 255, 255), bg_opacity=180)
        self.shrink_vertical_button = self.ui.findChild(QPushButton, "shrink_vertical")
        RoundButtonStyle.apply_icon_transfer_button_style(self.shrink_vertical_button, width=80, height=80, icon_path=":/file/SetArea/shrink_vert.png",icon_size=80,border_radius=15, bg_color=(255, 255, 255), bg_opacity=180)

        if self.grow_vertical_button:
            self.grow_vertical_button.clicked.connect(self.grow_selected_button_vertical)
        if self.shrink_vertical_button:
            self.shrink_vertical_button.clicked.connect(self.shrink_selected_button_vertical)

        self.grow_horizontal_button = self.ui.findChild(QPushButton, "grow_horizontal")
        RoundButtonStyle.apply_icon_transfer_button_style(self.grow_horizontal_button, width=80, height=80, icon_path=":/file/SetArea/grow_horiz.png",icon_size=80,border_radius=15, bg_color=(255, 255, 255), bg_opacity=180)
        self.shrink_horizontal_button = self.ui.findChild(QPushButton, "shrink_horizontal")
        RoundButtonStyle.apply_icon_transfer_button_style(self.shrink_horizontal_button, width=80, height=80, icon_path=":/file/SetArea/shrink_horiz.png",icon_size=80,border_radius=15, bg_color=(255, 255, 255), bg_opacity=180)
        if self.grow_horizontal_button:
            self.grow_horizontal_button.clicked.connect(self.grow_selected_button_horizontal)
        if self.shrink_horizontal_button:
            self.shrink_horizontal_button.clicked.connect(self.shrink_selected_button_horizontal)

        self.map_control_label = self.ui.findChild(QLabel, "map_control_label")

        self.control_type_select_checkbox = self.ui.findChild(QCheckBox, "control_type_select_checkbox")
        if self.control_type_select_checkbox:
            self.control_type_select_checkbox.stateChanged.connect(self.on_control_type_select_changed)
        # 최초 UI 상태 동기화는 setup_ui 끝난 뒤에만 호출 (setup_ui 마지막에 호출)

        self.restricted_area_button = self.findChild(QPushButton, "restricted_area_button")
        if not self.restricted_area_button:
            self.restricted_area_button = QPushButton("금지 구역")
            if hasattr(self, 'area_list_layout'):
                self.area_list_layout.addWidget(self.restricted_area_button)
        if hasattr(self, 'priority_path_area_button') and self.priority_path_area_button:
            pass
        else:
            self.priority_path_area_button = self.findChild(QPushButton, "priority_path_area_button")
            if not self.priority_path_area_button:
                self.priority_path_area_button = QPushButton("우선 경로")
                if hasattr(self, 'area_list_layout'):
                    self.area_list_layout.addWidget(self.priority_path_area_button)
        
        # 버튼을 체크 가능하게 설정
        self.restricted_area_button.setCheckable(True)
        self.priority_path_area_button.setCheckable(True)
        
        # 시그널 연결(중복 방지)
        try:
            self.restricted_area_button.clicked.disconnect()
        except Exception:
            pass
        try:
            self.priority_path_area_button.clicked.disconnect()
        except Exception:
            pass
        self.restricted_area_button.clicked.connect(self.on_restricted_area_button_clicked)
        self.priority_path_area_button.clicked.connect(self.on_priority_path_area_button_clicked)
        
        #self.load_forbidden_areas()

        self.is_area_setting_screen = False
        self.update_ui_visibility()  # 초기 UI 상태 업데이트

        # 삭제 버튼 생성
        self.area_delete_button = QPushButton()
        RoundButtonStyle.apply_icon_button_style(
            self.area_delete_button, 
            width=80, 
            height=80, 
            icon_path=":/file/SetArea/all_delete_area.png",
            icon_size=60,
            border_radius=15
        )
        self.area_delete_button.clicked.connect(self.on_area_delete_button_clicked)
        self.area_delete_button.setVisible(False)

        # 모두 삭제 버튼 생성
        self.area_delete_all_button = QPushButton()
        RoundButtonStyle.apply_icon_button_style(
            self.area_delete_all_button, 
            width=80, 
            height=80, 
            icon_path=":/file/SetArea/delete_area.png",
            icon_size=60,
            border_radius=15
        )
        self.area_delete_all_button.clicked.connect(self.on_area_delete_all_button_clicked)
        self.area_delete_all_button.setVisible(True)

        # 원본 금지 구역 데이터 저장 변수 추가 shju 20250617
        self.original_forbidden_areas = None;
        self.area_buttons_modified = False  # 금지구역 수정 여부 플래그
        
        self.is_forbidden_area = False
        self.touched_area_index = -1
        self.touched_button = None  # 현재 터치된 버튼 저장



    def setup_ui(self):
        """UI 설정"""
        # UI 로드
        loader = QUiLoader()
        ui_file_path = os.path.join(os.path.dirname(__file__), "..", "UI", "LocationAddView.ui")
        self.ui = loader.load(ui_file_path)
        
        # 메인 레이아웃에 UI 추가
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.ui)
        
        # 버튼 및 입력 필드 연결
        self.register_button = self.ui.findChild(QPushButton, "register_button")
        self.cancel_button = self.ui.findChild(QPushButton, "cancel_button")
        self.destination_name_input = self.ui.findChild(QLineEdit, "destination_name_input")
        
        # 호출벨 버튼 초기화
        self.not_called_button = self.ui.findChild(QPushButton, "not_called_button")
        self.callbell_activated_button = self.ui.findChild(QPushButton, "callbell_activated_button")
        self.updating_call_button = self.ui.findChild(QPushButton, "updating_call_button")
        self.not_called_button.setVisible(True)
        self.callbell_activated_button.setVisible(False)
        self.updating_call_button.setVisible(False)
        
        # 목적지 등록 스타일 적용
        RoundButtonStyle.apply_icon_button_style(
            self.register_button, 
            width=80, 
            height=80, 
            icon_path=":/file/RegistSelect/add_destination.png",
            icon_size=60,
            border_radius=15
        )

        # 목적지 취소 버튼에 스타일 적용
        RoundButtonStyle.apply_icon_button_style(
            self.cancel_button, 
            width=80, 
            height=80, 
            icon_path=":/file/RegistSelect/cancel_destination.png",
            icon_size=60,
            border_radius=15
        )

        # 맵 뷰 및 씬 설정
        self.map_view = self.ui.findChild(QGraphicsView, "map_view")
        self.map_scene = QGraphicsScene()
        self.map_view.setScene(self.map_scene)
        self.map_view.setRenderHint(QPainter.Antialiasing)
        self.map_view.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        
        # 맵 위젯 찾기
        self.map_widget = self.ui.findChild(QWidget, "map_widget")
        
        # status_widget 찾기
        self.status_widget = self.ui.findChild(QWidget, "status_widget")
        
        # UI 파일에 맞게 위젯 찾기
        self.map_control_widget = self.ui.findChild(QWidget, "map_control_widget")
        self.marker_control_widget = self.ui.findChild(QWidget, "marker_control_widget")
        self.move_area_widget = self.ui.findChild(QWidget, "move_area_widget")
        self.draw_area_widget = self.ui.findChild(QWidget, "draw_area_widget")
        self.navigation_control_widget = self.ui.findChild(QWidget, "navigation_control_widget")

        self.apply_area_btn = self.ui.findChild(QPushButton, "apply_area_btn")
        self.cancel_area_btn = self.ui.findChild(QPushButton, "cancel_area_btn")

        # 영역 설정 적용 스타일 적용
        RoundButtonStyle.apply_icon_button_style(
            self.apply_area_btn, 
            width=80, 
            height=80, 
            icon_path=":/file/SetArea/apply_area.png",
            icon_size=80,
            border_radius=15
        )

        # 영역 설정 취소 스타일 적용
        RoundButtonStyle.apply_icon_button_style(
            self.cancel_area_btn, 
            width=80, 
            height=80, 
            icon_path=":/file/SetArea/cancel_area.png",
            icon_size=80,
            border_radius=15
        )



        # area_list_layout(QHBoxLayout)와 horizontalLayoutWidget(QWidget) 찾기
        self.area_list_container = self.ui.findChild(QWidget, "horizontalLayoutWidget")
        self.area_list_layout = self.ui.findChild(QHBoxLayout, "area_list_layout")
        self.area_list_label = self.ui.findChild(QLabel, "area_list_label")
        
        # area_list_label이 레이아웃에 없으면 추가
        if self.area_list_label and self.area_list_layout and self.area_list_layout.indexOf(self.area_list_label) == -1:
            self.area_list_layout.addWidget(self.area_list_label)
        
        # set_area_button 찾기
        self.set_area_button = self.ui.findChild(QPushButton, "set_area_button")
        
        # 레이아웃 변경 버튼에 스타일 적용
        RoundButtonStyle.apply_icon_button_style(
            self.set_area_button, 
            width=80, 
            height=80, 
            icon_path=":/file/SetArea/area_setting.png",
            icon_size=60,
            border_radius=15
        )
        
        # 맵 컨트롤 버튼 연결
        self.map_zoom_in = self.ui.findChild(QPushButton, "object_zoom_in")
        RoundButtonStyle.apply_icon_transfer_button_style(self.map_zoom_in, width=self.style_button_size, height=self.style_button_size, icon_path=":/file/MapControl/zoom_in.png",icon_size=self.style_button_size,border_radius=15, bg_color=(255, 255, 255), bg_opacity=220)

        self.map_zoom_out = self.ui.findChild(QPushButton, "object_zoom_out")
        RoundButtonStyle.apply_icon_transfer_button_style(self.map_zoom_out, width=self.style_button_size, height=self.style_button_size, icon_path=":/file/MapControl/zoom_out.png",icon_size=self.style_button_size,border_radius=15, bg_color=(255, 255, 255), bg_opacity=self.button_opacity)
        
        self.map_up = self.ui.findChild(QPushButton, "map_up")
        RoundButtonStyle.apply_icon_transfer_button_style(self.map_up, width=self.style_button_size, height=self.style_button_size, icon_path=":/file/Registration/map_up_30.png",icon_size=self.style_button_size,border_radius=15, bg_color=(255, 255, 255), bg_opacity=self.button_opacity)
        
        self.map_down = self.ui.findChild(QPushButton, "map_down")
        RoundButtonStyle.apply_icon_transfer_button_style(self.map_down, width=self.style_button_size, height=self.style_button_size, icon_path=":/file/Registration/map_down_30.png",icon_size=self.style_button_size,border_radius=15, bg_color=(255, 255, 255), bg_opacity=self.button_opacity)

        self.map_left = self.ui.findChild(QPushButton, "map_left")
        RoundButtonStyle.apply_icon_transfer_button_style(self.map_left, width=self.style_button_size, height=self.style_button_size, icon_path=":/file/Registration/map_left_30.png",icon_size=self.style_button_size,border_radius=15, bg_color=(255, 255, 255), bg_opacity=self.button_opacity)

        self.map_right = self.ui.findChild(QPushButton, "map_right")
        RoundButtonStyle.apply_icon_transfer_button_style(self.map_right, width=self.style_button_size, height=self.style_button_size, icon_path=":/file/Registration/map_right_30.png",icon_size=self.style_button_size,border_radius=15, bg_color=(255, 255, 255), bg_opacity=self.button_opacity)

        self.map_move_refresh = self.ui.findChild(QPushButton, "map_move_refresh")
        RoundButtonStyle.apply_icon_transfer_button_style(self.map_move_refresh, width=self.style_button_size, height=self.style_button_size, icon_path=":/file/Registration/map_view_refresh.png",icon_size=self.style_button_size,border_radius=15, bg_color=(255, 255, 255), bg_opacity=self.button_opacity)
        
        # 로봇 컨트롤 버튼 연결
        self.robot_move_up = self.ui.findChild(QPushButton, "robot_move_up")
        RoundButtonStyle.apply_icon_transfer_button_style(self.robot_move_up, width=self.style_button_size, height=self.style_button_size, icon_path=":/file/MarkerMove/robot_move_up.png",icon_size=self.style_button_size,border_radius=15, bg_color=(255, 255, 255), bg_opacity=self.button_opacity)
        self.robot_move_down = self.ui.findChild(QPushButton, "robot_move_down")
        RoundButtonStyle.apply_icon_transfer_button_style(self.robot_move_down, width=self.style_button_size, height=self.style_button_size, icon_path=":/file/MarkerMove/robot_move_down.png",icon_size=self.style_button_size,border_radius=15, bg_color=(255, 255, 255), bg_opacity=self.button_opacity)
        self.robot_move_left = self.ui.findChild(QPushButton, "robot_move_left")
        RoundButtonStyle.apply_icon_transfer_button_style(self.robot_move_left, width=self.style_button_size, height=self.style_button_size, icon_path=":/file/MarkerMove/robot_move_left.png",icon_size=self.style_button_size,border_radius=15, bg_color=(255, 255, 255), bg_opacity=self.button_opacity)
        self.robot_move_right = self.ui.findChild(QPushButton, "robot_move_right")
        RoundButtonStyle.apply_icon_transfer_button_style(self.robot_move_right, width=self.style_button_size, height=self.style_button_size, icon_path=":/file/MarkerMove/robot_move_right.png",icon_size=self.style_button_size,border_radius=15, bg_color=(255, 255, 255), bg_opacity=self.button_opacity)
        self.robot_rotate_right = self.ui.findChild(QPushButton, "robot_rotate_right")
        RoundButtonStyle.apply_icon_transfer_button_style(self.robot_rotate_right, width=self.style_button_size, height=self.style_button_size, icon_path=":/file/MarkerMove/robot_rotate_right.png",icon_size=self.style_button_size,border_radius=15, bg_color=(255, 255, 255), bg_opacity=self.button_opacity)
        self.robot_rotate_left = self.ui.findChild(QPushButton, "robot_rotate_left")
        RoundButtonStyle.apply_icon_transfer_button_style(self.robot_rotate_left, width=self.style_button_size, height=self.style_button_size, icon_path=":/file/MarkerMove/robot_rotate_left.png",icon_size=self.style_button_size,border_radius=15, bg_color=(255, 255, 255), bg_opacity=self.button_opacity)
        
        # 디버그 텍스트 영역 연결
        self.map_data_debug_TextEdit = self.ui.findChild(QTextEdit, "map_data_debug_TextEdit")
        if self.map_data_debug_TextEdit:
            self.map_data_debug_TextEdit.setReadOnly(True)
        
        # 맵 뷰에 이벤트 필터 설치
        self.map_view.viewport().installEventFilter(self)
        
        # 영역 설정 관련 버튼 연결
        self.restricted_area_button = self.ui.findChild(QPushButton, "restricted_area_button")
        #RoundButtonStyle.apply_icon_transfer_button_style(self.restricted_area_button, width=70, height=70, icon_path=":/file/RegistSelect/restricted_50.png",icon_size=70,border_radius=15, bg_color=(255, 255, 255), bg_opacity=180)
        self.priority_path_area_button = self.ui.findChild(QPushButton, "priority_path_area_button")
        #RoundButtonStyle.apply_icon_transfer_button_style(self.priority_path_area_button, width=70, height=70, icon_path=":/file/RegistSelect/priority_path_50.png",icon_size=70,border_radius=15, bg_color=(255, 255, 255), bg_opacity=180)

        # 이동/회전/확대/축소 컨트롤 버튼 연결
        self.object_move_up = self.ui.findChild(QPushButton, "object_move_up")
        RoundButtonStyle.apply_icon_transfer_button_style(self.object_move_up, width=self.style_button_size, height=self.style_button_size, icon_path=":/file/Registration/map_up_30.png",icon_size=self.style_button_size,border_radius=15, bg_color=(255, 255, 255), bg_opacity=self.button_opacity)
        self.object_move_down = self.ui.findChild(QPushButton, "object_move_down")
        RoundButtonStyle.apply_icon_transfer_button_style(self.object_move_down, width=self.style_button_size, height=self.style_button_size, icon_path=":/file/Registration/map_down_30.png",icon_size=self.style_button_size,border_radius=15, bg_color=(255, 255, 255), bg_opacity=self.button_opacity)
        self.object_move_left = self.ui.findChild(QPushButton, "object_move_left")
        RoundButtonStyle.apply_icon_transfer_button_style(self.object_move_left, width=self.style_button_size, height=self.style_button_size, icon_path=":/file/Registration/map_left_30.png",icon_size=self.style_button_size,border_radius=15, bg_color=(255, 255, 255), bg_opacity=self.button_opacity)
        self.object_move_right = self.ui.findChild(QPushButton, "object_move_right")
        RoundButtonStyle.apply_icon_transfer_button_style(self.object_move_right, width=self.style_button_size, height=self.style_button_size, icon_path=":/file/Registration/map_right_30.png",icon_size=self.style_button_size,border_radius=15, bg_color=(255, 255, 255), bg_opacity=self.button_opacity)
        self.object_rotate = self.ui.findChild(QPushButton, "object_rotate")
        #RoundButtonStyle.apply_icon_transfer_button_style(self.object_rotate, width=60, height=60, icon_path=":/file/Registration/map_up_30.png",icon_size=60,border_radius=15, bg_color=(255, 255, 255), bg_opacity=180)
        
        # map_control_button 찾기 및 연결
        self.map_control_button = self.ui.findChild(QPushButton, "map_control_button")
        if self.map_control_button:
            self.map_control_button.clicked.connect(self.on_map_control_button_clicked)
            # 초기 상태 설정 (기본적으로 map_control_widget 표시)
            self.map_control_button.setChecked(True)

        self.control_type_select_checkbox = self.ui.findChild(QCheckBox, "control_type_select_checkbox")
        self.object_zoom_in = self.ui.findChild(QPushButton, "object_zoom_in")
        self.object_zoom_out = self.ui.findChild(QPushButton, "object_zoom_out")
        self.grow_vertical_button = self.ui.findChild(QPushButton, "grow_vertical")
        self.shrink_vertical_button = self.ui.findChild(QPushButton, "shrink_vertical")
        self.grow_horizontal_button = self.ui.findChild(QPushButton, "grow_horizontal")
        self.shrink_horizontal_button = self.ui.findChild(QPushButton, "shrink_horizontal")

        if self.object_zoom_in:
            self.object_zoom_in.pressed.connect(lambda: self.start_object_move_timer('zoom_in'))
            self.object_zoom_in.released.connect(self.stop_object_move_timer)
            self.object_zoom_in.clicked.connect(lambda: self.zoom_map(1.1))  # 맵 확대
        if self.object_zoom_out:
            self.object_zoom_out.pressed.connect(lambda: self.start_object_move_timer('zoom_out'))
            self.object_zoom_out.released.connect(self.stop_object_move_timer)
            self.object_zoom_out.clicked.connect(lambda: self.zoom_map(0.9))  # 맵 축소

        # 롱프레스용 타이머 및 상태
        self.object_move_timer = QTimer(self)
        self.object_move_timer.setInterval(80)  # 80ms마다 반복
        self.object_move_timer.timeout.connect(self._on_object_move_timer)
        self.object_move_action = None
        
        # 네비게이션 컨트롤 위젯 설정
        self.setup_navigation_control_widget()
        
        # 금지구역 이동 버튼 연결
        if self.object_move_up:
            self.object_move_up.pressed.connect(lambda: self.start_object_move_timer('up'))
            self.object_move_up.released.connect(self.stop_object_move_timer)
            self.object_move_up.clicked.connect(self.move_selected_button_up)
        if self.object_move_down:
            self.object_move_down.pressed.connect(lambda: self.start_object_move_timer('down'))
            self.object_move_down.released.connect(self.stop_object_move_timer)
            self.object_move_down.clicked.connect(self.move_selected_button_down)
        if self.object_move_left:
            self.object_move_left.pressed.connect(lambda: self.start_object_move_timer('left'))
            self.object_move_left.released.connect(self.stop_object_move_timer)
            self.object_move_left.clicked.connect(self.move_selected_button_left)
        if self.object_move_right:
            self.object_move_right.pressed.connect(lambda: self.start_object_move_timer('right'))
            self.object_move_right.released.connect(self.stop_object_move_timer)
            self.object_move_right.clicked.connect(self.move_selected_button_right)

        # 세 점 선 그리기 관련 변수
        self.is_three_point_line_drawing = False
        self.three_point_line_points = []
        self.three_point_line_circles = []
        self.three_point_line_lines = []

        # 초기 위젯 표시 상태 설정
        self.status_widget.setVisible(True)
        self.area_list_container.setVisible(False)
        
        # 버튼 연결
        self.set_area_button.clicked.connect(self.toggle_area_control)
        self.apply_area_btn.clicked.connect(self.on_apply_area)
        self.cancel_area_btn.clicked.connect(self.on_cancel_area)
        
        # 영역 설정 버튼 연결
        if self.restricted_area_button:
            self.restricted_area_button.clicked.connect(self.on_restricted_area_button_clicked)
        if self.priority_path_area_button:
            self.priority_path_area_button.clicked.connect(self.on_priority_path_area_button_clicked)

        # 초기 UI 요소 표시 상태 설정
        self.is_area_setting_screen = False
        self.update_ui_visibility()

        # setup_ui 마지막에만 상태 동기화
        if self.control_type_select_checkbox:
            self.on_control_type_select_changed(self.control_type_select_checkbox.checkState())

        # 컨트롤 아이콘 버튼 잠깐 hide 20250618
        # self.map_control_icon_button = self.ui.findChild(QPushButton, "map_control_icon_button")
        # self.marker_control_icon_button = self.ui.findChild(QPushButton, "marker_control_icon_button")
        # self.map_control_icon_button.hide()
        # self.marker_control_icon_button.hide()

        # 디버그 텍스트를 뷰 좌표계에 고정
        self.setup_overlay_debug_text()

    def setup_map(self):
        """맵 초기화"""
        # 씬 초기화
        self.map_scene = QGraphicsScene()
        self.map_view.setScene(self.map_scene)

        self.map_scene.setSceneRect(0,0, 552, 491)  # 씬 크기 설정 (픽셀 단위)
        
        # 뷰 설정
        self.map_view.setRenderHint(QPainter.Antialiasing)
        self.map_view.setRenderHint(QPainter.SmoothPixmapTransform)
        self.map_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.map_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.map_view.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.map_view.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.map_view.setTransformationAnchor(QGraphicsView.AnchorViewCenter)

        self.map_view.viewport().setAttribute(Qt.WA_AcceptTouchEvents, True)
        
        # 맵 매니저 초기화 (이미지 로드 활성화, 변환 정보 적용 비활성화)
        self.map_manager = MapManager(self.map_view, self.map_scene, load_image=True, apply_transform=False)
        # 오버레이 이미지 경로 설정
        self.map_manager.map_overlay_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                                        "resources", "map_original_image.png")
        
        # 맵 변환 정보 확인
        self.check_map_transform()

        self.map_manager.position_clicked.connect(self.on_position_clicked)

        # 마우스 이벤트 필터 설정
        self.map_view.viewport().installEventFilter(self)

        # forbidden_areas.json에서 금지구역 불러오기
        self.load_forbidden_areas()

    def setup_overlay_debug_text(self):
        """맵 뷰 위에 오버레이로 디버그 텍스트 추가"""
        # 기존 디버그 텍스트 위젯 참조
        #self.map_data_debug_TextEdit = self.ui.findChild(QTextEdit, "map_data_debug_TextEdit")
        
        if not self.map_view:
            print("맵 뷰를 찾을 수 없습니다.")
            return
            
        self.overlay_debug_text = QTextEdit(self.map_view)
        self.overlay_debug_text.setFixedSize(200, 90)
        self.overlay_debug_text.setReadOnly(True)
        self.overlay_debug_text.setStyleSheet("""
            QTextEdit {
                background-color: transparent;  /* 투명 대신 흰색 */
                color: red;               /* 글자색 빨간색 */
                border: none;  /* 테두리 제거 */
                font-size: 18px;          /* 글자 크기 키움 */
                font-weight: bold;        /* 굵은 글씨 */
                font-family: 'Noto Sans KR Black'; /* 폰트 설정 */
                padding: 4px;
            }
        """)
        
        # 왼쪽 상단에 위치 설정
        self.overlay_debug_text.move(10, 10)
        self.overlay_debug_text.setVisible(True)
        self.overlay_debug_text.raise_()  # 항상 맨 위에 표시
        
        # 기존 디버그 텍스트 초기화 (값은 복사)
        if hasattr(self, 'map_data_debug_TextEdit') and self.map_data_debug_TextEdit and self.map_data_debug_TextEdit.toPlainText():
            self.overlay_debug_text.setText(self.map_data_debug_TextEdit.toPlainText())
            
        # 기존 디버그 텍스트 위젯 숨기기 (선택적)
        if hasattr(self, 'map_data_debug_TextEdit') and self.map_data_debug_TextEdit:
            self.map_data_debug_TextEdit.setVisible(False)

    def update_debug_text(self, x, y, yaw):
        """맵 위치 정보를 디버그 텍스트에 업데이트"""
        print("\n===== update_debug_text 함수 시작 =====")
        try:
            # 숫자 형식화: 항상 소수점 이하 3자리까지 표시
            x_val = 0.0 if x is None else float(x)
            y_val = 0.0 if y is None else float(y)
            yaw_val = 0.0 if yaw is None else float(yaw)
            
            debug_text = f"X: {x_val:.3f}\nY: {y_val:.3f}\nYaw: {yaw_val:.3f}"
            print("디버그 텍스트:\n", debug_text)
            
            # 기존 디버그 텍스트 업데이트 (숨겨져 있더라도 값은 유지)
            if hasattr(self, 'map_data_debug_TextEdit') and self.map_data_debug_TextEdit:
                self.map_data_debug_TextEdit.setText(debug_text)
                print(f"디버그 텍스트 업데이트:\n {debug_text}")
            
            # 오버레이 디버그 텍스트 업데이트
            if hasattr(self, 'overlay_debug_text') and self.overlay_debug_text:
                current_text = self.overlay_debug_text.toPlainText()
                if current_text != debug_text:  # 텍스트가 변경된 경우에만 업데이트
                    self.overlay_debug_text.setText(debug_text)
                    self.overlay_debug_text.move(10, 10)
                    self.overlay_debug_text.raise_()
                    print(f"디버그 텍스트 업데이트: {debug_text}")
        except Exception as e:
            print(f"디버그 텍스트 업데이트 중 오류 발생: {e}")

    def update_ui_visibility(self):
        """영역 설정 모드에 따라 UI 요소 표시 여부 업데이트"""
        # map_control_button 상태에 따라 위젯 표시 여부 업데이트
        self.update_control_widgets_visibility()
        
        # is_area_setting_screen 값에 따라 UI 요소 표시/숨김 처리
        if self.is_area_setting_screen:
            # 영역 설정 모드일 때 표시할 위젯
            if hasattr(self, 'draw_area_widget') and self.draw_area_widget:
                self.draw_area_widget.show()
            if hasattr(self, 'apply_area_btn') and self.apply_area_btn:
                self.apply_area_btn.show()
            if hasattr(self, 'cancel_area_btn') and self.cancel_area_btn:
                self.cancel_area_btn.show()
            
            # 영역 목록 컨테이너 표시
            if hasattr(self, 'area_list_container') and self.area_list_container:
                self.area_list_container.show()
            
            # 마커 컨트롤은 숨김
            if hasattr(self, 'marker_control_widget') and self.marker_control_widget:
                self.marker_control_widget.hide()
            if hasattr(self, 'status_widget') and self.status_widget:
                self.status_widget.hide()
            if hasattr(self, 'navigation_control_widget') and self.navigation_control_widget:
                self.navigation_control_widget.hide()
            
            # 등록/취소 버튼 숨김
            if hasattr(self, 'register_button') and self.register_button:
                self.register_button.hide()
            if hasattr(self, 'cancel_button') and self.cancel_button:
                self.cancel_button.hide()

            # 레이아웃 초기화 - 모든 버튼 제거
            if hasattr(self, 'area_list_layout'):
                # 레이아웃에서 모든 항목 제거
                while self.area_list_layout.count() > 0:
                    item = self.area_list_layout.takeAt(0)
                    if item.widget():
                        item.widget().setParent(None)
                
                # 왼쪽 정렬: 삭제, 모두 삭제 버튼
                if hasattr(self, 'area_delete_button') and self.area_delete_button:
                    self.area_list_layout.addWidget(self.area_delete_button)
                    self.area_delete_button.setVisible(True)
                    
                if hasattr(self, 'area_delete_all_button') and self.area_delete_all_button:
                    self.area_list_layout.addWidget(self.area_delete_all_button)
                    self.area_delete_all_button.setVisible(True)
        else:
            # 일반 모드일 때는 마커 컨트롤만 표시
            if hasattr(self, 'marker_control_widget') and self.marker_control_widget:
                self.marker_control_widget.show()
            if hasattr(self, 'status_widget') and self.status_widget:
                self.status_widget.show()
            if hasattr(self, 'navigation_control_widget') and self.navigation_control_widget:
                self.navigation_control_widget.show()
            
            # 등록/취소 버튼 표시
            if hasattr(self, 'register_button') and self.register_button:
                self.register_button.show()
            if hasattr(self, 'cancel_button') and self.cancel_button:
                self.cancel_button.show()
            
            # 영역 설정 관련 위젯은 숨김
            if hasattr(self, 'draw_area_widget') and self.draw_area_widget:
                self.draw_area_widget.hide()
            if hasattr(self, 'apply_area_btn') and self.apply_area_btn:
                self.apply_area_btn.hide()
            if hasattr(self, 'cancel_area_btn') and self.cancel_area_btn:
                self.cancel_area_btn.hide()

            # 영역 목록 컨테이너 숨김
            if hasattr(self, 'area_list_container') and self.area_list_container:
                self.area_list_container.hide()

    def load_forbidden_areas(self):
        """forbidden_areas.json에서 금지구역을 불러와서 그래픽뷰에 그림"""
        print("\n===== load_forbidden_areas 함수 시작 =====")
        
        # map_manager가 None인지 확인
        if not hasattr(self, 'map_manager') or self.map_manager is None:
            print("map_manager가 초기화되지 않았습니다.")
            print("===== load_forbidden_areas 함수 종료 =====\n")
            return
            
        # map_scene이 None인지 확인
        if not hasattr(self.map_manager, 'map_scene') or self.map_manager.map_scene is None:
            print("map_scene이 초기화되지 않았습니다.")
            print("===== load_forbidden_areas 함수 종료 =====\n")
            return
            
        # 현재 씬 크기 저장 및 고정
        self.map_manager.map_scene.setSceneRect(0, 0, 552, 491)
        print(f"[MapManager] 씬 크기 고정: 552 x 491 픽셀")
        
        # 맵 정보와 변환 정보 초기화 확인
        if not self.map_manager.map_info:
            self.map_manager.setup_default_map_info()
            print("[load_forbidden_areas] 맵 정보 초기화됨")

        # 맵 변환 정보 로드 확인
        self.map_manager.load_overlay_transform()
        print(f"[load_forbidden_areas] 맵 변환 정보: {self.map_manager.overlay_transform}")
        
        # area_buttons가 없으면 초기화
        if not hasattr(self, 'area_buttons'):
            self.area_buttons = []
            print("[디버깅] area_buttons 리스트 초기화")
        else:
            print(f"[디버깅] 기존 area_buttons 개수: {len(self.area_buttons)}")
            
        data_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')
        file_path = os.path.join(data_dir, 'forbidden_areas.json')
        print(f"[디버깅] JSON 파일 경로: {file_path}")
        
        if not os.path.exists(file_path):
            print(f"JSON 파일이 존재하지 않음: {file_path}")
            print("===== load_forbidden_areas 함수 종료 =====\n")
            return
                
        try:
            print("[디버깅] JSON 파일 읽기 시작")
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                print(f"[디버깅] JSON 데이터 로드 완료: {type(data)}")
                
                # 데이터 구조 확인 및 추출
                area_list = []
                if isinstance(data, list):
                    area_list = data
                    print("[디버깅] 데이터 구조: 리스트")
                elif isinstance(data, dict) and 'areas' in data:
                    print(f"[디버깅] 데이터 구조: 딕셔너리 with 'areas' 키, 타입: {type(data['areas'])}")
                    if isinstance(data['areas'], list):
                        # 단순 리스트 형태
                        area_list = data['areas']
                        print("[디버깅] 'areas' 키의 값이 리스트")
                    elif isinstance(data['areas'], list) and len(data['areas']) > 0 and isinstance(data['areas'][0], list):
                        # 중첩 리스트 형태 (이전 구조)
                        area_list = data['areas'][0] if data['areas'] else []
                        print("[디버깅] 'areas' 키의 값이 중첩 리스트")
                
                print(f"JSON에서 읽은 영역 개수: {len(area_list)}")
                
                for area_idx, area in enumerate(area_list):
                    print(f"\n[디버깅] 영역 {area_idx+1} 처리 시작")
                    # area가 dict인지 확인
                    if not isinstance(area, dict):
                        print(f"잘못된 데이터 형식: {area}")
                        continue
                    
                    area_index = area.get('index', 0)
                    area_type = area.get('type', '')
                    area_category = area.get('area_type', '금지 구역')  # 영역 카테고리(금지 구역 또는 우선 경로) 가져오기
                    points = area.get('points', [])
                    
                    print(f"[디버깅] 영역 {area_idx+1} 정보: index={area_index}, type={area_type}, area_type={area_category}, points 개수={len(points)}")
                    
                    # 각 폴리곤 꼭짓점의 맵 좌표를 씬 좌표로 변환하여 출력
                    print(f"[디버깅] 폴리곤 꼭짓점 좌표 변환:")
                    scene_points = []
                    
                    # /keepout_zone 토픽으로 금지 구역 발행 (금지 구역인 경우만)
                    if area_category == "금지 구역" and hasattr(self, 'keepout_zone_publisher'):
                        try:
                            # 금지 구역 발행
                            self.keepout_zone_publisher.publish_keepout_zone(area_index, points)
                            print(f"[keepout_zone] 금지 구역 {area_index} 발행 완료: {len(points)}개 꼭짓점")
                        except Exception as e:
                            print(f"[keepout_zone] 금지 구역 발행 중 오류 발생: {str(e)}")
                    
                    # /preferred_area 토픽으로 우선 경로 영역 발행 (우선 경로인 경우만)
                    elif area_category == "우선 경로" or area_category == "\uc6b0\uc120 \uacbd\ub85c" and hasattr(self, 'preferred_area_publisher'):
                        try:
                            # 우선 경로 영역 발행
                            self.preferred_area_publisher.publish_preferred_area(area_index, points)
                            print(f"[preferred_area] 우선 경로 영역 {area_index} 발행 완료: {len(points)}개 꼭짓점")
                        except Exception as e:
                            print(f"[preferred_area] 우선 경로 영역 발행 중 오류 발생: {str(e)}")
                    
                    for i, point in enumerate(points):
                        map_x, map_y = point[0], point[1]
                        
                        # convert_map_to_scene_coords 함수 사용 (LocationAddView의 메서드)
                        scene_pos = self.convert_map_to_scene_coords(map_x, map_y)
                        scene_x, scene_y = scene_pos.x(), scene_pos.y()
                        scene_points.append((scene_x, scene_y))
                        
                        print(f"  꼭짓점 {i+1}: 맵 좌표({map_x:.3f}, {map_y:.3f}) -> 씬 좌표({scene_x:.1f}, {scene_y:.1f})")
                    
                    if len(points) == 4 and hasattr(self, 'map_manager') and self.map_manager.map_info:
                        # 원본 points 데이터를 사용하여 중심점 계산
                        center_x = sum(pt[0] for pt in points) / 4
                        center_y = sum(pt[1] for pt in points) / 4
                        print(f"[디버깅] 금지구역 중심 좌표(맵 좌표): ({center_x:.3f}, {center_y:.3f})")
                        
                        # 맵 좌표를 씬 좌표로 변환 (버튼 위치 지정용)
                        # LocationAddView의 convert_map_to_scene_coords 함수 사용
                        scene_center = self.convert_map_to_scene_coords(center_x, center_y)
                        scene_center_x = scene_center.x()
                        scene_center_y = scene_center.y()
                        print(f"[디버깅] 금지구역 중심 씬 좌표: ({scene_center_x:.1f}, {scene_center_y:.1f})")
                        
                        # create_forbidden_area_button 함수 참고하여 버튼 직접 생성
                        # 이미 비슷한 위치에 버튼이 있는지 확인
                        skip_creation = False
                        for btn in self.area_buttons:
                            if hasattr(btn, 'is_area_button') and btn.is_area_button:
                                bx, by = btn.pos().x(), btn.pos().y()
                                bw, bh = btn.width(), btn.height()
                                btn_cx = bx + bw / 2
                                btn_cy = by + bh / 2
                                if abs(btn_cx - scene_center_x) < 40 and abs(btn_cy - scene_center_y) < 40:
                                    print(f"기존 버튼 발견: 위치=({btn_cx}, {btn_cy}), 거리={abs(btn_cx - scene_center_x)}, {abs(btn_cy - scene_center_y)}")
                                    skip_creation = True
                                    break
                        
                        if skip_creation:
                            print("기존 버튼이 있어서 새로 생성하지 않음")
                            continue
                        
                        print("새 버튼 생성")
                        button = QPushButton(None)  # 부모를 None으로 설정
                        button.setCheckable(True)
                        
                        # 영역 타입에 따른 버튼 스타일 설정
                        if area_category == "우선 경로":
                            # 우선 경로(파란색) 버튼 스타일
                            button.setStyleSheet(
                                """
                                QPushButton {
                                    background-color: transparent;
                                    border: 2px solid #0000FF;
                                    color: #0000FF;
                                    font-size: 20px;
                                    font-weight: bold;
                                }
                                QPushButton:checked {
                                    background-color: rgba(0, 0, 255, 0.3);
                                    color: #0000FF;
                                }
                                """
                            )
                            button.setProperty("area_type", "우선 경로")
                        else:
                            # 금지 구역(빨간색) 버튼 스타일 (기본값)
                            button.setStyleSheet(
                                """
                                QPushButton {
                                    background-color: transparent;
                                    border: 2px solid #FF0000;
                                    color: #FF0000;
                                    font-size: 20px;
                                    font-weight: bold;
                                }
                                QPushButton:checked {
                                    background-color: rgba(255, 0, 0, 0.3);
                                    color: #FF0000;
                                }
                                """
                            )
                            button.setProperty("area_type", "금지 구역")
                        
                        # 영역 타입에 따라 내부 인덱스 설정 (UI에는 표시하지 않음)
                        button.setText("")  # 텍스트 비워두기
                        button.is_area_button = True
                        button.setProperty("area_index", area_index)
                        button.setProperty("internal_index", area_index)  # internal_index 속성 추가
                        button.clicked.connect(lambda checked=False, b=button: self.on_area_button_clicked(b))
                        
                        # 버튼 위치 및 크기 설정 (수정 후)
                        if len(scene_points) == 4:
                            # 좌표를 기반으로 버튼 크기와 위치 계산
                            min_x = min(pt[0] for pt in scene_points)
                            max_x = max(pt[0] for pt in scene_points)
                            min_y = min(pt[1] for pt in scene_points)
                            max_y = max(pt[1] for pt in scene_points)
                            
                            button_width = max_x - min_x
                            button_height = max_y - min_y
                            
                            print(f"[디버깅] 계산된 버튼 크기: {button_width:.1f} x {button_height:.1f}")
                            
                            # 버튼 위치 및 크기 설정 (정수로 변환)
                            button_x = int(min_x)
                            button_y = int(min_y)
                            button_width = int(button_width)
                            button_height = int(button_height)
                            
                            print(f"[디버깅] 버튼 위치 설정: x={button_x}, y={button_y}, width={button_width}, height={button_height}")
                            button.setGeometry(button_x, button_y, button_width, button_height)
                        else:
                            # 좌표가 4개가 아닌 경우 기본 크기 사용
                            button_size = 60
                            button_x = int(scene_center_x - button_size/2)
                            button_y = int(scene_center_y - button_size/2)
                            print(f"[디버깅] 기본 버튼 위치 설정: x={button_x}, y={button_y}, size={button_size}")
                            button.setGeometry(button_x, button_y, button_size, button_size)
                        
                        # 버튼 위치 정보 저장 (나중에 참조할 수 있도록)
                        # TODO 2025_06_17
                        button.scene_center_x = scene_center_x
                        button.scene_center_y = scene_center_y
                        
                        # 맵 뷰 변환 행렬 초기화 및 고정 크기 설정
                        self.map_view.resetTransform()
                        self.map_manager.map_scene.setSceneRect(0, 0, 552, 491)
                        
                        # 버튼을 맵 뷰에 추가
                        proxy_widget = self.map_view.scene().addWidget(button)
                        print(f"[디버깅] QPushButton 생성 및 맵 뷰에 추가 완료: 위치=({button_x}, {button_y})")
                        
                        # 버튼에 원본 points 데이터 저장
                        button.map_points = points
                        print(f"불러온 버튼 {area_index}에 map_points 속성 설정: {points}")
                        
                        # 버튼 리스트에 추가
                        self.area_buttons.append(button)
                        print(f"[디버깅] 버튼이 area_buttons 리스트에 추가됨")
                        
                        # 버튼 인덱스 업데이트
                        self._update_area_delete_button()
                        
                        # 영역 타입별 인덱스 업데이트
                        self.update_area_button_indices()
                        
                        # 맵 뷰 업데이트 - 버튼 추가 후에도 원래 크기 유지
                        self.map_view.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio)

            # 씬 크기를 고정된 값으로 설정
            self.map_manager.map_scene.setSceneRect(0, 0, 552, 491)
            print(f"[MapManager] 씬 크기 고정: 552 x 491 픽셀")
            
            # 맵 뷰 업데이트
            self.map_view.resetTransform()
            self.map_view.fitInView(self.map_manager.map_scene.sceneRect(), Qt.KeepAspectRatio)
            print(f"[MapManager] _update_map_ui 완료 - 씬 크기: {self.map_manager.map_scene.sceneRect().size()}")
            
            # 금지구역 정보를 MapManager에 전달
            if hasattr(self, 'map_manager') and hasattr(self.map_manager, 'update_forbidden_areas'):
                self.map_manager.update_forbidden_areas(self.area_buttons)
                print(f"[load_forbidden_areas] MapManager에 금지구역 정보 전달 완료: {len(self.area_buttons)}개")

        except Exception as e:
            print(f"forbidden_areas.json 불러오기 오류: {e}")
            print(f"[디버깅] 오류 발생 위치: {e.__traceback__.tb_frame.f_code.co_filename}, 라인: {e.__traceback__.tb_lineno}")
            import traceback
            traceback.print_exc()
            
        print("===== load_forbidden_areas 함수 종료 =====\n")

    def check_map_transform(self):
        """맵 변환 정보 확인"""
        try:
            # map_manager가 초기화되었는지 확인
            if not hasattr(self, 'map_manager') or self.map_manager is None:
                print("map_manager가 초기화되지 않아 맵 변환 정보를 확인할 수 없습니다.")
                return False
                
            # 맵 변환 정보 파일 경로
            transform_file = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                        "data", "map_transform.json")
            
            if not os.path.exists(transform_file):
                # 변환 정보 파일이 없는 경우
                result = MessageBox.show_message(
                    self,
                    "맵 설정 필요",
                    "맵 변환 정보가 없습니다. 맵 설정에서 매핑을 먼저 진행해주세요.",
                    button_labels=["맵 설정으로 이동", "취소"]
                )
                
                if result == MessageBox.OK:
                    # 맵 이미지 컨트롤 화면으로 이동
                    if hasattr(self, 'map_image_control_signal'):
                        self.map_image_control_signal.emit()
                    return False
                else:
                    # 취소한 경우 이전 화면으로 돌아가기
                    self.cancel_signal.emit()
                    return False
            else:
                # 변환 정보 파일이 있는 경우 로드
                import json
                with open(transform_file, 'r', encoding='utf-8') as f:
                    transform_data = json.load(f)
                    
                # 필수 키가 모두 있는지 확인
                required_keys = ['rotation', 'scale', 'translate_x', 'translate_y']
                if not all(key in transform_data for key in required_keys):
                    print("맵 변환 정보가 올바르지 않습니다.")
                    return False
                    
                print(f"맵 변환 정보 로드됨: {transform_data}")
                return True
                
        except Exception as e:
            print(f"맵 변환 정보 확인 중 오류 발생: {str(e)}")
            return False

    def on_map_updated(self, msg):
        """맵 업데이트 시 호출되는 콜백"""
        try:
            # 맵 매니저의 _update_map_ui 메서드 호출하여 오버레이 적용
            self.map_manager._update_map_ui(msg)
            
            # 맵과 오버레이를 뷰에 맞게 표시
            self.map_view.setSceneRect(self.map_scene.itemsBoundingRect())
            self.map_view.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio)
            
            # 뷰 업데이트 강제
            self.map_view.viewport().update()
            
            # 수정 모드이고 맵이 처음 로드된 경우 마커 표시
            if self.is_edit_mode and self.clicked_x is not None and self.clicked_y is not None:
                # 맵이 완전히 로드된 후 마커 표시 (약간의 지연)
                QTimer.singleShot(500, self.update_robot_marker)
                
        except Exception as e:
            print(f"맵 업데이트 콜백 중 오류 발생: {str(e)}")

    def resizeEvent(self, event):
        """위젯 크기 변경 시 호출"""
        super().resizeEvent(event)
        # 맵 뷰가 있고 씬이 비어있지 않을 때만 실행
        if hasattr(self, 'map_view') and hasattr(self, 'map_scene') and self.map_scene and not self.map_scene.sceneRect().isEmpty():
            # 맵 뷰의 변환 행렬 초기화
            self.map_view.resetTransform()
            # 맵 뷰가 씬 전체를 보여주도록 설정
            self.map_view.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio)
            print(f"[LocationAddView] resizeEvent - 맵 뷰 크기 조정: {self.map_view.size()}")

    def setup_connections(self):
        """시그널 연결"""
        # 기본 버튼 연결
        self.register_button.clicked.connect(self.on_save)
        self.cancel_button.clicked.connect(self.on_cancel_button_clicked)
        # 모두 삭제 버튼 연결
        # if hasattr(self, 'delete_all_button'):
        #     self.delete_all_button.clicked.connect(self.on_area_delete_button_clicked)
        
        # 맵 컨트롤 버튼 연결
        self.map_zoom_in.clicked.connect(self.on_map_zoom_in)
        self.map_zoom_out.clicked.connect(self.on_map_zoom_out)
        self.map_move_refresh.clicked.connect(self.on_map_refresh)
        
        # 새로 추가된 맵 컨트롤 버튼 연결
        if hasattr(self, 'map_up') and self.map_up:
            self.map_up.clicked.connect(self.on_map_move_up)
        if hasattr(self, 'map_down') and self.map_down:
            self.map_down.clicked.connect(self.on_map_move_down)
        if hasattr(self, 'map_left') and self.map_left:
            self.map_left.clicked.connect(self.on_map_move_left)
        if hasattr(self, 'map_right') and self.map_right:
            self.map_right.clicked.connect(self.on_map_move_right)
        
        # 로봇 컨트롤 버튼 연결
        self.robot_move_up.pressed.connect(lambda: self.start_moving('up'))
        self.robot_move_down.pressed.connect(lambda: self.start_moving('down'))
        self.robot_move_left.pressed.connect(lambda: self.start_moving('left'))
        self.robot_move_right.pressed.connect(lambda: self.start_moving('right'))
        self.robot_rotate_right.pressed.connect(lambda: self.start_rotating('right'))
        self.robot_rotate_left.pressed.connect(lambda: self.start_rotating('left'))

        self.robot_move_up.released.connect(self.stop_moving)
        self.robot_move_down.released.connect(self.stop_moving)
        self.robot_move_left.released.connect(self.stop_moving)
        self.robot_move_right.released.connect(self.stop_moving)
        self.robot_rotate_right.released.connect(self.stop_moving)
        self.robot_rotate_left.released.connect(self.stop_moving)

        # 클릭 이벤트 추가 (한 번 클릭시 한 스텝 이동)
        self.robot_move_up.clicked.connect(lambda: self.move_robot('up'))
        self.robot_move_down.clicked.connect(lambda: self.move_robot('down'))
        self.robot_move_left.clicked.connect(lambda: self.move_robot('left'))
        self.robot_move_right.clicked.connect(lambda: self.move_robot('right'))
        self.robot_rotate_right.clicked.connect(lambda: self.rotate_robot('right'))
        self.robot_rotate_left.clicked.connect(lambda: self.rotate_robot('left'))

        # MapManager의 영역 설정 모드 상태 변경 시그널 연결
        if hasattr(self, 'map_manager') and self.map_manager is not None:
            self.area_setting_changed.connect(self.map_manager.on_area_setting_changed)

    def start_moving(self, direction):
        """로봇 이동 시작"""
        self.movement_timer = QTimer(self)
        self.movement_timer.timeout.connect(lambda: self.move_robot(direction))
        self.movement_timer.start(100)  # 100ms마다 이동

    def start_rotating(self, direction):
        """로봇 회전 시작"""
        self.rotation_timer = QTimer(self)
        self.rotation_timer.timeout.connect(lambda: self.rotate_robot(direction))
        self.rotation_timer.start(100)  # 100ms마다 회전

    def stop_moving(self):
        """로봇 이동/회전 중지"""
        if hasattr(self, 'movement_timer'):
            self.movement_timer.stop()
        if hasattr(self, 'rotation_timer'):
            self.rotation_timer.stop()

    def move_robot(self, direction):
        """로봇 이동"""

        if hasattr(self, 'map_manager'):
            # 마커 이동 시도
            result = self.map_manager.move_robot(direction, self.ROBOT_MOVE_STEP)
            
            if result:
                # 마커 이동 후 씬 좌표 가져오기
                if hasattr(self.map_manager, 'original_scene_pos') and self.map_manager.original_scene_pos:
                    scene_pos = self.map_manager.original_scene_pos
                    print(f"마커 이동 후 씬 좌표: ({scene_pos.x()}, {scene_pos.y()})")
                    
                    # 씬 좌표를 맵 좌표로 변환
                    map_x, map_y, _ = self.convert_scene_to_map_coords(scene_pos)
                    print(f"변환된 맵 좌표: ({map_x:.3f}, {map_y:.3f})")
                    
                    # 디버그 텍스트 업데이트
                    self.clicked_x = map_x
                    self.clicked_y = map_y
                    self.update_debug_text(map_x, map_y, self.robot_yaw)
                
                # 맵 매니저에서 변환된 실제 좌표도 확인
                if self.map_manager.last_clicked_real_pos:
                    real_x, real_y = self.map_manager.last_clicked_real_pos
                    print(f"[LocationAddView] 로봇 이동: {direction}, 맵 매니저 좌표: ({real_x:.3f}, {real_y:.3f})")

                if direction == 'up':
                    self.robot_move_up_signal.emit()
                elif direction == 'down':
                    self.robot_move_down_signal.emit()
                elif direction == 'left':
                    self.robot_move_left_signal.emit()
                elif direction == 'right':
                    self.robot_move_right_signal.emit()
            else:
                print(f"[LocationAddView] 로봇 이동 실패: {direction} - 맵 경계 또는 금지구역")
                ToastManager.instance().show_toast("맵 경계를 벗어날 수 없습니다.", self)
                # 맵 매니저에서 변환된 실제 좌표 확인 (이동 실패 시에도 현재 좌표 표시)
                if self.map_manager.last_clicked_real_pos:
                    real_x, real_y = self.map_manager.last_clicked_real_pos
                    print(f"[LocationAddView] 현재 로봇 좌표: ({real_x:.3f}, {real_y:.3f})")

    def rotate_robot(self, direction):
        """로봇 회전"""
        if direction == 'right':
            self.robot_rotate_right_signal.emit()
        elif direction == 'left':
            self.robot_rotate_left_signal.emit()

    # 맵 컨트롤 함수들
    def on_map_zoom_in(self):
        """맵 확대"""
        self.map_zoom_in_signal.emit()
        
    def on_map_zoom_out(self):
        """맵 축소"""
        self.map_zoom_out_signal.emit()
        
    def on_map_move_up(self):
        """맵 위로 이동"""
        if hasattr(self, 'map_manager') and self.map_manager:
            self.map_manager.move_map('up', 50)
        
    def on_map_move_down(self):
        """맵 아래로 이동"""
        if hasattr(self, 'map_manager') and self.map_manager:
            self.map_manager.move_map('down', 50)
        
    def on_map_move_left(self):
        """맵 왼쪽으로 이동"""
        if hasattr(self, 'map_manager') and self.map_manager:
            self.map_manager.move_map('left', 50)
        
    def on_map_move_right(self):
        """맵 오른쪽으로 이동"""
        if hasattr(self, 'map_manager') and self.map_manager:
            self.map_manager.move_map('right', 50)
    
    def update_robot_marker(self):
        """
        로봇 마커 위치 업데이트
        
        클릭한 위치를 기반으로 마커를 표시합니다.
        로봇의 실시간 위치는 더 이상 사용하지 않습니다.
        사용자가 명시적으로 클릭한 경우에만 마커를 표시합니다.
        """
        try:
            # map_manager가 초기화되지 않았으면 무시
            if not hasattr(self, 'map_manager') or self.map_manager is None:
                print("맵 매니저가 초기화되지 않아 로봇 마커를 업데이트할 수 없습니다.")
                return
                
            # 클릭 위치가 없거나 사용자가 명시적으로 클릭하지 않은 경우 마커를 표시하지 않음
            if self.clicked_x is None or self.clicked_y is None or not self.user_has_clicked:
                print("마커 업데이트 실패: 위치 정보 없거나 사용자가 아직 클릭하지 않음")
                return
            
            # 현재 위치 값 가져오기 (클릭된 위치)
            real_x = self.clicked_x
            real_y = self.clicked_y
            print("[update_robot_marker] 클릭 위치:", real_x, real_y)
            
            # yaw 값을 그대로 사용
            self.map_manager.last_yaw = self.robot_yaw
                
            # 마커 업데이트 요청
            success = self.map_manager.update_marker(real_x, real_y, self.map_manager.last_yaw)
            
            if success:
                # 디버그 텍스트 업데이트
                self.update_debug_text(real_x, real_y, self.robot_yaw)
                print(f"[LocationAddView] 로봇 마커 업데이트 성공: x={self.clicked_x:.3f}, y={self.clicked_y:.3f}, θ={self.robot_yaw:.2f}")
            else:
                print(f"마커 업데이트 실패: 좌표계 변환 오류")
                
        except Exception as e:
            print(f"로봇 마커 업데이트 중 오류 발생: {str(e)}")

    def is_point_in_forbidden_area(self, scene_x, scene_y):
        """씬 좌표 기준으로 금지구역(사각형) 내부인지 판별, 내부면 해당 rect 반환"""
        for btn in self.area_buttons:
            if btn.property("area_type") == "금지 구역" and hasattr(btn, 'rect_item') and btn.rect_item:
                rect = btn.rect_item.rect()
                if rect.contains(QPointF(scene_x, scene_y)):
                    return rect
        return None

    def get_nearest_point_outside_rect(self, rect, x, y):
        """사각형 내부 클릭 시, 가장 가까운 외부 경계점 반환 (씬 좌표 기준)"""
        px = min(max(x, rect.left()), rect.right())
        py = min(max(y, rect.top()), rect.bottom())
        # 내부라면 경계선 바깥으로 1픽셀 이동
        if rect.contains(QPointF(x, y)):
            # x가 더 가까우면 x축 바깥, y가 더 가까우면 y축 바깥
            dx_left = abs(x - rect.left())
            dx_right = abs(x - rect.right())
            dy_top = abs(y - rect.top())
            dy_bottom = abs(y - rect.bottom())
            min_dist = min(dx_left, dx_right, dy_top, dy_bottom)
            if min_dist == dx_left:
                px = rect.left() - 1
            elif min_dist == dx_right:
                px = rect.right() + 1
            elif min_dist == dy_top:
                py = rect.top() - 1
            else:
                py = rect.bottom() + 1
        return px, py

    def check_forbidden_area(self, scene_pos):
        """
        클릭 위치가 금지구역 내부인지 확인하는 함수
        
        Parameters:
            scene_pos (QPointF): 확인할 씬 좌표
            
        Returns:
            tuple: (is_forbidden_area, touched_area_index, touched_button)
        """
        is_forbidden_area = False
        touched_area_index = -1
        touched_button = None
        
        scene_x, scene_y = scene_pos.x(), scene_pos.y()
        
        if hasattr(self, 'area_buttons') and self.area_buttons:
            print(f"[check_forbidden_area] area_buttons 길이: {len(self.area_buttons)}")
            
            # 버튼 확인
            for i, button in enumerate(self.area_buttons):
                if button is None:
                    print(f"[check_forbidden_area] 경고: area_buttons[{i}]가 None입니다.")
                    continue
                    
                try:
                    # 버튼의 현재 geometry를 사용하여 클릭 위치가 버튼 내부인지 확인
                    if hasattr(button, 'geometry'):
                        # 우선 경로 버튼은 금지구역으로 간주하지 않음
                        area_type = button.property("area_type")
                        if area_type == "우선 경로":
                            continue
                            
                        btn_rect = button.geometry()
                        # 클릭 위치가 버튼 영역 내에 있는지 확인
                        if btn_rect.contains(int(scene_x), int(scene_y)):
                            is_forbidden_area = True
                            touched_area_index = i
                            touched_button = button
                            print(f"[check_forbidden_area] 금지 구역 {i+1} 내부 터치 감지!")
                            break
                except Exception as e:
                    print(f"[check_forbidden_area] 버튼 {i} 확인 중 오류: {e}")
        
        return is_forbidden_area, touched_area_index, touched_button
    
    def check_position_in_other_forbidden_areas(self, pos_x, pos_y, exclude_index=-1):
        """
        위치가 다른 금지구역 내에 있는지 확인하는 함수
        
        Parameters:
            pos_x (float): 확인할 X 좌표
            pos_y (float): 확인할 Y 좌표
            exclude_index (int): 제외할 금지구역 인덱스
            
        Returns:
            tuple: (is_in_forbidden_area, forbidden_area_index)
        """
        is_in_forbidden_area = False
        forbidden_area_index = -1
        
        for j, button in enumerate(self.area_buttons):
            # 제외할 인덱스가 아니고 버튼이 존재하는 경우만 확인
            if j != exclude_index and button is not None:
                try:
                    if hasattr(button, 'geometry'):
                        # 우선 경로 버튼은 금지구역으로 간주하지 않음
                        area_type = button.property("area_type")
                        if area_type == "우선 경로":
                            continue
                            
                        btn_rect = button.geometry()
                        # 위치가 버튼 영역 내에 있는지 확인
                        if btn_rect.contains(int(pos_x), int(pos_y)):
                            is_in_forbidden_area = True
                            forbidden_area_index = j
                            print(f"[check_position_in_other_forbidden_areas] 위치 ({pos_x}, {pos_y})가 금지 구역 {j+1} 내에 있음!")
                            break
                except Exception as e:
                    print(f"[check_position_in_other_forbidden_areas] 버튼 {j} 확인 중 오류: {e}")
        
        return is_in_forbidden_area, forbidden_area_index
    
    def find_safe_position(self, scene_pos, touched_button, touched_area_index, scene_rect):
        """
        금지구역 외부의 안전한 위치를 찾는 함수
        
        Parameters:
            scene_pos (QPointF): 원래 클릭 위치
            touched_button (QPushButton): 터치된 금지구역 버튼
            touched_area_index (int): 터치된 금지구역 인덱스
            scene_rect (QRectF): 씬 경계 직사각형
            
        Returns:
            QPointF: 안전한 위치
        """
        button_rect = touched_button.geometry()
        
        # 버튼의 중심점과 경계 계산
        button_center = QPointF(
            button_rect.x() + button_rect.width()/2,
            button_rect.y() + button_rect.height()/2
        )
        
        # 클릭 위치에서 버튼 중심으로 향하는 벡터 계산
        dx = scene_pos.x() - button_center.x()
        dy = scene_pos.y() - button_center.y()
        
        # 벡터 정규화
        length = math.sqrt(dx*dx + dy*dy)
        if length > 0:
            dx /= length
            dy /= length
        else:
            # 클릭이 정확히 중심에 있는 경우, 임의 방향 선택
            dx = 1.0
            dy = 0.0
        
        print(f"[find_safe_position] 금지 구역 중심: ({button_center.x()}, {button_center.y()})")
        print(f"[find_safe_position] 방향 벡터: ({dx}, {dy})")
        
        # 안전 여백
        safety_margin = 10  # 더 큰 여백 사용
        
        # 직사각형에서 가장 먼 모서리까지의 거리 계산 (대각선 길이의 절반)
        max_distance = math.sqrt(button_rect.width()**2 + button_rect.height()**2) / 2
        
        # 안전한 위치 계산 (최대 거리 + 안전 여백 사용)
        safe_distance = max_distance + safety_margin
        safe_pos_x = button_center.x() + dx * safe_distance
        safe_pos_y = button_center.y() + dy * safe_distance
        
        # 안전한 위치가 씬 경계 내에 있는지 확인하고 조정
        if safe_pos_x < scene_rect.left():
            safe_pos_x = scene_rect.left()
        elif safe_pos_x > scene_rect.right():
            safe_pos_x = scene_rect.right()
            
        if safe_pos_y < scene_rect.top():
            safe_pos_y = scene_rect.top()
        elif safe_pos_y > scene_rect.bottom():
            safe_pos_y = scene_rect.bottom()
        
        # 계산된 안전한 위치가 여전히 금지 구역 내에 있는지 확인
        is_still_in_forbidden_area = (
            safe_pos_x >= button_rect.left() and
            safe_pos_x <= button_rect.right() and
            safe_pos_y >= button_rect.top() and
            safe_pos_y <= button_rect.bottom()
        )
        
        # 여전히 금지 구역 내에 있다면 더 멀리 이동
        if is_still_in_forbidden_area:
            print("[find_safe_position] 계산된 안전 위치가 여전히 금지 구역 내에 있음. 더 멀리 이동")
            # 거리를 두 배로 늘림
            safe_distance = safe_distance * 2
            safe_pos_x = button_center.x() + dx * safe_distance
            safe_pos_y = button_center.y() + dy * safe_distance
            
            # 씬 경계 내에 있는지 다시 확인
            if safe_pos_x < scene_rect.left():
                safe_pos_x = scene_rect.left()
            elif safe_pos_x > scene_rect.right():
                safe_pos_x = scene_rect.right()
                
            if safe_pos_y < scene_rect.top():
                safe_pos_y = scene_rect.top()
            elif safe_pos_y > scene_rect.bottom():
                safe_pos_y = scene_rect.bottom()
        
        # 계산된 안전 위치가 다른 금지구역 내에 있는지 확인
        is_in_another_forbidden_area, another_forbidden_area_index = self.check_position_in_other_forbidden_areas(
            safe_pos_x, safe_pos_y, touched_area_index
        )
        
        # 다른 금지구역 내에 있다면 방향을 바꿔서 다시 계산
        max_attempts = 8  # 최대 시도 횟수 (8방향)
        attempt = 0
        original_dx, original_dy = dx, dy
        
        while is_in_another_forbidden_area and attempt < max_attempts:
            attempt += 1
            # 방향 벡터를 45도씩 회전
            angle = math.pi / 4 * attempt  # 45도 * 시도 횟수
            dx = math.cos(angle) * original_dx - math.sin(angle) * original_dy
            dy = math.sin(angle) * original_dx + math.cos(angle) * original_dy
            
            # 벡터 정규화
            length = math.sqrt(dx*dx + dy*dy)
            if length > 0:
                dx /= length
                dy /= length
            
            print(f"[find_safe_position] 새 방향 벡터 시도 {attempt}: ({dx}, {dy})")
            
            # 새 방향으로 안전 위치 계산
            safe_distance = max_distance * 2 + safety_margin  # 더 멀리 이동
            safe_pos_x = button_center.x() + dx * safe_distance
            safe_pos_y = button_center.y() + dy * safe_distance
            
            # 씬 경계 내에 있는지 확인
            if safe_pos_x < scene_rect.left():
                safe_pos_x = scene_rect.left()
            elif safe_pos_x > scene_rect.right():
                safe_pos_x = scene_rect.right()
                
            if safe_pos_y < scene_rect.top():
                safe_pos_y = scene_rect.top()
            elif safe_pos_y > scene_rect.bottom():
                safe_pos_y = scene_rect.bottom()
            
            # 새 위치가 다른 금지구역 내에 있는지 다시 확인
            is_in_another_forbidden_area, another_forbidden_area_index = self.check_position_in_other_forbidden_areas(
                safe_pos_x, safe_pos_y, touched_area_index
            )
            
            if not is_in_another_forbidden_area:
                print(f"[find_safe_position] 안전한 위치를 찾았습니다: ({safe_pos_x:.2f}, {safe_pos_y:.2f})")
                break
        
        # 모든 방향을 시도해도 안전한 위치를 찾지 못한 경우
        if is_in_another_forbidden_area:
            print("[find_safe_position] 모든 방향을 시도했지만 안전한 위치를 찾지 못했습니다.")
            # 가장 먼 모서리로 이동 (현재 위치에서 가장 먼 씬 모서리)
            scene_center_x = scene_rect.width() / 2
            scene_center_y = scene_rect.height() / 2
            
            # 씬 중심에서 가장 먼 모서리 선택
            corners = [
                (scene_rect.left(), scene_rect.top()),      # 좌상단
                (scene_rect.right(), scene_rect.top()),     # 우상단
                (scene_rect.right(), scene_rect.bottom()),  # 우하단
                (scene_rect.left(), scene_rect.bottom())    # 좌하단
            ]
            
            max_dist = 0
            best_corner = corners[0]
            
            for corner in corners:
                dist = math.sqrt((corner[0] - button_center.x())**2 + (corner[1] - button_center.y())**2)
                if dist > max_dist:
                    max_dist = dist
                    best_corner = corner
            
            safe_pos_x, safe_pos_y = best_corner
            print(f"[find_safe_position] 가장 먼 모서리로 이동: ({safe_pos_x:.2f}, {safe_pos_y:.2f})")
        
        return QPointF(safe_pos_x, safe_pos_y)

    def eventFilter(self, obj, event):
        """이벤트 필터"""
        if obj == self.map_view.viewport():
            # 터치 드래그 처리
            if event.type() == QEvent.MouseButtonPress:
                if event.button() == Qt.LeftButton:
                    # 드래그 시작 위치 저장
                    self.drag_start_pos = event.pos()
                    self.last_drag_pos = event.pos()
                    self.is_dragging = True
                    
                    # 클릭 위치를 씬 좌표로 변환
                    scene_pos = self.map_view.mapToScene(event.pos())
                    scene_x, scene_y = scene_pos.x(), scene_pos.y()

                    # 영역 설정 모드일 때
                    if self.is_area_setting_screen:

                        # 장애물 확인
                        if self.is_obstacle_at_point(scene_pos):
                            print(f"터치 위치 ({scene_pos.x():.2f}, {scene_pos.y():.2f})는 장애물입니다.")
                            # 여기에 장애물일 때 수행할 작업 추가
                        else:
                            print(f"터치 위치 ({scene_pos.x():.2f}, {scene_pos.y():.2f})는 자유 공간입니다.")
                            # 여기에 장애물이 아닐 때 수행할 작업 추가

                        # restricted_area_button과 priority_path_area_button 모두 체크되지 않은 경우
                        if (not hasattr(self, 'restricted_area_button') or not self.restricted_area_button.isChecked()) and \
                           (not hasattr(self, 'priority_path_area_button') or not self.priority_path_area_button.isChecked()):
                            # 실제 맵 위치 계산 (새로운 함수 사용)
                            map_x, map_y, _ = self.convert_scene_to_map_coords(scene_pos)
                            print(f"[영역 설정 모드] 버튼 체크 없음 - 터치 위치 좌표: 씬({scene_x}, {scene_y}), 맵({map_x}, {map_y})")
                            return True
                        
                        # 클릭한 위치가 기존 버튼 내부인지 확인
                        button_clicked = False
                        for btn in self.area_buttons:
                            if hasattr(btn, 'is_area_button') and btn.is_area_button:
                                # 버튼의 geometry를 사용하여 클릭 위치가 버튼 내부인지 확인
                                btn_rect = btn.geometry()
                                if btn_rect.contains(int(scene_x), int(scene_y)):
                                    print(f"기존 버튼 클릭됨: 위치=({btn_rect.x()}, {btn_rect.y()}), 크기={btn_rect.width()}x{btn_rect.height()}")
                                    btn.setChecked(True)
                                    self.on_area_button_clicked(btn)
                                    button_clicked = True
                                    break
                        
                        # 기존 버튼을 클릭하지 않은 경우에만 새 버튼 생성
                        if not button_clicked:
                            button = self.create_forbidden_area_button(scene_x, scene_y)
                        
                        return True
                    else:
                        # 마커 기능 활성화
                        if self.is_marker_enabled:
                            print("[eventFilter] 마커 기능 활성화: 클릭 이벤트 감지")

                            # 맵 이미지 영역 내부인지 확인
                            if not self.is_point_in_map_image(scene_pos):
                                print(f"[eventFilter] 맵 이미지 영역 외부 클릭: ({scene_pos.x()}, {scene_pos.y()})")
                                ToastManager.instance().show_toast("맵 이미지 영역 내에서만 마커를 생성할 수 있습니다.", self)
                                return True

                            # 금지구역 확인 (리팩토링된 함수 사용)
                            is_forbidden_area, touched_area_index, touched_button = self.check_forbidden_area(scene_pos)
                            
                            if is_forbidden_area:
                                print(f"[eventFilter] 금지 구역 {touched_area_index+1} 터치됨!")
                                
                                # 씬 경계 가져오기
                                scene_rect = self.map_manager.map_scene.sceneRect()
                                if scene_rect.isEmpty() and self.map_manager.map_overlay_pixmap:
                                    scene_rect = self.map_manager.map_overlay_pixmap.rect()
                                
                                # 안전한 위치 찾기 (리팩토링된 함수 사용)
                                safe_scene_pos = self.find_safe_position(
                                    scene_pos, touched_button, touched_area_index, scene_rect
                                )
                                
                                # 안전한 위치로 좌표 업데이트
                                scene_pos = safe_scene_pos

                            # 맵 정보가 없으면 기본값 사용
                            if not self.map_manager.map_info:
                                print("[eventFilter] 맵 정보 없음. 기본 맵 정보로 설정")
                                self.map_manager.setup_default_map_info()
                            
                            scene_rect = self.map_manager.map_scene.sceneRect()
                            if scene_rect.isEmpty() and self.map_manager.map_overlay_pixmap:
                                scene_rect = self.map_manager.map_overlay_pixmap.rect()
                            
                            print(f"[eventFilter] scene_pos: {scene_pos.x()}, {scene_pos.y()}")
                            print(f"[eventFilter] scene_rect: {scene_rect}")
                            
                            # 새로운 함수를 사용하여 맵 좌표 계산
                            x, y, transformed_pos = self.convert_scene_to_map_coords(scene_pos)
                            if self.map_manager.overlay_transform:
                                print(f"[eventFilter] 변환 전 scene_pos: {scene_pos.x()}, {scene_pos.y()}")
                                print(f"[eventFilter] 변환 후 original_pos: {transformed_pos.x()}, {transformed_pos.y()}")
                                adjusted_scene_x = scene_rect.width() - transformed_pos.x()
                                print(f"[eventFilter] 조정된 씬 좌표: ({adjusted_scene_x}, {transformed_pos.y()})")
                                print(f"[eventFilter] 최종 맵 좌표: x={x}, y={y}")
                            else:
                                adjusted_scene_x = scene_rect.width() - sceneP_pos.x()
                                print(f"[eventFilter] (변환 없음) adjusted_scene_x: {adjusted_scene_x}, x={x}, y={y}")
                            
                            self.map_manager.last_clicked_scene_pos = (x,y)
                            self.map_manager.original_scene_pos = scene_pos

                            # 클릭한 위치 좌표 저장
                            self.clicked_x = x
                            self.clicked_y = y
                            self.robot_yaw = 0.0
                            
                            # 사용자가 명시적으로 클릭했음을 표시
                            self.user_has_clicked = True

                            # 클릭 위치를 시그널로 전달 (실제 좌표 전달)
                            print(f"[eventFilter] 최종 position_clicked.emit(x={x}, y={y})")
                            self.position_clicked.emit(x, y)
                            # 클릭 위치에 마커 표시
                            print(f"[eventFilter] show_click_marker(scene_pos=({scene_pos.x()}, {scene_pos.y()}), yaw=0.0)")
                            self.map_manager.show_click_marker(scene_pos, 0.0)
                            # 디버그 정보 출력
                            print(f"[eventFilter] update_debug_text(x={x}, y={y}, yaw=0.0)")
                            self.update_debug_text(x, y, 0.0)
                    return True
                
            elif event.type() == QEvent.MouseMove and self.is_dragging:
                # 드래그 중일 때 맵 이동 처리
                if self.drag_start_pos is not None and self.last_drag_pos is not None:
                    # 현재 위치와 마지막 위치의 차이 계산
                    delta = event.pos() - self.last_drag_pos
                    self.last_drag_pos = event.pos()
                    
                    # 맵 이동 (상하좌우)
                    if hasattr(self, 'map_manager') and self.map_manager:
                        # 이동 방향 결정 (delta가 양수면 반대 방향으로 이동)
                        if delta.x() != 0:
                            direction = 'right' if delta.x() < 0 else 'left'
                            self.map_manager.move_map(direction, abs(delta.x()))
                        
                        if delta.y() != 0:
                            direction = 'down' if delta.y() < 0 else 'up'
                            self.map_manager.move_map(direction, abs(delta.y()))
                    
                    return True
                
            elif event.type() == QEvent.MouseButtonRelease:
                # 드래그 종료
                if event.button() == Qt.LeftButton:
                    self.is_dragging = False
                    self.drag_start_pos = None
                    self.last_drag_pos = None
                    return True
                
        return super().eventFilter(obj, event)
    
    def showEvent(self, event):
        """위젯이 표시될 때 호출"""
        super().showEvent(event)
        
        # 맵 초기화 확인
        if not hasattr(self, 'map_manager') or self.map_manager is None:
            self.setup_map()
            print("[showEvent] 맵 초기화됨")
        
        # 네비게이션 컨트롤 위젯 초기 상태 설정
        if hasattr(self, 'map_control_type_button') and self.map_control_type_button:
            self.map_control_type_button.setChecked(True)
            self.on_map_control_type_clicked()
            
        # 영역 이동 위젯 숨김 (영역 설정 모드가 아닐 때는 항상 숨김)
        if hasattr(self, 'move_area_widget') and self.move_area_widget:
            self.move_area_widget.hide()
            
        # map_control_button 상태에 따라 위젯 표시 여부 업데이트
        self.update_control_widgets_visibility()
        
        # 맵 변환 정보 다시 확인 및 로드
        if hasattr(self, 'map_manager') and self.map_manager:
            if self.check_map_transform():
                self.map_manager.load_overlay_transform()
                
                # 맵 이미지 업데이트 강제
                if self.map_manager.map_overlay_pixmap:
                    self.map_manager._update_map_ui(self.map_manager.map_overlay_pixmap)
                    
                    # 사용자가 명시적으로 클릭한 경우에만 마커 업데이트
                    # 자동으로 마커 업데이트하지 않음
        
        # 맵 크기 조정 - 여러 시점에서 시도
        if hasattr(self, 'map_scene') and self.map_scene and not self.map_scene.sceneRect().isEmpty():
            self.map_view.resetTransform()
            self.map_view.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio)
            # 약간의 지연 후 다시 시도 (UI가 완전히 렌더링된 후)
            QTimer.singleShot(100, lambda: self.map_view.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio))
            QTimer.singleShot(300, lambda: self.map_view.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio))
        
        # 금지구역 데이터 로드 (지연 실행)
        QTimer.singleShot(200, self.safe_load_forbidden_areas)
        
        # 영역 인덱스 표시 여부 설정 (OtherSettingController에서 설정된 값 적용)
        # 기본값은 False로 설정 (인덱스 비표시)
        QTimer.singleShot(300, lambda: self.set_area_index_visibility(self.show_area_index))

    def safe_load_forbidden_areas(self):
        """안전하게 금지구역 로드 (예외 처리 포함)"""
        try:
            # 기존 버튼 정리
            if hasattr(self, 'area_buttons') and self.area_buttons:
                for button in self.area_buttons[:]:  # 복사본으로 반복
                    try:
                        if button.scene():
                            button.scene().removeItem(button)
                        if hasattr(self, 'area_list_layout'):
                            self.area_list_layout.removeWidget(button)
                        button.setParent(None)
                        button.deleteLater()
                    except Exception as e:
                        print(f"버튼 정리 중 오류: {e}")
                self.area_buttons.clear()
            
            # 금지 구역 발행자가 있으면 초기화 메시지 발행
            if hasattr(self, 'keepout_zone_publisher'):
                try:
                    # 빈 메시지를 발행하여 기존 금지 구역 초기화 신호 전송
                    self.keepout_zone_publisher.publish_keepout_zone(0, [])
                    print("[keepout_zone] 금지 구역 초기화 메시지 발행 완료")
                except Exception as e:
                    print(f"[keepout_zone] 금지 구역 초기화 중 오류 발생: {str(e)}")
            
            # 금지구역 로드
            self.load_forbidden_areas()
        except Exception as e:
            print(f"금지구역 로드 중 오류: {e}")
            import traceback
            traceback.print_exc()

    def hideEvent(self, event):
        """위젯이 숨겨질 때 호출"""
        super().hideEvent(event)

        self.cleanup_resources()

    def closeEvent(self, event):
        """윈도우 종료 시 리소스 정리"""
        self.cleanup_resources()
        event.accept()

    def cleanup_resources(self):
        """리소스 정리 함수"""
        import gc
        
        # 버튼 정리
        if hasattr(self, 'area_buttons') and self.area_buttons:
            for button in self.area_buttons[:]:  # 복사본으로 반복
                try:
                    if hasattr(button, 'scene') and button.scene():
                        button.scene().removeItem(button)
                    if hasattr(self, 'area_list_layout') and button in self.area_list_layout.children():
                        self.area_list_layout.removeWidget(button)
                    button.setParent(None)
                    button.deleteLater()
                except Exception as e:
                    print(f"버튼 정리 중 오류: {e}")
            self.area_buttons.clear()
        
        # 타이머 정리
        for attr_name in dir(self):
            if attr_name.endswith('_timer') and hasattr(self, attr_name):
                timer = getattr(self, attr_name)
                try:
                    if hasattr(timer, 'stop'):
                        timer.stop()
                    if hasattr(timer, 'deleteLater'):
                        timer.deleteLater()
                except Exception as e:
                    print(f"{attr_name} 정리 중 오류: {e}")
        
        # 맵 매니저 정리
        if hasattr(self, 'map_manager') and self.map_manager:
            try:
                self.map_manager.cleanup()
            except Exception as e:
                print(f"맵 매니저 정리 중 오류: {e}")
            self.map_manager = None
        
        # 맵 씬 정리
        if hasattr(self, 'map_scene') and self.map_scene:
            try:
                self.map_scene.clear()
            except Exception as e:
                print(f"맵 씬 정리 중 오류: {e}")
        
        # 금지 구역 발행자 정리
        if hasattr(self, 'keepout_zone_publisher'):
            try:
                self.keepout_zone_publisher.cleanup()
                self.keepout_zone_publisher = None
            except Exception as e:
                print(f"금지 구역 발행자 정리 중 오류: {e}")
                
        # 금지 구역 삭제 발행자 정리
        if hasattr(self, 'delete_keepout_zone_publisher'):
            try:
                self.delete_keepout_zone_publisher.cleanup()
                self.delete_keepout_zone_publisher = None
            except Exception as e:
                print(f"금지 구역 삭제 발행자 정리 중 오류: {e}")
                
        # 우선 경로 영역 발행자 정리
        if hasattr(self, 'preferred_area_publisher'):
            try:
                self.preferred_area_publisher.cleanup()
                self.preferred_area_publisher = None
            except Exception as e:
                print(f"우선 경로 영역 발행자 정리 중 오류: {e}")
        
        # 시그널 연결 해제
        try:
            self.disconnect()
        except Exception as e:
            print(f"시그널 연결 해제 중 오류: {e}")
            
        # 가비지 컬렉션 강제 실행
        gc.collect()
        
        print("LocationAddView 리소스 정리 완료")

    def on_save(self):
        """저장 버튼 클릭"""
        if hasattr(self, 'last_detected_callbell_id') and self.last_detected_callbell_id:
            print(f"저장할 UUID: {self.last_detected_callbell_id}")
            self.save_signal.emit()
        else:
            self.save_signal.emit()

    def on_cancel_button_clicked(self):
        """취소 버튼 클릭 (뒤로가기 기능 포함)"""
        self.cancel_signal.emit()

    def on_callbell_detected(self, callbell_id):
        """호출벨이 감지되었을 때 호출되는 메서드
        
        Args:
            callbell_id (str): 감지된 호출벨의 고유 ID (MAC 주소 등)
        """
        # 이전에 감지된 호출벨과 동일한 경우 무시
        if self.is_callbell_active and callbell_id == self.last_detected_callbell_id:
            print(f"동일한 호출벨 감지됨 (무시): {callbell_id}")
            return
            
        # 새로운 호출벨이 감지된 경우
        if self.is_callbell_active and callbell_id != self.last_detected_callbell_id:
            print(f"새로운 호출벨 감지됨: {callbell_id}")
            # 모든 호출벨 버튼 숨기기
            self.not_called_button.setVisible(False)
            self.callbell_activated_button.setVisible(False)
            
            # updating_call_button 활성화
            self.updating_call_button.setVisible(True)
            
            # 500ms 후에 다시 callbell_activated_button으로 전환
            QTimer.singleShot(500, lambda: self.activate_callbell(callbell_id))
        else:
            # 처음 호출벨이 감지된 경우
            print(f"첫 호출벨 감지됨: {callbell_id}")
            self.activate_callbell(callbell_id)
    
    def activate_callbell(self, callbell_id):
        """호출벨을 활성화 상태로 변경
        
        Args:
            callbell_id (str): 활성화할 호출벨의 고유 ID
        """
        self.last_detected_callbell_id = callbell_id
        self.is_callbell_active = True
        
        # 모든 버튼 숨기기
        self.not_called_button.setVisible(False)
        self.updating_call_button.setVisible(False)
        
        # 활성화 버튼만 표시
        self.callbell_activated_button.setVisible(True)
    
    def reset_callbell(self):
        """호출벨 상태를 초기화"""
        self.last_detected_callbell_id = None
        self.is_callbell_active = False
        
        # 모든 버튼 숨기기
        self.callbell_activated_button.setVisible(False)
        self.updating_call_button.setVisible(False)
        
        # 기본 버튼만 표시
        self.not_called_button.setVisible(True)

    def set_robot_position(self, x, y, yaw):
        """
        로봇 위치를 설정하는 함수입니다.
        더 이상 실시간 위치를 받아오지 않지만, 호환성을 위해 유지합니다.
        """
        # 로봇 위치 정보는 별도 변수로 저장 (맵 클릭 위치와 구분)
        self.robot_x = x
        self.robot_y = y
        self.robot_yaw = yaw
        
        # 사용자가 명시적으로 클릭하기 전까지는 마커를 생성하지 않음
        # 디버그 텍스트만 업데이트
        if hasattr(self, 'clicked_x') and hasattr(self, 'clicked_y') and self.clicked_x is not None and self.clicked_y is not None:
            self.update_debug_text(self.clicked_x, self.clicked_y, self.robot_yaw)

    def get_location_data(self):
        """현재 입력된 위치 데이터 반환 (좌표 및 회전값 포함)"""
        data = {
            'name': self.destination_name_input.text().strip(),
            'callbell_id': self.last_detected_callbell_id,
            'clicked_x': getattr(self, 'clicked_x', None),
            'clicked_y': getattr(self, 'clicked_y', None),
            'clicked_theta': self.robot_yaw,  # 로봇 방향 저장
            'robot_x': getattr(self, 'robot_x', None),
            'robot_y': getattr(self, 'robot_y', None),
            'robot_theta': getattr(self, 'robot_yaw', None)
        }
        print(f"LocationAddView - 전달할 데이터: {data}")
        print(f"LocationAddView - 현재 호출벨 ID: {self.last_detected_callbell_id}")
        return data

    def set_location_data(self, data):
        """위치 데이터 설정"""
        if data:
            print(f"위치 데이터 설정: {data}")
            # 위치 데이터 로딩 중 플래그 - 여러 번 마커 업데이트 방지
            self.loading_location = True
            
            self.destination_name_input.setText(data.get('name', ''))

                        # 호출벨 ID 설정 (호출 상태가 있으면 호출됨으로 표시)            
            if 'unique_id' in data and data['unique_id']:
                callbell_id = data['unique_id']
                
                # 호출벨 활성화
                self.on_callbell_detected(callbell_id)
                
                # 호출벨 상태 로그 출력
                print(f"호출벨 ID 설정: {callbell_id}, 호출벨 활성화됨")

            # 로봇 위치/방향 설정 (있으면)
            position = data.get('position', {})
            if position:
                self.clicked_x = position.get('x')
                self.clicked_y = position.get('y')
                self.robot_yaw = position.get('yaw', 0.0)
                
                print(f"위치 설정: x={self.clicked_x}, y={self.clicked_y}, yaw={self.robot_yaw}")
                
                # 맵이 표시된 후에 마커를 업데이트하기 위한 타이머 설정
                if self.clicked_x is not None and self.clicked_y is not None:
                    # 즉시 한 번 시도
                    self.update_robot_marker()
                    # 맵이 로드된 직후 시도 (짧은 딜레이)
                    QTimer.singleShot(500, self.update_robot_marker)
                    # 맵이 완전히 로드된 후 시도 (긴 딜레이)
                    QTimer.singleShot(1000, self.update_robot_marker)
                    # 추가 시도 (맵 로딩이 지연될 경우를 대비)
                    QTimer.singleShot(2000, self.update_robot_marker)
            
            # 로딩 완료
            self.loading_location = False

    def set_mode_add(self):
        """추가 모드 설정"""
        self.set_mode_edit(False)

    def set_mode_edit(self, is_edit_mode):
        """수정 모드 UI 상태 설정"""
        self.is_edit_mode = is_edit_mode
        # if is_edit_mode:
        #     self.register_button.setText("수정")
        # else:
        #     self.register_button.setText("등록")

    def reset_view(self):
        """뷰를 초기화합니다."""
        # 기본 UI 초기화
        self.destination_name_input.clear()
        
        # 위치 값 초기화
        self.clicked_x = None
        self.clicked_y = None
        self.robot_yaw = 0.0
        
        # 사용자 클릭 상태 초기화
        self.user_has_clicked = False
        
        # 디버그 텍스트 초기화
        if hasattr(self, 'map_data_debug_TextEdit') and self.map_data_debug_TextEdit:
            self.update_debug_text(0.0, 0.0, 0.0)
        
        # 호출벨 상태 초기화
        self.reset_callbell()
        
        # 맵 초기화
        if hasattr(self, 'map_scene') and self.map_scene is not None:
            self.map_scene.clear()
        elif hasattr(self, 'map_manager') and self.map_manager and hasattr(self.map_manager, 'map_scene') and self.map_manager.map_scene is not None:
            self.map_manager.map_scene.clear()
            
        # 영역 설정 초기화
        self.clear_current_rect()
        self.clear_current_circle()
        self.clear_current_line()
        
        # 로봇 마커 초기화
        self.update_robot_marker()
        
        # 맵 매니저가 초기화되었는지 확인
        if hasattr(self, 'map_manager') and self.map_manager is not None:
            # 금지 영역 로드
            self.load_forbidden_areas()
            
            # 맵 변환 확인
            self.check_map_transform()

    def set_unique_id(self, uuid_value):
        """UUID가 추출되었을 때 호출되는 메서드"""
        print(f"LocationAddView - set_unique_id 호출됨: {uuid_value}")
        
        try:
            import sys
            # RingoBellManager 간접 참조로 확인
            if 'models.RingoBellManager' in sys.modules:
                ring_manager = sys.modules['models.RingoBellManager'].RingoBellManager.instance()
                if ring_manager.is_uuid_registered(uuid_value):
                    # 이미 등록된 호출벨인 경우
                    from models.LocationSelectionModel import LocationSelectionModel
                    location_model = LocationSelectionModel()
                    location, location_index = location_model.find_location_by_uuid(uuid_value)
                    
                    if location and location_index >= 0:
                        location_name = location.get('name', '알 수 없음')
                        
                        # MessageBox로 사용자에게 확인 (이름 두 번 표시)
                        message = f"이미 등록된 호출벨입니다 '{location_name}' 목적지를 삭제하시겠습니까?"
                        result = MessageBox.show_message(
                            parent=self, 
                            title="등록된 호출벨",
                            message=message,
                            button_labels=["확인", "취소"]
                        )
                        
                        # 확인 버튼을 눌렀을 경우 (OK 또는 YES)
                        if result == MessageBox.OK or result == MessageBox.YES:
                            # 위치 삭제
                            if location_model.delete_location(location_index):
                                print(f"목적지 '{location_name}' 삭제 성공")
                                
                                # 중요: RingoBellManager의 UUID 캐시 갱신
                                try:
                                    ring_manager.refresh_uuids()
                                    print("RingoBellManager UUID 캐시 갱신됨")
                                    
                                    # RingoBellManager를 통해 MAIN_CONTROLLER 접근
                                    from models.RingoBellManager import MAIN_CONTROLLER
                                    if MAIN_CONTROLLER and hasattr(MAIN_CONTROLLER, 'selection_controller'):
                                        MAIN_CONTROLLER.selection_controller.update_locations()
                                        print("LocationSelectionController 목록 갱신됨")
                                    else:
                                        print("MAIN_CONTROLLER 또는 selection_controller를 찾을 수 없음")
                                except Exception as e:
                                    print(f"캐시 갱신 중 오류: {e}")
                                
                                # 호출벨 감지 메서드 호출
                                self.on_callbell_detected(uuid_value)
                                return
                        else:
                            # 취소 버튼을 눌렀을 경우 아무 일도 하지 않음
                            print(f"목적지 '{location_name}' 삭제 취소")
                            return
        except Exception as e:
            print(f"호출벨 확인 중 오류: {e}")

        # 호출벨 감지 메서드 호출
        self.on_callbell_detected(uuid_value)
    
    # 호출벨 추가 메서드는 새로운 호출벨 감지 로직으로 대체되었습니다.

    # 호출벨 관련 메서드는 on_callbell_detected, activate_callbell, reset_callbell로 대체되었습니다.
    
    def get_random_color(self):
        """중복 없이 랜덤 색상 반환, 모두 소진 시 리셋"""
        if not self.available_colors:
            self.available_colors = self.area_colors.copy()
        color = random.choice(self.available_colors)
        self.available_colors.remove(color)
        return color
    
    # 호출벨 전환 메서드는 새로운 호출벨 감지 로직으로 대체되었습니다.

    def on_position_clicked(self, x, y):
        """그래픽 뷰 클릭 시"""
        if self.is_restricted_area_drawing:
            print("금지 구역 그리기 모드이면")
            button = self.create_forbidden_area_button(x, y)
            if button:
                button.setStyleSheet("""
                    QPushButton {
                        background-color: transparent;
                        border: 2px solid #FF0000;
                        color: #FF0000;
                        font-size: 40px;
                        font-weight: bold;
                    }
                    QPushButton:checked {
                        background-color: rgba(255, 0, 0, 0.3);
                        color: #FF0000;
                    }
                """)
                button.setProperty("area_type", "금지 구역")
        else:
            print("우선 경로 영역 그리기 모드이면")
            button = self.create_forbidden_area_button(x, y)
            # button이 None이면 색상 유효성 검사 실패로 생성 불가
            if button is None:
                print("우선 경로 영역 생성이 취소되었습니다.")
                return
                
            if button:
                button.setStyleSheet("""
                    QPushButton {
                        background-color: transparent;
                        border: 2px solid #0000FF;
                        color: #0000FF;
                        font-size: 40px;
                        font-weight: bold;
                    }
                    QPushButton:checked {
                        background-color: rgba(0, 0, 255, 0.3);
                        color: #0000FF;
                    }
                """)
                button.setProperty("area_type", "우선 경로")

    def update_position_from_model(self):
        """MapManager 모델에서 위치 정보를 가져와 뷰의 상태 업데이트"""
        try:
            if hasattr(self, 'map_manager'):
                # 실제 좌표 가져오기
                if self.map_manager.last_clicked_real_pos:
                    self.clicked_x, self.clicked_y = self.map_manager.last_clicked_real_pos
                
                # 방향 정보 가져오기
                if hasattr(self.map_manager, 'last_yaw'):
                    self.robot_yaw = self.map_manager.last_yaw
                
                print(f"로봇 위치 업데이트: x={self.clicked_x:.3f}, y={self.clicked_y:.3f}, θ={self.robot_yaw:.2f}")
                
                # 디버그 텍스트 업데이트
                self.update_debug_text(self.clicked_x, self.clicked_y, self.robot_yaw)
                
                # 위치 정보가 변경되었으므로 마커도 업데이트
                self.update_robot_marker()
        except Exception as e:
            print(f"Model에서 위치 데이터 업데이트 중 오류 발생: {str(e)}")

    # 호출벨 메뉴 관련 메서드는 새로운 호출벨 감지 로직으로 대체되었습니다.
        
    # 호출벨 메뉴 관련 메서드는 새로운 호출벨 감지 로직으로 대체되었습니다.
        
    # 호출벨 메뉴 관련 메서드는 새로운 호출벨 감지 로직으로 대체되었습니다.

    def toggle_area_control(self):
        """영역 설정 모드 전환"""
        self.is_area_setting_screen = not self.is_area_setting_screen
        self.set_area_button.setChecked(self.is_area_setting_screen)
        
        if self.is_area_setting_screen:
            self.is_marker_enabled = False
            print("영역 설정 모드 활성화")

            # 영역 설정 모드로 들어갈 때 restricted_area_button을 체크 상태로 설정 0620
            self.restricted_area_button.setChecked(False)
            self.is_restricted_area_drawing = True
            self.current_area_type = "restricted"
            # priority_path_area_button은 체크 해제
            self.priority_path_area_button.setChecked(False)
            
            # 영역 설정 모드에서는 set_area_button 숨김
            if hasattr(self, 'set_area_button') and self.set_area_button:
                self.set_area_button.hide()
                
            # 삭제 버튼과 모두 삭제 버튼 표시
            if hasattr(self, 'area_delete_button') and self.area_delete_button:
                self.area_delete_button.setVisible(True)
            if hasattr(self, 'area_delete_all_button') and self.area_delete_all_button:
                self.area_delete_all_button.setVisible(True)
        else:
            self.is_marker_enabled = True
            print("영역 설정 모드 비활성화")
            
            # 일반 모드에서는 set_area_button 표시
            if hasattr(self, 'set_area_button') and self.set_area_button:
                self.set_area_button.show()
        
        # UI 요소 표시 상태 업데이트
        self.update_ui_visibility()
        
        # 영역 설정 상태 변경 시그널 발생
        self.area_setting_changed.emit(self.is_area_setting_screen)

    # 두 점 기준 그리기 모드와 선 그리기 모드 전환 메서드
    def toggle_two_point_drawing(self):
        """두 점 기준 그리기 모드 전환"""
        if not self.is_two_point_drawing:
            print("두 점 기준 그리기 모드 활성화")
            self.is_two_point_drawing = True
            self.is_restricted_area_drawing = False
            self.is_two_point_line_drawing = False

            # center_based_draw_button의 배경을 #2A2A2A로 설정
            if self.center_based_draw_button:
                self.center_based_draw_button.setStyleSheet("""
                    QPushButton {
                        background-image: url(:/file/Registration/center_based_draw.png);
                        background-repeat: no-repeat;
                        background-position: center;
                        background-color: #2A2A2A;
                        border: 1px solid #444444;
                        border-radius: 6px;
                    }
                """)
                self.center_based_draw_button.setText("")

            # two_point_draw_button의 배경을 빨간색으로 설정
            if self.two_point_draw_button:
                self.two_point_draw_button.setStyleSheet("""
                    QPushButton {
                        background-image: url(:/file/Registration/two_point.png);
                        background-repeat: no-repeat;
                        background-position: center;
                        background-color: #FF0000;
                        border: 1px solid #444444;
                        border-radius: 6px;
                    }
                """)
                self.two_point_draw_button.setText("")

            if self.two_point_line_draw_button:
                self.two_point_line_draw_button.setText("")
                self.first_click_point = None

    def toggle_two_point_line_drawing(self):
        """선 그리기 모드 전환"""
        self.is_two_point_line_drawing = not self.is_two_point_line_drawing
        self.is_restricted_area_drawing = False
        self.is_two_point_drawing = False
        
        if self.is_two_point_line_drawing:
            self.two_point_line_draw_button.setStyleSheet("""
                QPushButton {
                    background-color: #FF0000;
                    border: 2px solid #FF0000;
                    border-radius: 6px;
                }
            """)
        else:
            self.two_point_line_draw_button.setStyleSheet("""
                QPushButton {
                    background-color: #2A2A2A;
                    border: 1px solid #444444;
                    border-radius: 6px;
                }
                QPushButton:hover {
                    background-color: #3A3A3A;
                    border: 1px solid #666666;
                }
                QPushButton:pressed {
                    background-color: #333333;
                }
            """)

    def toggle_three_point_line_drawing(self):
        """세 점 기준 선 그리기 모드 전환"""
        if not self.is_three_point_line_drawing:
            self.is_three_point_line_drawing = True
            self.is_restricted_area_drawing = False
            self.is_two_point_drawing = False
            self.is_two_point_line_drawing = False
            self.three_point_line_points = []
            self.clear_three_point_line_temp()
            
            # two_point_line_draw_button의 체크 해제 및 스타일 변경
            if self.two_point_line_draw_button:
                self.two_point_line_draw_button.setChecked(False)
                self.two_point_line_draw_button.setStyleSheet("""
                    QPushButton {
                        background-image: url(:/file/Registration/two_point_line.png);
                        background-repeat: no-repeat;
                        background-position: center;
                        background-color: #2A2A2A;
                        border: 1px solid #444444;
                        border-radius: 6px;
                    }
                    QPushButton:hover {
                        background-color: #3A3A3A;
                        border: 1px solid #666666;
                    }
                    QPushButton:checked {
                        background-color: #FF0000;
                    }
                """)
            
            # three_point_line_draw_button 체크 및 스타일 변경
            if self.three_point_line_draw_button:
                self.three_point_line_draw_button.setChecked(True)
                self.three_point_line_draw_button.setStyleSheet("""
                    QPushButton {
                        background-image: url(:/file/Registration/three_point_line.png);
                        background-repeat: no-repeat;
                        background-position: center;
                        background-color: #2A2A2A;
                        border: 1px solid #444444;
                        border-radius: 6px;
                    }
                    QPushButton:hover {
                        background-color: #3A3A3A;
                        border: 1px solid #666666;
                    }
                    QPushButton:checked {
                        background-color: #FF0000;
                    }
                """)
            
            if self.center_based_draw_button:
                self.center_based_draw_button.setText("")
            if self.two_point_draw_button:
                self.two_point_draw_button.setText("")

    def clear_three_point_line_temp(self):
        # 임시 원/선 삭제
        for c in getattr(self, 'three_point_line_circles', []):
            try:
                self.map_scene.removeItem(c)
            except Exception:
                pass
        for l in getattr(self, 'three_point_line_lines', []):
            try:
                self.map_scene.removeItem(l)
            except Exception:
                pass
        self.three_point_line_circles = []
        self.three_point_line_lines = []

    def create_rect(self, pos):
        """사각형 생성"""
        self.clear_current_rect()
        self.current_rect = QGraphicsRectItem()
        self.current_rect.setPen(QPen(QColor(255, 0, 0), 2))  # 빨간색 테두리
        self.current_rect.setBrush(QBrush(Qt.NoBrush))  # 투명한 내부
        self.map_scene.addItem(self.current_rect)
        self.update_rect(pos)

    def create_circle(self, center_pos):
        """원 생성"""
        self.clear_current_circle()
        
        # 맵 해상도에 따른 픽셀 크기 계산
        if hasattr(self, 'map_manager') and self.map_manager.map_info:
            resolution = self.map_manager.map_info.resolution
            # 선 그리기 모드일 때는 더 작은 원 사용
            if self.is_two_point_line_drawing:
                size = 0.2  # 선 그리기용 작은 원 크기
            else:
                size = self.circle_size if self.is_two_point_drawing else self.rect_size
            radius_pixels = (size / 2) / resolution
            
            # 원 생성
            from PySide6.QtWidgets import QGraphicsEllipseItem
            self.current_circle = QGraphicsEllipseItem(
                center_pos.x() - radius_pixels,
                center_pos.y() - radius_pixels,
                radius_pixels * 2,
                radius_pixels * 2
            )
            # 선 그리기 모드일 때는 파란색으로 고정
            if self.is_two_point_line_drawing:
                self.current_circle.setPen(QPen(QColor("#0307fc"), 2))
            else:
                self.current_circle.setPen(QPen(QColor(255, 0, 0), 2))
            self.current_circle.setBrush(QBrush(Qt.NoBrush))
            self.map_scene.addItem(self.current_circle)

    def update_rect(self, pos):
        """사각형 업데이트"""
        if self.current_rect and self.start_point:
            rect = QRectF(self.start_point, pos)
            self.current_rect.setRect(rect)

    def clear_current_rect(self):
        """현재 사각형 제거"""
        if self.current_rect:
            try:
                if self.current_rect.scene() == self.map_scene:
                    self.map_scene.removeItem(self.current_rect)
            except RuntimeError:
                print("사각형 제거 오류 무시: 이미 삭제됨")
            self.current_rect = None

    def clear_current_circle(self):
        """현재 원 제거"""
        if self.current_circle:
            try:
                if self.current_circle.scene() == self.map_scene:
                    self.map_scene.removeItem(self.current_circle)
            except RuntimeError:
                print("원 제거 오류 무시: 이미 삭제됨")
            self.current_circle = None

    def clear_current_line(self):
        """현재 선 제거"""
        if self.current_line:
            try:
                if self.current_line.scene() == self.map_scene:
                    self.map_scene.removeItem(self.current_line)
            except RuntimeError:
                print("선 제거 오류 무시: 이미 삭제됨")
            self.current_line = None

    def create_rect_from_two_points(self, point1, point2):
        """두 점을 이용한 사각형 생성"""
        rect = QRectF(point1, point2)
        self.current_rect = QGraphicsRectItem(rect)
        color = self.get_random_color()
        self.current_rect.setPen(QPen(color, 2))
        self.current_rect.setBrush(QBrush(Qt.NoBrush))
        self.map_scene.addItem(self.current_rect)
        print("두 점 기준 사각형이 생성되었습니다! 시작점:")
        self.add_area_button("두 점 기준 사각형", color, rect_item=self.current_rect)
        if self.two_point_draw_button:
            self.two_point_draw_button.setStyleSheet("""
                QPushButton {
                    background-image: url(:/file/Registration/two_point.png);
                    background-repeat: no-repeat;
                    background-position: center;
                    background-color: #FF0000;
                    border: 1px solid #444444;
                    border-radius: 6px;
                }
            """)
            self.two_point_draw_button.setText("")

    def add_area_button(self, area_type, color=None, rect_item=None, text_item=None):
        """영역 목록에 사각형 이모티콘 버튼 추가 및 사각형 중앙에 인덱스 표시"""
        button = QPushButton()
        button.setFixedSize(60, 60)
        button.setCheckable(True)
        index = len([b for b in self.area_buttons if hasattr(b, 'rect_item') and b.rect_item])
        # 색상 고정: 빨간색
        if color is None:
            color = QColor(255, 0, 0)
        button.setStyleSheet(f"""
            QPushButton {{
                background-color: #2D2D30;
                border: 2px solid {color.name()};
                border-radius: 10px;
                color: #FFFFFF;
                font-size: 40px;
            }}
            QPushButton:checked {{
                background-color: #FF0000;
            }}
        """)
        button.setCheckable(True)
        button.setText(str(index + 1))
        button.setProperty("area_type", area_type)
        button.setProperty("area_index", len(self.area_buttons))
        button.setProperty("color", color)
        button.clicked.connect(lambda: self.on_area_button_clicked(button))
        button.rect_item = rect_item  # 사각형 객체 저장
        if text_item is not None:
            button.text_item = text_item
        self.area_list_layout.addWidget(button)
        self.area_buttons.append(button)
        button.setToolTip(f"{area_type} #{len(self.area_buttons)}")
        # 사각형 중앙에 인덱스 표시
        if rect_item and text_item is None:
            rect = rect_item.rect()
            center = rect.center()
            from PySide6.QtWidgets import QGraphicsTextItem
            text_item = QGraphicsTextItem(str(index + 1))
            text_item.setDefaultTextColor(color)
            font = text_item.font()
            font.setPointSize(18)
            font.setBold(True)
            text_item.setFont(font)
            text_rect = text_item.boundingRect()
            text_item.setPos(center.x() - text_rect.width()/2, center.y() - text_rect.height()/2)
            self.map_scene.addItem(text_item)
            button.text_item = text_item  # 버튼에 텍스트 객체도 저장
        elif text_item is not None:
            button.text_item = text_item
        self._update_area_delete_button()
        
        return button  # 생성한 버튼 반환

    def _update_area_delete_button(self):
        """리스트에 버튼이 하나라도 있으면 삭제 버튼을 맨 오른쪽에 추가, 없으면 숨김. 모두 삭제 버튼도 동일하게 처리"""
        # 삭제 버튼이 이미 삭제된 경우 예외 방지
        if not hasattr(self, 'area_delete_button') or self.area_delete_button is None:
            return
        try:
            if self.area_list_layout.indexOf(self.area_delete_button) != -1:
                self.area_list_layout.removeWidget(self.area_delete_button)
        except RuntimeError:
            # 이미 삭제된 경우 무시
            return
        try:
            if self.area_list_layout.indexOf(self.area_delete_all_button) != -1:
                self.area_list_layout.removeWidget(self.area_delete_all_button)
        except Exception:
            pass
        if len(self.area_buttons) > 0:
            self.area_list_layout.addWidget(self.area_delete_button)
            self.area_delete_button.setVisible(True)
            self.area_list_layout.addWidget(self.area_delete_all_button)
            self.area_delete_all_button.setVisible(True)
        else:
            self.area_delete_button.setVisible(False)
            self.area_delete_all_button.setVisible(False)

    def on_area_button_clicked(self, button):
        """영역 버튼 클릭 시 단일 선택만 허용하고, 선택된 버튼을 저장"""
        for btn in self.area_buttons:
            btn.setChecked(False)
        button.setChecked(True)
        self.selected_area_button = button

    def get_selected_area_button(self):
        return getattr(self, 'selected_area_button', None)

    def on_restricted_area_changed(self, state):
        """제한 영역 체크박스 상태 변경 시 호출"""
        
        if state == Qt.Checked:
            # 제한 영역이 선택되면 허용 영역 체크 해제
            self.allowed_area_checkBox.blockSignals(True)  # 시그널 일시 차단
            self.allowed_area_checkBox.setChecked(False)
            self.allowed_area_checkBox.blockSignals(False)  # 시그널 차단 해제
        elif state == Qt.Unchecked:
            # 제한 영역이 해제되면 허용 영역 체크
            self.allowed_area_checkBox.blockSignals(True)  # 시그널 일시 차단
            self.allowed_area_checkBox.setChecked(True)
            self.allowed_area_checkBox.blockSignals(False)  # 시그널 차단 해제
    
    def on_allowed_area_changed(self, state):
        """허용 영역 체크박스 상태 변경 시 호출"""
        
        if state == Qt.Checked:
            # 허용 영역이 선택되면 제한 영역 체크 해제
            self.restricted_area_checkBox.blockSignals(True)  # 시그널 일시 차단
            self.restricted_area_checkBox.setChecked(False)
            self.restricted_area_checkBox.blockSignals(False)  # 시그널 차단 해제
        elif state == Qt.Unchecked:
            # 허용 영역이 해제되면 제한 영역 체크
            self.restricted_area_checkBox.blockSignals(True)  # 시그널 일시 차단
            self.restricted_area_checkBox.setChecked(True)
            self.restricted_area_checkBox.blockSignals(False)  # 시그널 차단 해제
        
    def on_apply_area(self):
        """적용 버튼 클릭 시 모든 금지 구역 데이터를 polygon 형태로 저장하고, 영역 설정 모드 종료 및 forbidden_areas.json 데이터 기반으로만 다시 그림"""
        print("\n===== on_apply_area 함수 시작 =====")
        forbidden_area_data = []
        red_idx = 1  # 금지 구역 인덱스
        blue_idx = 1  # 우선 경로 인덱스
        
        # 기존 금지 구역 데이터 초기화를 위해 빈 PolygonStamped 메시지 발행
        if hasattr(self, 'keepout_zone_publisher'):
            try:
                # 빈 메시지를 발행하여 기존 금지 구역 초기화 신호 전송
                self.keepout_zone_publisher.publish_keepout_zone(0, [])
                print("[keepout_zone] 금지 구역 초기화 메시지 발행 완료")
            except Exception as e:
                print(f"[keepout_zone] 금지 구역 초기화 중 오류 발생: {str(e)}")
        for btn in self.area_buttons:
            if hasattr(btn, 'is_area_button') and btn.is_area_button:
                # 영역 타입 확인
                area_type = btn.property('area_type')
                current_idx = red_idx if area_type == "금지 구역" else blue_idx
                print(f"\n버튼 {current_idx} 처리 시작 - 타입: {area_type}")
                # 체크 해제 후 기본 스타일 적용
                btn.setChecked(False)
                area_type = btn.property("area_type")
                if area_type == "금지 구역":
                    btn.setStyleSheet(
                        """
                        QPushButton {
                            background-color: transparent;
                            border: 2px solid #FF0000;
                            color: #FF0000;
                            font-size: 16px;
                            font-weight: bold;
                        }
                        QPushButton:checked {
                            background-color: rgba(255, 0, 0, 0.3);
                            color: #FF0000;
                        }
                        """
                    )
                elif area_type == "우선 경로":
                    btn.setStyleSheet(
                        """
                        QPushButton {
                            background-color: transparent;
                            border: 2px solid #0000FF;
                            color: #0000FF;
                            font-size: 16px;
                            font-weight: bold;
                        }
                        QPushButton:checked {
                            background-color: rgba(0, 0, 255, 0.3);
                            color: #0000FF;
                        }
                        """
                    )
                
                # 버튼의 현재 위치와 크기를 기반으로 맵 좌표 계산
                map_points = []
                print(f"버튼 객체 ID: {id(btn)}")
                
                # 버튼의 현재 위치와 크기 가져오기
                button_rect = btn.geometry()
                print(f"버튼 위치: ({button_rect.x()}, {button_rect.y()}), 크기: {button_rect.width()}x{button_rect.height()}")
                
                # 버튼의 네 꼭짓점 계산 (현재 크기 반영)
                # 주의: 여기서는 버튼의 좌표를 직접 씬 좌표로 취급합니다.
                # 버튼은 이미 씬 좌표계에 배치되어 있기 때문입니다.
                points = [
                    QPointF(button_rect.x(), button_rect.y()),  # 좌상단
                    QPointF(button_rect.x() + button_rect.width(), button_rect.y()),  # 우상단
                    QPointF(button_rect.x() + button_rect.width(), button_rect.y() + button_rect.height()),  # 우하단
                    QPointF(button_rect.x(), button_rect.y() + button_rect.height())  # 좌하단
                ]
                
                print("===== 금지 구역 버튼 꼭짓점 좌표 =====")
                print(f"버튼 크기: {button_rect.width()} x {button_rect.height()}")
                
                # 각 꼭짓점을 맵 좌표로 변환
                for i, point in enumerate(points):
                    # 버튼 좌표는 이미 씬 좌표계에 있으므로 mapToScene을 사용하지 않음
                    scene_pos = point  # 직접 씬 좌표로 사용
                    print(f"꼭짓점 {i+1} 씬 좌표: ({scene_pos.x():.2f}, {scene_pos.y():.2f})")
                    
                    # 맵 좌표로 변환
                    map_x, map_y, _ = self.convert_scene_to_map_coords(scene_pos)
                    print(f"꼭짓점 {i+1} 맵 좌표: ({map_x:.6f}, {map_y:.6f})")
                    
                    map_points.append([map_x, map_y])
                
                # 변환된 맵 좌표를 버튼에 저장 (다음 사용을 위해)
                btn.map_points = map_points
                print(f"버튼 객체에 맵 좌표 저장 완료: {map_points}")
                
                # 폴리곤 데이터 추가 (영역 타입 정보 포함)
                # 영역 타입에 따라 다른 인덱스 사용
                if area_type == "금지 구역":
                    current_idx = red_idx
                    red_idx += 1
                elif area_type == "우선 경로":
                    current_idx = blue_idx
                    blue_idx += 1
                else:
                    current_idx = 0
                    
                polygon_data = {
                    "index": current_idx,
                    "type": "polygon",
                    "area_type": area_type,  # 영역 타입(금지 구역 또는 우선 경로) 저장
                    "points": map_points
                }
                forbidden_area_data.append(polygon_data)
                print(f"{area_type} {current_idx}의 폴리곤 데이터 추가 완료")
                
                # 금지 구역인 경우 /keepout_zone 토픽으로 발행
                if area_type == "금지 구역" and hasattr(self, 'keepout_zone_publisher'):
                    try:
                        # 금지 구역 발행
                        self.keepout_zone_publisher.publish_keepout_zone(current_idx, map_points)
                        print(f"[keepout_zone] 금지 구역 {current_idx} 발행 완료: {len(map_points)}개 꼭짓점")
                    except Exception as e:
                        print(f"[keepout_zone] 금지 구역 발행 중 오류 발생: {str(e)}")
                
        # 전체 금지 구역 데이터 저장
        print(f"\n총 {len(forbidden_area_data)}개의 금지구역 데이터 저장")
        self.save_forbidden_areas_data(forbidden_area_data)
        
        # 영역 설정 모드 종료 및 forbidden_areas.json 데이터 기반으로만 다시 그림
        print("영역 설정 모드 종료")
        self.is_area_setting_screen = False
        self.update_ui_visibility()  # UI 요소 표시 상태 업데이트
        self.area_setting_changed.emit(self.is_area_setting_screen)
        #self.map_widget.setVisible(True)
        #self.map_area_control_widget.setVisible(False)
        #self.status_widget.setVisible(True)
        #self.area_list_container.setVisible(False)
        self.is_area_drawing_mode = False
        self.is_marker_enabled = True
        self.set_area_button.setChecked(False)
        
        # 일반 모드에서는 set_area_button 표시
        if hasattr(self, 'set_area_button') and self.set_area_button:
            self.set_area_button.show()
        
        # MapManager에 금지구역 정보 업데이트
        if hasattr(self, 'map_manager') and hasattr(self.map_manager, 'update_forbidden_areas'):
            self.map_manager.update_forbidden_areas(self.area_buttons)
            print(f"[on_apply_area] MapManager에 금지구역 정보 업데이트: {len(self.area_buttons)}개")
        
        #self.load_forbidden_areas()

    def on_cancel_area(self):
        """영역 취소 버튼 클릭 (저장된 forbidden_areas.json 데이터만 남기고 나머지 삭제)"""
        # forbidden_areas.json에서 저장된 꼭짓점 정보 읽기
        print("\n===== on_cancel_area 함수 시작 =====")
        self.is_area_setting_screen = False
        print(f"영역 모드 상태 : {self.is_area_setting_screen}")
        self.area_setting_changed.emit(self.is_area_setting_screen)  # 시그널 발생

        saved_corners_list = []
        try:
            data_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')
            file_path = os.path.join(data_dir, 'forbidden_areas.json')
            if os.path.exists(file_path):
                print(f"JSON 파일 존재: {file_path}")
                with open(file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    # 리스트 구조 지원
                    if isinstance(data, list):
                        area_list = data
                    elif isinstance(data, dict) and 'areas' in data:
                        area_list = data['areas']
                    else:
                        area_list = []
                    print(f"JSON에서 읽은 영역 개수: {len(area_list)}")
                    for area in area_list:
                        if not isinstance(area, dict):
                            print(f"잘못된 데이터 형식: {area}")
                            continue
                        points = area.get('points', [])
                        corners = tuple((round(pt[0], 4), round(pt[1], 4)) for pt in points)
                        saved_corners_list.append(corners)
                    print(f"저장된 꼭짓점 목록 개수: {len(saved_corners_list)}")
            else:
                print(f"JSON 파일이 존재하지 않음: {file_path}")
        except Exception as e:
            print(f"forbidden_areas.json 읽기 오류: {e}")
            import traceback
            traceback.print_exc()

        # 남길 버튼 리스트
        remain_buttons = []
        # 삭제할 버튼 리스트
        remove_buttons = []
        
        print(f"현재 버튼 개수: {len(self.area_buttons)}")
        for button in self.area_buttons:
            # 버튼에 저장된 맵 좌표 확인
            if hasattr(button, 'map_points') and button.map_points:
                print(f"버튼에 map_points 속성 있음: {button.map_points}")
                # map_points 좌표를 튜플로 변환하여 비교
                corners = tuple((round(pt[0], 4), round(pt[1], 4)) for pt in button.map_points)
                if corners in saved_corners_list:
                    print(f"JSON에 저장된 좌표와 일치하는 버튼 발견")
                    remain_buttons.append(button)
                else:
                    print(f"JSON에 저장되지 않은 버튼 발견")
                    remove_buttons.append(button)
            # 이전 방식으로 생성된 버튼 (map_points 속성 없음)
            elif hasattr(button, 'rect_item') and button.rect_item:
                print(f"기존 방식으로 생성된 버튼 처리 (rect_item 사용)")
                try:
                    rect = button.rect_item.rect()
                    topLeft = rect.topLeft()
                    topRight = rect.topRight()
                    bottomRight = rect.bottomRight()
                    bottomLeft = rect.bottomLeft()
                    # 씬 좌표 → 맵 좌표 변환
                    if hasattr(self, 'map_manager') and self.map_manager.map_info:
                        map_info = self.map_manager.map_info
                        resolution = map_info.resolution
                        origin_x = map_info.origin.position.x
                        origin_y = map_info.origin.position.y
                        scene_width = map_info.width
                        scene_corners = [topLeft, topRight, bottomRight, bottomLeft]
                        corners = tuple((
                            round((scene_width - pt.x()) * resolution + origin_x, 4),
                            round(pt.y() * resolution + origin_y, 4)
                        ) for pt in scene_corners)
                        
                        if corners in saved_corners_list:
                            print(f"JSON에 저장된 좌표와 일치하는 rect_item 버튼 발견")
                            remain_buttons.append(button)
                        else:
                            print(f"JSON에 저장되지 않은 rect_item 버튼 발견")
                            remove_buttons.append(button)
                except RuntimeError:
                    print("이미 삭제된 객체는 건너뜀")
                    remove_buttons.append(button)
            else:
                print(f"map_points와 rect_item이 모두 없는 버튼 발견 - 삭제 대상")
                remove_buttons.append(button)
                
        print(f"남길 버튼 개수: {len(remain_buttons)}, 삭제할 버튼 개수: {len(remove_buttons)}")
        
        # 삭제할 버튼들만 삭제
        for button in remove_buttons:
            if hasattr(button, 'rect_item') and button.rect_item:
                try:
                    if button.rect_item.scene() == self.map_scene:
                        self.map_scene.removeItem(button.rect_item)
                except RuntimeError:
                    print("사각형 제거 오류 무시: 이미 삭제됨")
            if hasattr(button, 'text_item') and button.text_item:
                try:
                    if button.text_item.scene() == self.map_scene:
                        self.map_scene.removeItem(button.text_item)
                except RuntimeError:
                    print("텍스트 제거 오류 무시: 이미 삭제됨")
            if hasattr(button, 'line_item') and button.line_item:
                try:
                    if button.line_item.scene() == self.map_scene:
                        self.map_scene.removeItem(button.line_item)
                except RuntimeError:
                    print("선 제거 오류 무시: 이미 삭제됨")
            if hasattr(button, 'line_item1') and button.line_item1:
                try:
                    if button.line_item1.scene() == self.map_scene:
                        self.map_scene.removeItem(button.line_item1)
                except RuntimeError:
                    print("선1 제거 오류 무시: 이미 삭제됨")
            if hasattr(button, 'line_item2') and button.line_item2:
                try:
                    if button.line_item2.scene() == self.map_scene:
                        self.map_scene.removeItem(button.line_item2)
                except RuntimeError:
                    print("선2 제거 오류 무시: 이미 삭제됨")
            # area_list_layout에서 버튼 제거
            if hasattr(self, 'area_list_layout'):
                self.area_list_layout.removeWidget(button)
            button.deleteLater()
        
        # area_buttons 리스트 갱신
        self.area_buttons = remain_buttons
        self._update_area_delete_button()
        # 인덱스 텍스트 업데이트 (영역 타입별로 구분하여 인덱스 부여)
        self.update_area_button_indices()
        print(f"남은 버튼 개수: {len(self.area_buttons)}")
        
        # 영역 설정 모드 종료
        self.is_area_setting_screen = False
        self.update_ui_visibility()
        #self.map_widget.setVisible(True)
        #self.map_area_control_widget.setVisible(False)
        # self.status_widget.setVisible(True)
        # self.area_list_container.setVisible(False)
        # self.set_area_button.setText("영역 설정")
        self.is_area_drawing_mode = False
        self.is_marker_enabled = True  # 마커 활성화
        self.set_area_button.setChecked(False)
        
        # set_area_button 다시 표시
        if hasattr(self, 'set_area_button') and self.set_area_button:
            self.set_area_button.show()
        
        # MapManager에 금지구역 정보 업데이트
        if hasattr(self, 'map_manager') and hasattr(self.map_manager, 'update_forbidden_areas'):
            self.map_manager.update_forbidden_areas(self.area_buttons)
            print(f"[on_cancel_area] MapManager에 금지구역 정보 업데이트: {len(self.area_buttons)}개")
            
        print("===== on_cancel_area 함수 종료 =====\n")
        
        # forbidden_areas.json 데이터 기반으로 다시 그리기
        #self.load_forbidden_areas()

    def save_forbidden_areas_data(self, areas_data):
        """금지구역 데이터를 JSON 파일에 저장"""
        try:
            # 금지구역 데이터 파일 경로
            data_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "data")
            os.makedirs(data_dir, exist_ok=True)
            file_path = os.path.join(data_dir, "forbidden_areas.json")
            
            # 데이터 구조 생성
            data_to_save = {"areas": areas_data}
            
            # 데이터 저장
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(data_to_save, f, indent=4)
            
            print(f"금지구역 데이터가 저장되었습니다: {file_path}")
            
            # 우선 경로 영역 데이터 발행 (데이터 변경 시 토픽 업데이트)
            if hasattr(self, 'preferred_area_publisher'):
                try:
                    # 우선 경로 영역 데이터 필터링
                    preferred_areas = []
                    for area in areas_data:
                        if area.get('area_type') == '우선 경로' or area.get('area_type') == '\uc6b0\uc120 \uacbd\ub85c':
                            preferred_areas.append({
                                'index': area.get('index'),
                                'points': area.get('points', [])
                            })
                    
                    # 각 우선 경로 영역 데이터 발행
                    for area in preferred_areas:
                        index = area.get('index')
                        points = area.get('points')
                        
                        if index is not None and points:
                            self.preferred_area_publisher.publish_preferred_area(index, points)
                            print(f"[save_forbidden_areas_data] 우선 경로 영역 {index} 발행 완료: {len(points)}개 꼭짓점")
                            
                except Exception as e:
                    print(f"[save_forbidden_areas_data] 우선 경로 영역 발행 중 오류 발생: {str(e)}")
            
        except Exception as e:
            print(f"금지구역 데이터 저장 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()

    def create_center_based_rect(self, center_pos):
        """중심점 기준 사각형 생성(QPushButton으로 대체)"""
        if hasattr(self, 'map_manager') and self.map_manager.map_info:
            resolution = self.map_manager.map_info.resolution
            half_size_pixels = (self.rect_size / 2) / resolution
            size_pixels = self.rect_size / resolution

            button = QPushButton(None)
            button.setFixedSize(int(size_pixels), int(size_pixels))
            button.setCheckable(True)
            color = self.get_random_color()
            button.setStyleSheet(f"""
                QPushButton {{
                    background-color: transparent;
                    border: 2px solid {color.name()};
                    border-radius: 10px;
                    color: #FFFFFF;
                    font-size: 24px;
                    font-weight: bold;
                }}
                QPushButton:checked {{
                    background-color: rgba(255, 0, 0, 0.3);
                }}
            """)
            button.move(int(center_pos.x() - half_size_pixels), int(center_pos.y() - half_size_pixels))
            button.is_area_button = True
            button.setProperty("area_type", "중심점 기준 사각형")
            button.setProperty("area_index", len(self.area_buttons))
            button.setProperty("color", color)
            button.clicked.connect(lambda: self.on_area_button_clicked(button))
            self.map_view.scene().addWidget(button)
            self.area_buttons.append(button)
            button.setToolTip(f"중심점 기준 사각형 #{len(self.area_buttons)}")
            self._update_area_delete_button()
            self.update_area_button_indices()

    def create_line_from_two_points(self, point1, point2):
        """두 점을 이용한 선 생성"""
        from PySide6.QtWidgets import QGraphicsLineItem
        from PySide6.QtCore import QLineF
        
        # 두 점을 이용하여 선 생성
        line = QLineF(point1, point2)
        self.current_line = QGraphicsLineItem(line)
        # 선 색상을 #0307fc로 고정
        self.current_line.setPen(QPen(QColor("#0307fc"), 2))
        self.map_scene.addItem(self.current_line)
        
        # 선 생성 후 영역 목록에 버튼 추가 (선 객체 전달)
        self.add_line_button("두 점 기준 선", QColor("#0307fc"), line_item=self.current_line)

    def add_line_button(self, line_type, color, line_item=None):
        """영역 목록에 선 이모티콘 버튼 추가"""
        button = QPushButton()
        button.setFixedSize(60, 60)
        button.setCheckable(True)
        button.setStyleSheet(f"""
            QPushButton {{
                background-color: #2D2D30;
                border: 2px solid {color.name()};
                border-radius: 10px;
                color: {color.name()};
                font-size: 40px;
            }}
            QPushButton:checked {{
                background-color: #FF0000;
            }}
        """)
        button.setCheckable(True)
        button.setText("━")  # 선 이모티콘
        button.setProperty("line_type", line_type)
        button.setProperty("line_index", len(self.area_buttons))
        button.setProperty("color", color)
        button.clicked.connect(lambda: self.on_area_button_clicked(button))
        button.line_item = line_item  # 선 객체 저장
        self.area_list_layout.addWidget(button)
        self.area_buttons.append(button)
        button.setToolTip(f"{line_type} #{len(self.area_buttons)}")
        self._update_area_delete_button()

    def on_area_delete_button_clicked(self):
        selected_button = self.get_selected_area_button()
        if not selected_button:
            return
            
        # 금지 구역 인덱스 저장 (삭제 메시지 발행용)
        area_index = -1
        area_type = ""
        if hasattr(selected_button, 'property'):
            area_index = selected_button.property('area_index')
            area_type = selected_button.property('area_type')
            
        if hasattr(selected_button, 'rect_item') and selected_button.rect_item:
            try:
                if selected_button.rect_item.scene() == self.map_scene:
                    self.map_scene.removeItem(selected_button.rect_item)
            except RuntimeError:
                print("사각형 제거 오류 무시: 이미 삭제됨")
        if hasattr(selected_button, 'text_item') and selected_button.text_item:
            try:
                if selected_button.text_item.scene() == self.map_scene:
                    self.map_scene.removeItem(selected_button.text_item)
            except RuntimeError:
                print("텍스트 제거 오류 무시: 이미 삭제됨")
        if hasattr(selected_button, 'line_item') and selected_button.line_item:
            try:
                if selected_button.line_item.scene() == self.map_scene:
                    self.map_scene.removeItem(selected_button.line_item)
            except RuntimeError:
                print("선 제거 오류 무시: 이미 삭제됨")
        if hasattr(selected_button, 'line_item1') and selected_button.line_item1:
            try:
                if selected_button.line_item1.scene() == self.map_scene:
                    self.map_scene.removeItem(selected_button.line_item1)
            except RuntimeError:
                print("선1 제거 오류 무시: 이미 삭제됨")
        if hasattr(selected_button, 'line_item2') and selected_button.line_item2:
            try:
                if selected_button.line_item2.scene() == self.map_scene:
                    self.map_scene.removeItem(selected_button.line_item2)
            except RuntimeError:
                print("선2 제거 오류 무시: 이미 삭제됨")
        self.area_list_layout.removeWidget(selected_button)
        if selected_button in self.area_buttons:
            self.area_buttons.remove(selected_button)
            
        # 금지 구역인 경우 삭제 메시지 발행
        if area_type == "금지 구역" and area_index is not None and area_index >= 0:
            if hasattr(self, 'delete_keepout_zone_publisher'):
                try:
                    # 금지 구역 삭제 메시지 발행
                    self.delete_keepout_zone_publisher.publish_delete_keepout_zone(area_index)
                    print(f"[delete_keepout_zone] 금지 구역 {area_index} 삭제 메시지 발행 완료")
                except Exception as e:
                    print(f"[delete_keepout_zone] 금지 구역 삭제 메시지 발행 중 오류 발생: {str(e)}")
                    
        selected_button.deleteLater()
        self._update_area_delete_button()
        self.update_area_button_indices()
        
        # MapManager에 금지구역 정보 업데이트
        if hasattr(self, 'map_manager') and hasattr(self.map_manager, 'update_forbidden_areas'):
            self.map_manager.update_forbidden_areas(self.area_buttons)
            print(f"[on_area_delete_button_clicked] MapManager에 금지구역 정보 업데이트: {len(self.area_buttons)}개")

    def _update_rect_index_texts(self):
        """남아있는 사각형들의 인덱스 텍스트와 리스트 버튼 텍스트를 1,2,3...으로 재정렬"""
        idx = 1
        for btn in self.area_buttons:
            if hasattr(btn, 'rect_item') and btn.rect_item and hasattr(btn, 'text_item') and btn.text_item:
                rect = btn.rect_item.rect()
                center = rect.center()
                # 텍스트 항목은 비표시 처리
                btn.text_item.setVisible(False)
                text_rect = btn.text_item.boundingRect()
                btn.text_item.setPos(center.x() - text_rect.width()/2, center.y() - text_rect.height()/2)
            # 인덱스는 내부적으로만 저장
            btn.setProperty("internal_index", idx)
            idx += 1

    def move_selected_rect(self, dx, dy):
        btn = self.get_selected_area_button()
        if not btn:
            return
        # 사각형 이동
        if hasattr(btn, 'rect_item') and btn.rect_item:
            rect_item = btn.rect_item
            rect = rect_item.rect()
            rect.translate(dx, dy)
            rect_item.setRect(rect)
            # 텍스트도 같이 이동
            if hasattr(btn, 'text_item') and btn.text_item:
                center = rect.center()
                text_item = btn.text_item
                text_rect = text_item.boundingRect()
                text_item.setPos(center.x() - text_rect.width()/2, center.y() - text_rect.height()/2)
        # 선 이동 (두 점 선)
        if hasattr(btn, 'line_item') and btn.line_item:
            line_item = btn.line_item
            line = line_item.line()
            line.translate(dx, dy)
            line_item.setLine(line)
        # 세 점 선 이동 (line_item1, line_item2)
        if hasattr(btn, 'line_item1') and hasattr(btn, 'line_item2'):
            for line_item in [btn.line_item1, btn.line_item2]:
                if line_item:
                    line = line_item.line()
                    line.translate(dx, dy)
                    line_item.setLine(line)
        # 인덱스 텍스트 업데이트 (영역 타입별로 구분하여 인덱스 부여)
        self.update_area_button_indices()

    def rotate_selected_rect(self):
        btn = self.get_selected_area_button()
        if not btn:
            return
        # 사각형 회전
        if hasattr(btn, 'rect_item') and btn.rect_item:
            rect_item = btn.rect_item
            current_angle = rect_item.rotation() if hasattr(rect_item, 'rotation') else 0
            rect_item.setRotation(current_angle + 10)  # 10도씩 회전
            # 텍스트도 같이 회전 및 위치 갱신
            if hasattr(btn, 'text_item') and btn.text_item:
                rect = rect_item.rect()
                center = rect.center()
                text_item = btn.text_item
                text_rect = text_item.boundingRect()
                text_item.setPos(center.x() - text_rect.width()/2, center.y() - text_rect.height()/2)
                text_item.setRotation(current_angle + 10)
        # 선 회전 (두 점 선)
        if hasattr(btn, 'line_item') and btn.line_item:
            line_item = btn.line_item
            line = line_item.line()
            center = line.center()
            angle = 10  # 10도 회전
            line_item.setRotation(line_item.rotation() + angle)
        # 세 점 선 회전 (line_item1, line_item2)
        if hasattr(btn, 'line_item1') and hasattr(btn, 'line_item2'):
            for line_item in [btn.line_item1, btn.line_item2]:
                if line_item:
                    line_item.setRotation(line_item.rotation() + 10)

    def scale_selected_rect(self, scale_factor):
        btn = self.get_selected_area_button()
        if not btn:
            return
        # 사각형 확대/축소
        if hasattr(btn, 'rect_item') and btn.rect_item:
            rect_item = btn.rect_item
            rect = rect_item.rect()
            center = rect.center()
            new_width = rect.width() * scale_factor
            new_height = rect.height() * scale_factor
            new_rect = QRectF(
                center.x() - new_width / 2,
                center.y() - new_height / 2,
                new_width,
                new_height
            )
            rect_item.setRect(new_rect)
            # 텍스트도 같이 이동
            if hasattr(btn, 'text_item') and btn.text_item:
                text_item = btn.text_item
                text_rect = text_item.boundingRect()
                text_item.setPos(center.x() - text_rect.width()/2, center.y() - text_rect.height()/2)
        # 선 확대/축소 (두 점 선)
        if hasattr(btn, 'line_item') and btn.line_item:
            line_item = btn.line_item
            line = line_item.line()
            center = line.center()
            new_length = line.length() * scale_factor
            angle = line.angle()
            new_line = QLineF()
            new_line.setP1(center)
            new_line.setLength(new_length)
            new_line.setAngle(angle)
            line_item.setLine(new_line)
        # 세 점 선 확대/축소 (line_item1, line_item2)
        if hasattr(btn, 'line_item1') and hasattr(btn, 'line_item2'):
            for line_item in [btn.line_item1, btn.line_item2]:
                if line_item:
                    line = line_item.line()
                    center = line.center()
                    new_length = line.length() * scale_factor
                    angle = line.angle()
                    new_line = QLineF()
                    new_line.setP1(center)
                    new_line.setLength(new_length)
                    new_line.setAngle(angle)
                    line_item.setLine(new_line)

    def start_object_move_timer(self, direction):
        """객체 이동 타이머 시작"""
        self.object_move_action = direction
        self.object_move_timer.start()

    def stop_object_move_timer(self):
        """객체 이동 타이머 정지"""
        self.object_move_timer.stop()
        self.object_move_action = None

    def _on_object_move_timer(self):
        """타이머 이벤트 처리"""
        if self.object_move_action:
            if self.object_move_action == 'up':
                self.move_selected_button_up()
            elif self.object_move_action == 'down':
                self.move_selected_button_down()
            elif self.object_move_action == 'left':
                self.move_selected_button_left()
            elif self.object_move_action == 'right':
                self.move_selected_button_right()

    def setup_type_select_button(self):
        """type_select_button 초기 설정"""
        if self.type_select_button:
            # 초기 이미지 설정
            self.update_type_button_image()
            # 클릭 이벤트 연결
            self.type_select_button.clicked.connect(self.toggle_type_image)

    def toggle_type_image(self):
        """type_select_button 클릭 시 이미지 전환 및 위젯 전환"""
        if self.type_select_button:
            # 다음 이미지 인덱스 계산
            self.current_type_index = (self.current_type_index + 1) % len(self.type_images)
            # 이미지 업데이트
            self.update_type_button_image()
            # 위젯 전환
            if self.restricted_area_widget and self.priority_path_area_widget:
                if self.current_type_index == 0:  # 금지 영역
                    self.restricted_area_widget.setVisible(True)
                    self.priority_path_area_widget.setVisible(False)
                    # 금지 영역 모드로 전환 시 center_based_draw_button 활성화
                    self.is_restricted_area_drawing = True
                    self.is_two_point_drawing = False
                    self.is_two_point_line_drawing = False
                    self.is_three_point_line_drawing = False
                    
                    # center_based_draw_button의 배경을 빨간색으로 설정
                    if self.center_based_draw_button:
                        self.center_based_draw_button.setStyleSheet("""
                            QPushButton {
                                background-image: url(:/file/Registration/center_based_draw.png);
                                background-repeat: no-repeat;
                                background-position: center;
                                background-color: #FF0000;
                                border: 1px solid #444444;
                                border-radius: 6px;
                            }
                        """)
                    
                    # two_point_draw_button의 배경을 #2A2A2A로 설정
                    if self.two_point_draw_button:
                        self.two_point_draw_button.setStyleSheet("""
                            QPushButton {
                                background-image: url(:/file/Registration/two_point.png);
                                background-repeat: no-repeat;
                                background-position: center;
                                background-color: #2A2A2A;
                                border: 1px solid #444444;
                                border-radius: 6px;
                            }
                        """)
                else:  # 우선 영역
                    self.restricted_area_widget.setVisible(False)
                    self.priority_path_area_widget.setVisible(True)
                    # 우선 영역 모드로 전환 시 two_point_line_draw_button 활성화
                    self.is_restricted_area_drawing = False
                    self.is_two_point_drawing = False
                    self.is_two_point_line_drawing = True
                    self.is_three_point_line_drawing = False
                    
                    # two_point_line_draw_button의 배경을 빨간색으로 설정
                    if self.two_point_line_draw_button:
                        self.two_point_line_draw_button.setStyleSheet("""
                            QPushButton {
                                background-image: url(:/file/Registration/two_point_line.png);
                                background-repeat: no-repeat;
                                background-position: center;
                                background-color: #FF0000;
                                border: 1px solid #444444;
                                border-radius: 6px;
                            }
                        """)
                    
                    # three_point_line_draw_button의 배경을 #2A2A2A로 설정
                    if self.three_point_line_draw_button:
                        self.three_point_line_draw_button.setStyleSheet("""
                            QPushButton {
                                background-image: url(:/file/Registration/three_point_line.png);
                                background-repeat: no-repeat;
                                background-position: center;
                                background-color: #2A2A2A;
                                border: 1px solid #444444;
                                border-radius: 6px;
                            }
                        """)

    def update_type_button_image(self):
        """type_select_button의 배경 이미지 업데이트"""
        if self.type_select_button:
            current_image = self.type_images[self.current_type_index]
            self.type_select_button.setStyleSheet(f"""
                QPushButton {{
                    background-image: url({current_image});
                    background-repeat: no-repeat;
                    background-position: center;
                }}
            """)
        
    def setup_area_widgets(self):
        """영역 설정 위젯 초기 설정"""
        pass

    def create_temp_circle(self, center_pos):
        # 임시 원 생성 (세 점 선 그리기용, 파란색)
        if hasattr(self, 'map_manager') and self.map_manager.map_info:
            resolution = self.map_manager.map_info.resolution
            size = 0.2
            radius_pixels = (size / 2) / resolution
            from PySide6.QtWidgets import QGraphicsEllipseItem
            circle = QGraphicsEllipseItem(
                center_pos.x() - radius_pixels,
                center_pos.y() - radius_pixels,
                radius_pixels * 2,
                radius_pixels * 2
            )
            circle.setPen(QPen(QColor("#0307fc"), 2))
            circle.setBrush(QBrush(Qt.NoBrush))
            self.map_scene.addItem(circle)
            return circle
        return None

    def create_temp_line(self, p1, p2):
        # 임시 선 생성 (세 점 선 그리기용, 파란색)
        from PySide6.QtWidgets import QGraphicsLineItem
        from PySide6.QtCore import QLineF
        line = QGraphicsLineItem(QLineF(p1, p2))
        line.setPen(QPen(QColor("#0307fc"), 2))
        self.map_scene.addItem(line)
        return line

    def add_three_point_line_button(self, line_type, color, line_item1, line_item2):
        """세 점 선 쌍을 area_list에 버튼으로 추가"""
        button = QPushButton()
        button.setFixedSize(60, 60)
        button.setCheckable(True)
        button.setStyleSheet(f"""
            QPushButton {{
                background-color: #2D2D30;
                border: 2px solid {color.name()};
                border-radius: 10px;
                color: {color.name()};
                font-size: 40px;
            }}
            QPushButton:checked {{
                background-color: #FF0000;
            }}
        """)
        button.setCheckable(True)
        button.setText("L")  # 세 점 선 이모티콘(혹은 원하는 문자)
        button.setProperty("line_type", line_type)
        button.setProperty("line_index", len(self.area_buttons))
        button.setProperty("color", color)
        button.clicked.connect(lambda: self.on_area_button_clicked(button))
        button.line_item1 = line_item1  # 첫 번째 선 객체
        button.line_item2 = line_item2  # 두 번째 선 객체
        self.area_list_layout.addWidget(button)
        self.area_buttons.append(button)
        button.setToolTip(f"{line_type} #{len(self.area_buttons)}")
        self._update_area_delete_button()

    def on_area_delete_all_button_clicked(self):
        """모두 삭제 버튼 클릭 시 맵 위의 모든 금지구역 QPushButton 삭제"""
        # 금지 구역 삭제 메시지 발행 (모든 금지 구역 삭제를 의미하는 특수 인덱스 -1 사용)
        if hasattr(self, 'delete_keepout_zone_publisher'):
            try:
                # -1은 모든 금지 구역 삭제를 의미
                self.delete_keepout_zone_publisher.publish_delete_keepout_zone(-1)
                print("[delete_keepout_zone] 모든 금지 구역 삭제 메시지 발행 완료")
            except Exception as e:
                print(f"[delete_keepout_zone] 모든 금지 구역 삭제 메시지 발행 중 오류 발생: {str(e)}")
                
        # area_buttons에서 금지구역 QPushButton만 삭제
        remove_buttons = [btn for btn in self.area_buttons if isinstance(btn, QPushButton) and getattr(btn, 'is_area_button', False)]
        for button in remove_buttons:
            # 맵에서 제거
            proxy = button.parentWidget()
            if proxy:
                proxy.deleteLater()
            button.deleteLater()
            if button in self.area_buttons:
                self.area_buttons.remove(button)
        self._update_area_delete_button()
        self.update_area_button_indices()
        
        # MapManager에 금지구역 정보 업데이트
        if hasattr(self, 'map_manager') and hasattr(self.map_manager, 'update_forbidden_areas'):
            self.map_manager.update_forbidden_areas(self.area_buttons)
            print(f"[on_area_delete_all_button_clicked] MapManager에 금지구역 정보 업데이트: {len(self.area_buttons)}개")

    def rotate_rect(self, angle):
        """사각형 회전"""
        if self.current_rect:
            # 현재 사각형의 중심점 계산
            rect = self.current_rect.rect()
            center = rect.center()
            
            # 현재 회전 각도 가져오기
            current_rotation = self.current_rect.rotation()
            
            # 새로운 회전 각도 계산
            new_rotation = current_rotation + angle
            
            # 회전 중심점을 사각형의 중심으로 설정하고 회전
            self.current_rect.setTransformOriginPoint(center)
            self.current_rect.setRotation(new_rotation)

    def grow_selected_rect_vertical(self):
        btn = self.get_selected_area_button()
        if not btn:
            return
        if hasattr(btn, 'rect_item') and btn.rect_item:
            rect_item = btn.rect_item
            rect = rect_item.rect()
            center = rect.center()
            new_height = rect.height() * 1.1
            # 최소 크기 제한 (10픽셀)
            if new_height < 10:
                new_height = 10
            new_rect = QRectF(
                rect.left(),
                center.y() - new_height / 2,
                rect.width(),
                new_height
            )
            rect_item.setRect(new_rect)
            # 텍스트도 같이 이동
            if hasattr(btn, 'text_item') and btn.text_item:
                text_item = btn.text_item
                text_rect = text_item.boundingRect()
                text_item.setPos(center.x() - text_rect.width()/2, center.y() - text_rect.height()/2)

    def shrink_selected_rect_vertical(self):
        btn = self.get_selected_area_button()
        if not btn:
            return
        if hasattr(btn, 'rect_item') and btn.rect_item:
            rect_item = btn.rect_item
            rect = rect_item.rect()
            center = rect.center()
            new_height = rect.height() * 0.9
            # 최소 크기 제한 (10픽셀)
            if new_height < 10:
                new_height = 10
            new_rect = QRectF(
                rect.left(),
                center.y() - new_height / 2,
                rect.width(),
                new_height
            )
            rect_item.setRect(new_rect)
            # 텍스트도 같이 이동
            if hasattr(btn, 'text_item') and btn.text_item:
                text_item = btn.text_item
                text_rect = text_item.boundingRect()
                text_item.setPos(center.x() - text_rect.width()/2, center.y() - text_rect.height()/2)

    def grow_selected_rect_horizontal(self):
        btn = self.get_selected_area_button()
        if not btn:
            return
        if hasattr(btn, 'rect_item') and btn.rect_item:
            rect_item = btn.rect_item
            rect = rect_item.rect()
            center = rect.center()
            new_width = rect.width() * 1.1
            # 최소 크기 제한 (10픽셀)
            if new_width < 10:
                new_width = 10
            new_rect = QRectF(
                center.x() - new_width / 2,
                rect.top(),
                new_width,
                rect.height()
            )
            rect_item.setRect(new_rect)
            # 텍스트도 같이 이동
            if hasattr(btn, 'text_item') and btn.text_item:
                text_item = btn.text_item
                text_rect = text_item.boundingRect()
                text_item.setPos(center.x() - text_rect.width()/2, center.y() - text_rect.height()/2)

    def shrink_selected_rect_horizontal(self):
        btn = self.get_selected_area_button()
        if not btn:
            return
        if hasattr(btn, 'rect_item') and btn.rect_item:
            rect_item = btn.rect_item
            rect = rect_item.rect()
            center = rect.center()
            new_width = rect.width() * 0.9
            # 최소 크기 제한 (10픽셀)
            if new_width < 10:
                new_width = 10
            new_rect = QRectF(
                center.x() - new_width / 2,
                rect.top(),
                new_width,
                rect.height()
            )
            rect_item.setRect(new_rect)
            # 텍스트도 같이 이동
            if hasattr(btn, 'text_item') and btn.text_item:
                text_item = btn.text_item
                text_rect = text_item.boundingRect()
                text_item.setPos(center.x() - text_rect.width()/2, center.y() - text_rect.height()/2)

    def on_control_type_select_changed(self, state):
        # 영역 제어 모드일 때
        if self.control_type_select_checkbox.isChecked():
            # 수직/수평 크기 조절 버튼 표시
            if self.grow_vertical_button:
                self.grow_vertical_button.setVisible(True)
            if self.shrink_vertical_button:
                self.shrink_vertical_button.setVisible(True)
            if self.grow_horizontal_button:
                self.grow_horizontal_button.setVisible(True)
            if self.shrink_horizontal_button:
                self.shrink_horizontal_button.setVisible(True)
            # 줌 버튼 숨김
            if self.object_zoom_in:
                self.object_zoom_in.setVisible(False)
            if self.object_zoom_out:
                self.object_zoom_out.setVisible(False)
            # 체크박스 텍스트 변경
            self.control_type_select_checkbox.setText("영역 제어")
        # 맵 제어 모드일 때
        else:
            # 수직/수평 크기 조절 버튼 숨김
            if self.grow_vertical_button:
                self.grow_vertical_button.setVisible(False)
            if self.shrink_vertical_button:
                self.shrink_vertical_button.setVisible(False)
            if self.grow_horizontal_button:
                self.grow_horizontal_button.setVisible(False)
            if self.shrink_horizontal_button:
                self.shrink_horizontal_button.setVisible(False)
            # 줌 버튼 표시
            if self.object_zoom_in:
                self.object_zoom_in.setVisible(True)
            if self.object_zoom_out:
                self.object_zoom_out.setVisible(True)
            # 체크박스 텍스트 변경
            self.control_type_select_checkbox.setText("맵 제어")
        
        # 이동 버튼들은 항상 표시
        if self.object_move_up:
            self.object_move_up.setVisible(True)
        if self.object_move_down:
            self.object_move_down.setVisible(True)
        if self.object_move_left:
            self.object_move_left.setVisible(True)
        if self.object_move_right:
            self.object_move_right.setVisible(True)

    def zoom_map(self, factor):
        """맵 확대/축소"""
        if self.map_view:
            current_scale = self.map_view.transform().m11()  # 현재 스케일 가져오기
            new_scale = current_scale * factor
            
            # 스케일 제한 (선택적)
            min_scale = 0.1
            max_scale = 10.0
            new_scale = max(min_scale, min(max_scale, new_scale))
            
            # 맵 뷰의 중심점 기준으로 확대/축소
            center = self.map_view.viewport().rect().center()
            self.map_view.setTransform(QTransform().scale(new_scale, new_scale))
            self.map_view.centerOn(self.map_view.mapToScene(center))

    def create_forbidden_area_button(self, x, y):
        # 이미 비슷한 위치에 버튼이 있으면 새로 생성하지 않고 해당 버튼을 checked로!
        print("\n===== create_forbidden_area_button 함수 시작 =====")
        print(f"입력 좌표: x={x}, y={y}")
        
        # 맵 이미지 영역을 벗어나는지 확인
        if hasattr(self, 'map_manager') and self.map_manager and self.map_manager.map_overlay_pixmap:
            map_width = self.map_manager.map_overlay_pixmap.width()
            map_height = self.map_manager.map_overlay_pixmap.height()
            
            # 버튼 반경(30)을 고려하여 맵 영역을 벗어나는지 확인
            if x - 30 < 0 or y - 30 < 0 or x + 30 > map_width or y + 30 > map_height:
                print("맵 이미지 영역을 벗어남")
                ToastManager.instance().show_toast("맵 영역을 벗어나 생성할 수 없습니다.", self)
                return None
        
        for btn in self.area_buttons:
            if hasattr(btn, 'is_area_button') and btn.is_area_button:
                bx, by = btn.pos().x(), btn.pos().y()
                bw, bh = btn.width(), btn.height()
                btn_cx = bx + bw / 2
                btn_cy = by + bh / 2
                if abs(btn_cx - x) < 40 and abs(btn_cy - y) < 40:
                    print(f"기존 버튼 발견: 위치=({btn_cx}, {btn_cy}), 거리={abs(btn_cx - x)}, {abs(btn_cy - y)}")
                    btn.setChecked(True)
                    self.on_area_button_clicked(btn)
                    print("===== create_forbidden_area_button 함수 종료 (기존 버튼 사용) =====\n")
                    return btn  # 기존 버튼 반환

        # 우선 경로 영역 생성 시 색상 유효성 검사
        if self.priority_path_area_button.isChecked():
            # 60x60 영역 내의 모든 픽셀 색상 검사
            is_valid = self.check_area_color_validity(x, y, 30)  # 반경 30 (60x60 버튼)
            if not is_valid:
                print("우선 경로 영역 생성 실패: 흰색(255,255,255) 외의 다른 색상이 포함된 영역입니다.")
                ToastManager.instance().show_toast("흰색 영역에만 우선 경로를 설정할 수 있습니다.", self)
                return None

        print("새 버튼 생성")
        button = QPushButton(None)  # 부모를 None으로 설정
        button.setCheckable(True)
        # 금지구역/우선경로에 따라 스타일 분기
        if self.restricted_area_button.isChecked():
            # 금지구역(빨간색)
            button.setStyleSheet(
                """
                QPushButton {
                    background-color: transparent;
                    border: 2px solid #FF0000;
                    color: #FF0000;
                    font-size: 40px;
                    font-weight: bold;
                }
                QPushButton:checked {
                    background-color: rgba(255, 0, 0, 0.3);
                    color: #FF0000;
                }
                """
            )
            button.setProperty("area_type", "금지 구역")
        elif self.priority_path_area_button.isChecked():
            # 우선경로(파란색)
            button.setStyleSheet(
                """
                QPushButton {
                    background-color: transparent;
                    border: 2px solid #0000FF;
                    color: #0000FF;
                    font-size: 40px;
                    font-weight: bold;
                }
                QPushButton:checked {
                    background-color: rgba(0, 0, 255, 0.3);
                    color: #0000FF;
                }
                """
            )
            button.setProperty("area_type", "우선 경로")
        else:
            # 기본값(금지구역)
            button.setStyleSheet(
                """
                QPushButton {
                    background-color: transparent;
                    border: 2px solid #FF0000;
                    color: #FF0000;
                    font-size: 40px;
                    font-weight: bold;
                }
                QPushButton:checked {
                    background-color: rgba(255, 0, 0, 0.3);
                    color: #FF0000;
                }
                """
            )
            button.setProperty("area_type", "금지 구역")
        button.setGeometry(x - 30, y - 30, 60, 60)
        button.is_area_button = True
        button.setProperty("area_index", len(self.area_buttons))
        button.clicked.connect(lambda: self.on_area_button_clicked(button))
        # 버튼 텍스트를 비워서 인덱스를 표시하지 않음
        button.setText("")
        self.map_view.scene().addWidget(button)
        self.area_buttons.append(button)
        self._update_area_delete_button()
        self.update_area_button_indices()
        button.setChecked(True)
        self.on_area_button_clicked(button)
        
        # 버튼의 4개 꼭짓점 좌표 계산 및 출력
        print("\n===== 금지 구역 버튼 꼭짓점 좌표 =====")
        button_pos = button.pos()
        button_size = button.size()
        
        # 버튼의 씬 좌표 직접 사용 (버튼은 이미 씬에 추가되었으므로)
        scene_x, scene_y = x, y  # 버튼 중심 좌표
        
        # 버튼의 네 꼭짓점 좌표 계산 (씬 좌표)
        half_width = button_size.width() / 2
        half_height = button_size.height() / 2
        points = [
            QPointF(scene_x - half_width, scene_y - half_height),  # 좌상단
            QPointF(scene_x + half_width, scene_y - half_height),  # 우상단
            QPointF(scene_x + half_width, scene_y + half_height),  # 우하단
            QPointF(scene_x - half_width, scene_y + half_height)   # 좌하단
        ]
        
        print("버튼 크기:", button_size.width(), "x", button_size.height())
        print(f"버튼 중심 씬 좌표: ({scene_x}, {scene_y})")
        
        # 맵 좌표 저장
        map_points = []
        for i, point in enumerate(points):
            # 씬 좌표를 맵 좌표로 변환
            map_x, map_y, _ = self.convert_scene_to_map_coords(point)
            
            corner_name = ["좌상단", "우상단", "우하단", "좌하단"][i]
            print(f"꼭짓점 {i+1} ({corner_name}) - 씬 좌표: ({point.x():.2f}, {point.y():.2f}), 맵 좌표: ({map_x:.6f}, {map_y:.6f})")
            
            # 맵 좌표 저장
            map_points.append([map_x, map_y])
        
        # 버튼 객체에 맵 좌표 저장
        button.map_points = map_points
        print(f"버튼 객체에 맵 좌표 저장 완료: {map_points}")
        print(f"버튼 객체 ID: {id(button)}")
        print(f"버튼 객체에 map_points 속성 확인: {hasattr(button, 'map_points')}")
        if hasattr(button, 'map_points'):
            print(f"저장된 map_points 데이터: {button.map_points}")
        
        # 디버깅을 위해 중심점도 출력
        center_map_x, center_map_y, _ = self.convert_scene_to_map_coords(QPointF(scene_x, scene_y))
        print(f"중심점 - 씬 좌표: ({scene_x:.2f}, {scene_y:.2f}), 맵 좌표: ({center_map_x:.6f}, {center_map_y:.6f})")
        print("=====================================\n")
        
        return button

    def update_area_button_indices(self):
        print("Updating area button indices")
        red_idx = 1  # 금지구역 인덱스
        blue_idx = 1  # 우선경로 인덱스
        for btn in self.area_buttons:
            if isinstance(btn, QPushButton) and getattr(btn, 'is_area_button', False):
                area_type = btn.property("area_type")
                if area_type == "금지 구역":
                    # 인덱스는 내부적으로만 저장하고 UI에는 표시하지 않음
                    btn.setProperty("internal_index", red_idx)
                    # 텍스트 항목도 비표시
                    if hasattr(btn, 'text_item') and btn.text_item:
                        btn.text_item.setVisible(False)
                    btn.setStyleSheet(
                        """
                        QPushButton {
                            background-color: transparent;
                            border: 2px solid #FF0000;
                            font-size: 16px;
                            font-weight: bold;
                            color: #FF0000;
                        }
                        QPushButton:checked {
                            background-color: rgba(255, 0, 0, 0.3);
                            color: #FF0000;
                        }
                        """
                    )
                    red_idx += 1
                elif area_type == "우선 경로":
                    # 인덱스는 내부적으로만 저장하고 UI에는 표시하지 않음
                    btn.setProperty("internal_index", blue_idx)
                    # 텍스트 항목도 비표시
                    if hasattr(btn, 'text_item') and btn.text_item:
                        btn.text_item.setVisible(False)
                    btn.setStyleSheet(
                        """
                        QPushButton {
                            background-color: transparent;
                            border: 2px solid #0000FF;
                            font-size: 16px;
                            font-weight: bold;
                            color: #0000FF;
                        }
                        QPushButton:checked {
                            background-color: rgba(0, 0, 255, 0.3);
                            color: #0000FF;
                        }
                        """
                    )
                    blue_idx += 1
                
    # 크기 조절 버튼 이벤트 핸들러 (이미 구현되어 있을 수 있음)
    def grow_selected_button_horizontal(self):
        """수평 방향으로 금지 구역 크기 증가"""
        if hasattr(self, 'selected_area_button') and self.selected_area_button:
            current_rect = self.selected_area_button.geometry()
            new_width = current_rect.width() + 10  # 10픽셀씩 증가
            
            # 중심점 유지하면서 너비 변경
            center_x = current_rect.x() + current_rect.width()/2
            center_y = current_rect.y() + current_rect.height()/2
            new_x = center_x - new_width/2
            
            # 맵 이미지 영역을 벗어나는지 확인
            if hasattr(self, 'map_manager') and self.map_manager and self.map_manager.map_overlay_pixmap:
                map_width = self.map_manager.map_overlay_pixmap.width()
                map_height = self.map_manager.map_overlay_pixmap.height()
                
                # 버튼이 맵 영역을 벗어나는지 확인
                if new_x < 0 or new_x + new_width > map_width:
                    print("맵 이미지 영역을 벗어남")
                    ToastManager.instance().show_toast("맵 영역을 벗어나 크기를 조절할 수 없습니다.", self)
                    return
            
            # 우선 경로 영역인 경우 확장 영역의 색상 유효성 검사
            area_type = self.selected_area_button.property("area_type")
            if area_type == "우선 경로":
                # 확장될 영역의 반경 계산 (기존 버튼 크기의 절반 + 새로 추가될 영역의 절반)
                radius = new_width / 2
                
                # 색상 유효성 검사
                is_valid = self.check_area_color_validity(center_x, center_y, radius)
                if not is_valid:
                    print("우선 경로 영역 확장 실패: 흰색(255,255,255) 외의 다른 색상이 포함된 영역입니다.")
                    # Toast 메시지 표시
                    ToastManager.instance().show_toast("장애물이 있습니다.", self)
                    return
            
            self.selected_area_button.setGeometry(
                int(new_x), 
                current_rect.y(), 
                new_width, 
                current_rect.height()
            )
            print(f"[on_grow_horizontal_clicked] 버튼 크기 변경: {new_width}x{current_rect.height()}")
            
            # 버튼 크기가 변경되었음을 표시
            self.area_buttons_modified = True

            self.update_button_map_points(self.selected_area_button)

    def shrink_selected_button_horizontal(self):
        """수평 방향으로 금지 구역 크기 감소"""
        if hasattr(self, 'selected_area_button') and self.selected_area_button:
            current_rect = self.selected_area_button.geometry()
            new_width = max(20, current_rect.width() - 10)  # 최소 20픽셀 유지
            
            # 중심점 유지하면서 너비 변경
            center_x = current_rect.x() + current_rect.width()/2
            center_y = current_rect.y() + current_rect.height()/2
            new_x = center_x - new_width/2
            
            self.selected_area_button.setGeometry(
                int(new_x), 
                current_rect.y(), 
                new_width, 
                current_rect.height()
            )
            print(f"[on_shrink_horizontal_clicked] 버튼 크기 변경: {new_width}x{current_rect.height()}")
            
            # 버튼 크기가 변경되었음을 표시
            self.area_buttons_modified = True

            self.update_button_map_points(self.selected_area_button)

    def grow_selected_button_vertical(self):
        """수직 방향으로 금지 구역 크기 증가"""
        if hasattr(self, 'selected_area_button') and self.selected_area_button:
            current_rect = self.selected_area_button.geometry()
            new_height = current_rect.height() + 10  # 10픽셀씩 증가
            
            # 중심점 유지하면서 높이 변경
            center_x = current_rect.x() + current_rect.width()/2
            center_y = current_rect.y() + current_rect.height()/2
            new_y = center_y - new_height/2
            
            # 맵 이미지 영역을 벗어나는지 확인
            if hasattr(self, 'map_manager') and self.map_manager and self.map_manager.map_overlay_pixmap:
                map_width = self.map_manager.map_overlay_pixmap.width()
                map_height = self.map_manager.map_overlay_pixmap.height()
                
                # 버튼이 맵 영역을 벗어나는지 확인
                if new_y < 0 or new_y + new_height > map_height:
                    print("맵 이미지 영역을 벗어남")
                    ToastManager.instance().show_toast("맵 영역을 벗어나 크기를 조절할 수 없습니다.", self)
                    return
            
            # 우선 경로 영역인 경우 확장 영역의 색상 유효성 검사
            area_type = self.selected_area_button.property("area_type")
            if area_type == "우선 경로":
                # 확장될 영역의 반경 계산 (가로 길이의 절반과 새로운 세로 길이의 절반 중 큰 값)
                radius = max(current_rect.width() / 2, new_height / 2)
                
                # 색상 유효성 검사
                is_valid = self.check_area_color_validity(center_x, center_y, radius)
                if not is_valid:
                    print("우선 경로 영역 확장 실패: 흰색(255,255,255) 외의 다른 색상이 포함된 영역입니다.")
                    ToastManager.instance().show_toast("장애물이 있습니다.", self)
                    return
            
            self.selected_area_button.setGeometry(
                current_rect.x(), 
                int(new_y), 
                current_rect.width(), 
                new_height
            )
            print(f"[on_grow_vertical_clicked] 버튼 크기 변경: {current_rect.width()}x{new_height}")
            
            # 버튼 크기가 변경되었음을 표시
            self.area_buttons_modified = True

            self.update_button_map_points(self.selected_area_button)

    def shrink_selected_button_vertical(self):
        """수직 방향으로 금지 구역 크기 감소"""
        if hasattr(self, 'selected_area_button') and self.selected_area_button:
            current_rect = self.selected_area_button.geometry()
            new_height = max(20, current_rect.height() - 10)  # 최소 20픽셀 유지
            
            # 중심점 유지하면서 높이 변경
            center_x = current_rect.x() + current_rect.width()/2
            center_y = current_rect.y() + current_rect.height()/2
            new_y = center_y - new_height/2
            
            self.selected_area_button.setGeometry(
                current_rect.x(), 
                int(new_y), 
                current_rect.width(), 
                new_height
            )
            print(f"[on_shrink_vertical_clicked] 버튼 크기 변경: {current_rect.width()}x{new_height}")
            
            # 버튼 크기가 변경되었음을 표시
            self.area_buttons_modified = True

            self.update_button_map_points(self.selected_area_button)

    def move_selected_button_up(self):
        selected_button = self.get_selected_area_button()
        if selected_button:
            current_pos = selected_button.pos()
            current_rect = selected_button.geometry()
            new_y = current_pos.y() - 10
            
            # 맵 이미지 영역을 벗어나는지 확인
            if hasattr(self, 'map_manager') and self.map_manager and self.map_manager.map_overlay_pixmap:
                if new_y < 0:
                    print("맵 이미지 영역을 벗어남")
                    ToastManager.instance().show_toast("맵 영역을 벗어날 수 없습니다.", self)
                    return
                
                # 버튼의 위쪽 가장자리를 따라 픽셀 색상 확인
                button_width = current_rect.width()
                
                # 위쪽으로 이동할 위치의 픽셀 색상 확인
                for check_x in range(int(current_pos.x()), int(current_pos.x() + button_width), 5):
                    check_pos = QPointF(check_x, new_y)
                    color = self.get_pixel_color_at_point(check_pos)
                    
                    # 색상이 흰색(255,255,255)이 아닌 경우 이동 제한
                    if color and (color.red() != 255 or color.green() != 255 or color.blue() != 255):
                        print(f"이동 제한: 위치 ({check_x}, {new_y})에 흰색이 아닌 색상 감지됨")
                        ToastManager.instance().show_toast("흰색 영역으로만 이동할 수 있습니다.", self)
                        return
            
            selected_button.move(current_pos.x(), new_y)
            self.update_button_map_points(selected_button)

    def move_selected_button_down(self):
        selected_button = self.get_selected_area_button()
        if selected_button:
            current_pos = selected_button.pos()
            current_rect = selected_button.geometry()
            new_y = current_pos.y() + 10
            
            # 맵 이미지 영역을 벗어나는지 확인
            if hasattr(self, 'map_manager') and self.map_manager and self.map_manager.map_overlay_pixmap:
                map_height = self.map_manager.map_overlay_pixmap.height()
                if new_y + current_rect.height() > map_height:
                    print("맵 이미지 영역을 벗어남")
                    ToastManager.instance().show_toast("맵 영역을 벗어날 수 없습니다.", self)
                    return
                
                # 버튼의 아래쪽 가장자리를 따라 픽셀 색상 확인
                button_width = current_rect.width()
                bottom_edge_y = new_y + current_rect.height()
                
                # 아래쪽으로 이동할 위치의 픽셀 색상 확인
                for check_x in range(int(current_pos.x()), int(current_pos.x() + button_width), 5):
                    check_pos = QPointF(check_x, bottom_edge_y)
                    color = self.get_pixel_color_at_point(check_pos)
                    
                    # 색상이 흰색(255,255,255)이 아닌 경우 이동 제한
                    if color and (color.red() != 255 or color.green() != 255 or color.blue() != 255):
                        print(f"이동 제한: 위치 ({check_x}, {bottom_edge_y})에 흰색이 아닌 색상 감지됨")
                        ToastManager.instance().show_toast("흰색 영역으로만 이동할 수 있습니다.", self)
                        return
            
            selected_button.move(current_pos.x(), new_y)
            self.update_button_map_points(selected_button)

    def move_selected_button_left(self):
        selected_button = self.get_selected_area_button()
        if selected_button:
            current_pos = selected_button.pos()
            new_x = current_pos.x() - 10
            
            # 맵 이미지 영역을 벗어나는지 확인
            if hasattr(self, 'map_manager') and self.map_manager and self.map_manager.map_overlay_pixmap:
                if new_x < 0:
                    print("맵 이미지 영역을 벗어남")
                    ToastManager.instance().show_toast("맵 영역을 벗어날 수 없습니다.", self)
                    return
                
                # 버튼의 왼쪽 가장자리를 따라 픽셀 색상 확인
                button_rect = selected_button.geometry()
                button_height = button_rect.height()
                
                # 왼쪽으로 이동할 위치의 픽셀 색상 확인
                for check_y in range(int(current_pos.y()), int(current_pos.y() + button_height), 5):
                    check_pos = QPointF(new_x, check_y)
                    color = self.get_pixel_color_at_point(check_pos)
                    
                    # 색상이 흰색(255,255,255)이 아닌 경우 이동 제한
                    if color and (color.red() != 255 or color.green() != 255 or color.blue() != 255):
                        print(f"이동 제한: 위치 ({new_x}, {check_y})에 흰색이 아닌 색상 감지됨")
                        ToastManager.instance().show_toast("흰색 영역으로만 이동할 수 있습니다.", self)
                        return
            
            selected_button.move(new_x, current_pos.y())
            self.update_button_map_points(selected_button)

    def move_selected_button_right(self):
        selected_button = self.get_selected_area_button()
        if selected_button:
            current_pos = selected_button.pos()
            current_rect = selected_button.geometry()
            new_x = current_pos.x() + 10
            
            # 맵 이미지 영역을 벗어나는지 확인
            if hasattr(self, 'map_manager') and self.map_manager and self.map_manager.map_overlay_pixmap:
                map_width = self.map_manager.map_overlay_pixmap.width()
                if new_x + current_rect.width() > map_width:
                    print("맵 이미지 영역을 벗어남")
                    ToastManager.instance().show_toast("맵 영역을 벗어날 수 없습니다.", self)
                    return
                
                # 버튼의 오른쪽 가장자리를 따라 픽셀 색상 확인
                button_height = current_rect.height()
                right_edge_x = new_x + current_rect.width()
                
                # 오른쪽으로 이동할 위치의 픽셀 색상 확인
                for check_y in range(int(current_pos.y()), int(current_pos.y() + button_height), 5):
                    check_pos = QPointF(right_edge_x, check_y)
                    color = self.get_pixel_color_at_point(check_pos)
                    
                    # 색상이 흰색(255,255,255)이 아닌 경우 이동 제한
                    if color and (color.red() != 255 or color.green() != 255 or color.blue() != 255):
                        print(f"이동 제한: 위치 ({right_edge_x}, {check_y})에 흰색이 아닌 색상 감지됨")
                        ToastManager.instance().show_toast("흰색 영역으로만 이동할 수 있습니다.", self)
                        return
            
            selected_button.move(new_x, current_pos.y())
            self.update_button_map_points(selected_button)

    def on_restricted_area_button_clicked(self):
        """금지 구역 버튼 클릭 시"""
        print("금지 구역 버튼 클릭")
        if self.restricted_area_button.isChecked():
            self.priority_path_area_button.setChecked(False)
            self.is_restricted_area_drawing = True
            self.current_area_type = "restricted"
            # map_control_button 체크 해제
            if hasattr(self, 'map_control_button') and self.map_control_button:
                self.map_control_button.setChecked(False)
                self.update_control_widgets_visibility()
        else:
            self.is_restricted_area_drawing = False

    def on_priority_path_area_button_clicked(self):
        """우선 경로 버튼 클릭 시"""
        print("우선 경로 버튼 클릭")
        if self.priority_path_area_button.isChecked():
            self.restricted_area_button.setChecked(False)
            self.is_restricted_area_drawing = False
            self.current_area_type = "priority"
            # map_control_button 체크 해제
            if hasattr(self, 'map_control_button') and self.map_control_button:
                self.map_control_button.setChecked(False)
                self.update_control_widgets_visibility()
        else:
            self.is_restricted_area_drawing = True

    def convert_scene_to_map_coords(self, scene_pos):
        """
        씬 좌표를 맵 좌표로 변환하는 통합 함수
        
        이 함수는 그래픽 뷰의 씬 좌표를 실제 맵 좌표(미터 단위)로 변환합니다.
        eventFilter, on_apply_area 등 여러 곳에서 동일한 좌표 변환 로직을 사용해야 할 때 활용합니다.
        
        Parameters:
        - scene_pos (QPointF): 변환할 씬 좌표
        
        Returns:
        - map_x (float): 변환된 맵 X 좌표 (미터 단위)
        - map_y (float): 변환된 맵 Y 좌표 (미터 단위)
        - transformed_pos (QPointF): 변환 중간 좌표 (overlay_transform 적용 후)
        """
        if not self.map_manager.map_info:
            self.map_manager.setup_default_map_info()
        
        scene_rect = self.map_manager.map_scene.sceneRect()
        if scene_rect.isEmpty() and self.map_manager.map_overlay_pixmap:
            scene_rect = self.map_manager.map_overlay_pixmap.rect()
        
        if self.map_manager.overlay_transform:
            # 변환 적용 (map_transform.json에 저장된 변환 정보 사용)
            transform = QTransform()
            center_x = scene_rect.width() / 2
            center_y = scene_rect.height() / 2
            transform.translate(center_x + self.map_manager.overlay_transform['translate_x'], 
                               center_y + self.map_manager.overlay_transform['translate_y'])
            transform.rotate(self.map_manager.overlay_transform['rotation'])
            transform.scale(self.map_manager.overlay_transform['scale'], 
                           self.map_manager.overlay_transform['scale'])
            transform.translate(-center_x, -center_y)
            inverse_transform = transform.inverted()[0]
            original_pos = inverse_transform.map(scene_pos)
            
            # 맵 좌표 계산
            adjusted_scene_x = scene_rect.width() - original_pos.x()
            map_x = adjusted_scene_x * self.map_manager.map_info.resolution + self.map_manager.map_info.origin.position.x
            map_y = original_pos.y() * self.map_manager.map_info.resolution + self.map_manager.map_info.origin.position.y
            
            return map_x, map_y, original_pos
        else:
            # overlay_transform 없는 경우 직접 변환
            adjusted_scene_x = scene_rect.width() - scene_pos.x()
            map_x = adjusted_scene_x * self.map_manager.map_info.resolution + self.map_manager.map_info.origin.position.x
            map_y = scene_pos.y() * self.map_manager.map_info.resolution + self.map_manager.map_info.origin.position.y
            
            return map_x, map_y, scene_pos

    def convert_map_to_scene_coords(self, map_x, map_y):
        """
        맵 좌표(map_x, map_y)를 고정 기준의 씬 좌표(QPointF)로 변환
        기준 좌표계(예: sceneRect.width())에 영향을 받지 않도록 고정값 사용
        Returns:
            QPointF: 씬 좌표
        """
        if not hasattr(self, 'map_manager') or self.map_manager is None:
            print("[convert_map_to_scene_coords] 맵 매니저가 초기화되지 않았습니다.")
            return QPointF(0, 0)
            
        if not hasattr(self.map_manager, 'map_info') or not self.map_manager.map_info:
            self.map_manager.setup_default_map_info()
            print("[convert_map_to_scene_coords] 맵 정보가 없어 기본값으로 초기화했습니다.")

        map_info = self.map_manager.map_info
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        scene_width = map_info.width    # scene의 픽셀 너비 (예: 800)
        scene_height = map_info.height  # scene의 픽셀 높이 (예: 600)

        # 1. 맵 좌표 → 셀 기준 좌표로 변환
        map_cell_x = (map_x - origin_x) / resolution
        map_cell_y = (map_y - origin_y) / resolution

        # 2. 씬 좌표로 변환 (X축은 좌우 반전: 오른쪽으로 갈수록 map_x가 작아짐)
        scene_x = scene_width - map_cell_x
        scene_y = map_cell_y

        scene_pos = QPointF(scene_x, scene_y)

        # 3. overlay_transform 적용 (있으면)
        if self.map_manager.overlay_transform:
            transform = QTransform()
            center_x = scene_width / 2
            center_y = scene_height / 2
            t = self.map_manager.overlay_transform

            transform.translate(center_x + t['translate_x'], center_y + t['translate_y'])
            transform.rotate(t['rotation'])
            transform.scale(t['scale'], t['scale'])
            transform.translate(-center_x, -center_y)

            scene_pos = transform.map(scene_pos)

        return scene_pos

    def on_map_refresh(self):
        """맵 뷰 초기화 - 맵 뷰의 변환 행렬을 초기화하고 맵을 원래 위치와 크기로 재설정"""
        if hasattr(self, 'map_manager') and self.map_manager and self.map_view:
            print("맵 뷰 초기화 실행")
            
            # 맵 뷰의 변환 행렬 초기화 (확대/축소 및 이동 초기화)
            self.map_view.resetTransform()
            
            # 스크롤바 초기 위치로 설정
            self.map_view.horizontalScrollBar().setValue(0)
            self.map_view.verticalScrollBar().setValue(0)
            
            # 맵 뷰의 변환 행렬 초기화 후 맵 전체가 보이도록 조정
            self.map_view.fitInView(self.map_manager.map_scene.sceneRect(), Qt.KeepAspectRatio)
            
            # 맵 뷰 업데이트
            self.map_view.update()
            
            print("맵 뷰 초기화 완료")
        else:
            print("맵 뷰 초기화 실패: 맵 매니저 또는 맵 뷰가 없음")

    def get_pixel_color_at_point(self, scene_pos):
        """
        그래픽 뷰에 로드된 이미지에서 특정 위치의 픽셀 색상을 반환합니다.
        Parameters:
            scene_pos (QPointF): 씬 좌표
        Returns:
            QColor: 해당 위치의 픽셀 색상, 색상을 가져올 수 없으면 None
        """
        try:
            # 맵 매니저에서 맵 오버레이 이미지 가져오기
            if hasattr(self, 'map_manager') and self.map_manager and self.map_manager.map_overlay_pixmap:
                # QPixmap을 QImage로 변환
                image = self.map_manager.map_overlay_pixmap.toImage()
                
                # 씬 좌표를 이미지 좌표로 변환
                x = int(scene_pos.x())
                y = int(scene_pos.y())
                
                # 이미지 범위 내에 있는지 확인
                if 0 <= x < image.width() and 0 <= y < image.height():
                    # 해당 위치의 픽셀 색상 가져오기
                    color = QColor(image.pixel(x, y))
                    print(f"위치 ({x}, {y})의 픽셀 색상: RGB({color.red()}, {color.green()}, {color.blue()})")
                    return color
                else:
                    print(f"위치 ({x}, {y})가 이미지 범위를 벗어남 (크기: {image.width()}x{image.height()})")
                    return None
            else:
                print("맵 오버레이 이미지를 찾을 수 없음")
                return None
        except Exception as e:
            print(f"픽셀 색상 가져오기 오류: {e}")
            return None

    def is_obstacle_at_point(self, scene_pos, threshold=200):
        """
        특정 위치가 장애물인지 확인합니다. (흰색이 아닌 픽셀은 장애물로 간주)
        Parameters:
            scene_pos (QPointF): 씬 좌표
            threshold (int): 흰색으로 간주할 RGB 값의 임계값 (기본값: 200)
        Returns:
            bool: 장애물이면 True, 아니면 False
        """
        color = self.get_pixel_color_at_point(scene_pos)
        if color is None:
            return False  # 색상을 가져올 수 없으면 장애물이 아닌 것으로 간주
        
        # RGB 값이 모두 임계값 이상이면 흰색으로 간주 (장애물 아님)
        is_white = color.red() > threshold and color.green() > threshold and color.blue() > threshold
        return not is_white  # 흰색이 아니면 장애물

    def update_button_map_points(self, button):
        """버튼의 현재 위치와 크기를 기반으로 map_points 값을 업데이트"""
        if button:
            # 버튼의 현재 위치와 크기 가져오기
            button_rect = button.geometry()
            
            # 버튼의 네 꼭짓점 계산
            points = [
                QPointF(button_rect.x(), button_rect.y()),  # 좌상단
                QPointF(button_rect.x() + button_rect.width(), button_rect.y()),  # 우상단
                QPointF(button_rect.x() + button_rect.width(), button_rect.y() + button_rect.height()),  # 우하단
                QPointF(button_rect.x(), button_rect.y() + button_rect.height())  # 좌하단
            ]

            # 각 꼭짓점을 맵 좌표로 변환
            map_points = []
            for i, point in enumerate(points):
                # 씬 좌표를 맵 좌표로 변환
                map_x, map_y, _ = self.convert_scene_to_map_coords(point)
                map_points.append([map_x, map_y])
                print(f"꼭짓점 {i + 1}: 씬 좌표({point.x():.3f}, {point.y():.3f}) -> 맵 좌표({map_x:.3f}, {map_y:.3f})")
            
            # 버튼 객체에 맵 좌표 저장
            button.map_points = map_points
            print(f"버튼 map_points 최종 업데이트: {map_points}")
            
            # 버튼 크기가 변경되었음을 표시
            self.area_buttons_modified = True

    def check_area_color_validity(self, center_x, center_y, radius):
        """
        지정된 영역 내의 모든 픽셀이 흰색(255,255,255)인지 확인
        
        Args:
            center_x (float): 중심 X 좌표
            center_y (float): 중심 Y 좌표
            radius (float): 검사할 영역의 반경
            
        Returns:
            bool: 모든 픽셀이 흰색이면 True, 아니면 False
        """
        print(f"색상 유효성 검사 시작: 중심=({center_x}, {center_y}), 반경={radius}")
        
        # 맵 이미지가 없으면 검사 불가
        if not hasattr(self, 'map_manager') or not self.map_manager or not self.map_manager.map_overlay_pixmap:
            print("맵 이미지가 없어 색상 검사를 수행할 수 없습니다.")
            return True  # 맵 이미지가 없으면 검사 통과로 처리
            
        # 맵 이미지를 QImage로 변환
        map_image = self.map_manager.map_overlay_pixmap.toImage()
        
        # 검사 영역 좌표 계산
        left = max(0, int(center_x - radius))
        top = max(0, int(center_y - radius))
        right = min(map_image.width() - 1, int(center_x + radius))
        bottom = min(map_image.height() - 1, int(center_y + radius))
        
        print(f"검사 영역: 좌상단=({left}, {top}), 우하단=({right}, {bottom})")
        
        # 영역 내 모든 픽셀 검사
        non_white_pixels = 0
        for y in range(top, bottom + 1):
            for x in range(left, right + 1):
                # 원 영역 내 픽셀만 검사 (사각형이 아닌 원형 영역)
                if (x - center_x)**2 + (y - center_y)**2 <= radius**2:
                    pixel_color = map_image.pixelColor(x, y)
                    r, g, b = pixel_color.red(), pixel_color.green(), pixel_color.blue()
                    
                    # 흰색(255,255,255)이 아닌 픽셀 발견
                    if r != 255 or g != 255 or b != 255:
                        non_white_pixels += 1
                        print(f"흰색이 아닌 픽셀 발견: 위치=({x}, {y}), RGB=({r},{g},{b})")
                        if non_white_pixels >= 5:  # 5개 이상 발견되면 검사 중단
                            print(f"흰색이 아닌 픽셀 {non_white_pixels}개 발견, 검사 중단")
                            return False
        
        if non_white_pixels > 0:
            print(f"흰색이 아닌 픽셀 {non_white_pixels}개 발견")
            return False
            
        print("모든 픽셀이 흰색(255,255,255)입니다.")
        return True

    def is_point_in_map_image(self, scene_pos):
        """
        특정 위치가 맵 이미지 내부인지 확인합니다.
        Parameters:
            scene_pos (QPointF): 씬 좌표
        Returns:
            bool: 맵 이미지 내부이면 True, 아니면 False
        """
        try:
            # 맵 매니저에서 맵 오버레이 이미지 가져오기
            if hasattr(self, 'map_manager') and self.map_manager and self.map_manager.map_overlay_pixmap:
                # 씬 좌표를 이미지 좌표로 변환
                x = int(scene_pos.x())
                y = int(scene_pos.y())
                
                # 이미지 범위 내에 있는지 확인
                image_width = self.map_manager.map_overlay_pixmap.width()
                image_height = self.map_manager.map_overlay_pixmap.height()
                
                if 0 <= x < image_width and 0 <= y < image_height:
                    # 해당 위치의 픽셀 색상 가져오기
                    color = self.get_pixel_color_at_point(scene_pos)
                    if color is not None:
                        return True
                    
                print(f"위치 ({x}, {y})가 이미지 범위를 벗어남 (크기: {image_width}x{image_height})")
                return False
            else:
                print("맵 오버레이 이미지를 찾을 수 없음")
                return False
        except Exception as e:
            print(f"맵 이미지 내부 확인 오류: {e}")
            return False
            
    def set_area_index_visibility(self, show_index):
        """영역 인덱스 표시 여부 설정
        
        Args:
            show_index (bool): 인덱스 표시 여부
        """
        print(f"[LocationAddView] 영역 인덱스 표시 설정: {show_index}, 타입: {type(show_index)}")
        
        # bool 타입 확인 및 변환
        if not isinstance(show_index, bool):
            print(f"[LocationAddView] show_index가 bool 타입이 아님: {show_index}, {type(show_index)}")
            # 문자열인 경우 변환
            if isinstance(show_index, str):
                show_index = (show_index.lower() == 'true')
            # 숫자인 경우 변환
            elif isinstance(show_index, (int, float)):
                show_index = bool(show_index)
            print(f"[LocationAddView] show_index를 bool로 변환: {show_index}")
        
        # 영역 버튼 인덱스 표시 여부 설정
        button_count = 0
        for btn in self.area_buttons:
            if isinstance(btn, QPushButton) and getattr(btn, 'is_area_button', False):
                button_count += 1
                area_type = btn.property("area_type")
                internal_index = btn.property("internal_index")
                
                # internal_index가 없으면 기본값 설정
                if internal_index is None:
                    if area_type == "금지 구역":
                        # 금지 구역 버튼 인덱스 계산
                        red_idx = 1
                        for b in self.area_buttons:
                            if isinstance(b, QPushButton) and getattr(b, 'is_area_button', False):
                                if b.property("area_type") == "금지 구역" and b != btn:
                                    red_idx += 1
                        internal_index = red_idx
                        btn.setProperty("internal_index", internal_index)
                    elif area_type == "우선 경로":
                        # 우선 경로 버튼 인덱스 계산
                        blue_idx = 1
                        for b in self.area_buttons:
                            if isinstance(b, QPushButton) and getattr(b, 'is_area_button', False):
                                if b.property("area_type") == "우선 경로" and b != btn:
                                    blue_idx += 1
                        internal_index = blue_idx
                        btn.setProperty("internal_index", internal_index)
                
                print(f"[LocationAddView] 버튼 {button_count}: area_type={area_type}, internal_index={internal_index}")
                
                if show_index:
                    # 인덱스 표시
                    if area_type == "금지 구역":
                        btn.setText(str(internal_index))
                        print(f"[LocationAddView] 금지 구역 버튼 {internal_index}에 텍스트 설정")
                    elif area_type == "우선 경로":
                        btn.setText(str(internal_index))
                        print(f"[LocationAddView] 우선 경로 버튼 {internal_index}에 텍스트 설정")
                    
                    # 텍스트 항목 표시
                    if hasattr(btn, 'text_item') and btn.text_item:
                        btn.text_item.setVisible(True)
                        btn.text_item.setPlainText(str(internal_index))
                        print(f"[LocationAddView] 텍스트 항목 표시: {internal_index}")
                else:
                    # 인덱스 숨기기
                    btn.setText("")
                    print(f"[LocationAddView] 버튼 텍스트 숨김")
                    
                    # 텍스트 항목 숨기기
                    if hasattr(btn, 'text_item') and btn.text_item:
                        btn.text_item.setVisible(False)
                        print(f"[LocationAddView] 텍스트 항목 숨김")
        
        print(f"[LocationAddView] 총 {button_count}개의 버튼 처리 완료")
                        
        # 영역 인덱스 표시 여부 저장
        self.show_area_index = show_index
        
    def on_map_control_button_clicked(self):
        """맵 컨트롤 버튼 클릭 이벤트 핸들러"""
        # map_control_button이 체크되면 영역 버튼들 체크 해제
        if self.map_control_button.isChecked():
            if hasattr(self, 'restricted_area_button') and self.restricted_area_button:
                self.restricted_area_button.setChecked(False)
            if hasattr(self, 'priority_path_area_button') and self.priority_path_area_button:
                self.priority_path_area_button.setChecked(False)
                
        self.update_control_widgets_visibility()
        
    def update_control_widgets_visibility(self):
        """map_control_button 상태에 따라 위젯 표시 여부 업데이트"""
        # 영역 설정 모드가 아닐 때는 항상 move_area_widget 숨김
        if not hasattr(self, 'is_area_setting_screen') or not self.is_area_setting_screen:
            if hasattr(self, 'move_area_widget') and self.move_area_widget:
                self.move_area_widget.hide()
            return
            
        # 영역 설정 모드일 때만 map_control_button 상태에 따라 위젯 표시 여부 업데이트
        if hasattr(self, 'map_control_button') and self.map_control_button:
            if self.map_control_button.isChecked():
                # 맵 컨트롤 위젯 표시, 영역 이동 위젯 숨김
                if hasattr(self, 'map_control_widget') and self.map_control_widget:
                    self.map_control_widget.show()
                if hasattr(self, 'move_area_widget') and self.move_area_widget:
                    self.move_area_widget.hide()
            else:
                # 맵 컨트롤 위젯 숨김, 영역 이동 위젯 표시
                if hasattr(self, 'map_control_widget') and self.map_control_widget:
                    self.map_control_widget.hide()
                if hasattr(self, 'move_area_widget') and self.move_area_widget:
                    self.move_area_widget.show()
                    
    def setup_navigation_control_widget(self):
        """네비게이션 컨트롤 위젯 설정"""
        # UI 파일에서 위젯 찾기
        self.navigation_control_widget = self.ui.findChild(QWidget, "navigation_control_widget")
        self.map_control_type_button = self.ui.findChild(QPushButton, "map_control_type_button")
        self.marker_control_type_button = self.ui.findChild(QPushButton, "marker_control_type_button")
        
        # 맵 컨트롤 위젯 찾기
        self.map_control_widget = self.ui.findChild(QWidget, "map_control_widget")
        
        # 맵 컨트롤 버튼 찾기
        self.map_zoom_in_button = self.ui.findChild(QPushButton, "object_zoom_in")
        self.map_zoom_out_button = self.ui.findChild(QPushButton, "object_zoom_out")
        self.map_move_up_button = self.ui.findChild(QPushButton, "map_up")
        self.map_move_down_button = self.ui.findChild(QPushButton, "map_down")
        self.map_move_left_button = self.ui.findChild(QPushButton, "map_left")
        self.map_move_right_button = self.ui.findChild(QPushButton, "map_right")
        self.map_move_refresh_button = self.ui.findChild(QPushButton, "map_move_refresh")
        
        # 마커 컨트롤 위젯 찾기
        self.marker_control_widget = self.ui.findChild(QWidget, "marker_control_widget")
        
        # 마커 컨트롤 버튼 찾기
        self.robot_move_up_button = self.ui.findChild(QPushButton, "robot_move_up")
        self.robot_move_down_button = self.ui.findChild(QPushButton, "robot_move_down")
        self.robot_move_left_button = self.ui.findChild(QPushButton, "robot_move_left")
        self.robot_move_right_button = self.ui.findChild(QPushButton, "robot_move_right")
        self.robot_rotate_left_button = self.ui.findChild(QPushButton, "robot_rotate_left")
        self.robot_rotate_right_button = self.ui.findChild(QPushButton, "robot_rotate_right")
        
        # 초기 상태 설정
        self.map_control_type_button.setChecked(True)
        self.marker_control_type_button.setChecked(False)
        self.map_control_widget.show()
        self.marker_control_widget.hide()
        
        # 버튼 연결
        self.map_control_type_button.clicked.connect(self.on_map_control_type_clicked)
        self.marker_control_type_button.clicked.connect(self.on_marker_control_type_clicked)
        
        # 맵 컨트롤 버튼 시그널 연결
        if self.map_zoom_in_button:
            self.map_zoom_in_button.clicked.connect(self.on_map_zoom_in)
        if self.map_zoom_out_button:
            self.map_zoom_out_button.clicked.connect(self.on_map_zoom_out)
        if self.map_move_up_button:
            self.map_move_up_button.clicked.connect(self.on_map_move_up)
        if self.map_move_down_button:
            self.map_move_down_button.clicked.connect(self.on_map_move_down)
        if self.map_move_left_button:
            self.map_move_left_button.clicked.connect(self.on_map_move_left)
        if self.map_move_right_button:
            self.map_move_right_button.clicked.connect(self.on_map_move_right)
        if self.map_move_refresh_button:
            self.map_move_refresh_button.clicked.connect(self.on_map_refresh)
        
        # 마커 컨트롤 버튼 시그널 연결
        if self.robot_move_up_button:
            self.robot_move_up_button.clicked.connect(lambda: self.move_robot('up'))
        if self.robot_move_down_button:
            self.robot_move_down_button.clicked.connect(lambda: self.move_robot('down'))
        if self.robot_move_left_button:
            self.robot_move_left_button.clicked.connect(lambda: self.move_robot('left'))
        if self.robot_move_right_button:
            self.robot_move_right_button.clicked.connect(lambda: self.move_robot('right'))
        if self.robot_rotate_left_button:
            self.robot_rotate_left_button.clicked.connect(lambda: self.rotate_robot('left'))
        if self.robot_rotate_right_button:
            self.robot_rotate_right_button.clicked.connect(lambda: self.rotate_robot('right'))
        
    def on_map_control_type_clicked(self):
        """맵 컨트롤 타입 버튼 클릭 시 처리"""
        self.map_control_type_button.setChecked(True)
        self.marker_control_type_button.setChecked(False)
        self.map_control_widget.show()
        self.marker_control_widget.hide()
        
    def on_marker_control_type_clicked(self):
        """마커 컨트롤 타입 버튼 클릭 시 처리"""
        self.map_control_type_button.setChecked(False)
        self.marker_control_type_button.setChecked(True)
        self.map_control_widget.hide()
        self.marker_control_widget.show()
