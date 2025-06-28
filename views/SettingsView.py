from PySide6.QtWidgets import QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout
from PySide6.QtGui import QPixmap, QFont
from PySide6.QtCore import Qt, Signal, QFile, QDir
from PySide6.QtUiTools import QUiLoader
from views.StatusBar import StatusBar, StatusBarType
from Utils.ButtonStyle import RoundButtonStyle
import os

class SettingsView(QWidget):
    # 시그널 정의
    back_signal = Signal()
    info_signal = Signal()
    exit_signal = Signal()
    wifi_signal = Signal()  # 와이파이 버튼 시그널 추가
    move_option_signal = Signal()  # 이동 옵션 버튼 시그널 추가
    map_signal = Signal()  # 맵 버튼 시그널 추가
    touch_test_signal = Signal()  # 터치 테스트 버튼 시그널 추가
    developer_mode_signal = Signal()  # 개발자 모드 버튼 시그널 추가
    other_settings_signal = Signal()  # 기타 설정 버튼 시그널 추가

    def __init__(self, parent=None):
        super().__init__(parent)

        # UI 파일 로드
        loader = QUiLoader()

        ui_file_path = os.path.join(os.path.dirname(__file__), "..", "UI", "SettingsView.ui")
        ui_file = QFile(ui_file_path)
        ui_file.open(QFile.ReadOnly)
        self.ui = loader.load(ui_file, self)
        ui_file.close()

        # 윈도우 설정
        self.setWindowTitle('Dentium')
        self.setFixedSize(1024, 600)

        # 레이아웃 설정
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # 상태바 추가 (StatusBar WITH_BACK 타입 사용, 설정 버튼 없음)
        self.status_bar = StatusBar(self, StatusBarType.WITH_BACK, "설정")
        self.status_bar.back_signal.connect(self.go_back)  # 뒤로가기 버튼 시그널 연결

        # status_bar_placeholder에 상태바 추가
        placeholder_layout = QVBoxLayout(self.ui.status_bar_placeholder)
        placeholder_layout.setContentsMargins(0, 0, 0, 0)
        placeholder_layout.addWidget(self.status_bar)

        # UI 위젯을 메인 레이아웃에 추가
        main_layout.addWidget(self.ui)

        # 터치 테스트 버튼 생성 및 추가
        self.touch_test_button = QPushButton("터치 테스트", self.ui)
        self.touch_test_button.setGeometry(512, 350, 200, 60)
        self.touch_test_button.setStyleSheet("""
            QPushButton {
                background-color: #2A2A2A;
                color: white;
                border: 1px solid #444444;
                border-radius: 6px;
                font-size: 16px;
                font-weight: 500;
            }
            QPushButton:hover {
                background-color: #3A3A3A;
                border: 1px solid #666666;
            }
            QPushButton:pressed {
                background-color: #333333;
            }
        """)
        self.touch_test_button.clicked.connect(self.on_touch_test_clicked)
        self.touch_test_button.hide()

        # 버튼에 대한 시그널 연결
        self.ui.move_option_button.clicked.connect(self.on_move_option_clicked)  # 이동 옵션 버튼 연결
        self.ui.wifi_button.clicked.connect(self.on_wifi_clicked)
        self.ui.info_button.clicked.connect(self.on_info_clicked)
        self.ui.exit_button.clicked.connect(self.on_exit_clicked)
        self.ui.map_button.clicked.connect(self.on_map_clicked)  # 맵 버튼 시그널 연결
        self.ui.developer_button.clicked.connect(self.on_developer_mode_clicked)  # 개발자 모드 버튼 연결
        self.ui.other_settings_button.clicked.connect(self.on_other_settings_clicked)  # 기타 설정 버튼 연결

        # 버튼에 그림자 효과 적용
        RoundButtonStyle.apply_shadow_to_widget(self.ui.move_option_button, blur_radius=15, color_opacity=80, offset_x=3, offset_y=3)
        RoundButtonStyle.apply_shadow_to_widget(self.ui.wifi_button, blur_radius=15, color_opacity=80, offset_x=3, offset_y=3)
        RoundButtonStyle.apply_shadow_to_widget(self.ui.info_button, blur_radius=15, color_opacity=80, offset_x=3, offset_y=3)
        RoundButtonStyle.apply_shadow_to_widget(self.ui.exit_button, blur_radius=15, color_opacity=80, offset_x=3, offset_y=3)
        RoundButtonStyle.apply_shadow_to_widget(self.ui.map_button, blur_radius=15, color_opacity=80, offset_x=3, offset_y=3)
        RoundButtonStyle.apply_shadow_to_widget(self.ui.developer_button, blur_radius=15, color_opacity=80, offset_x=3, offset_y=3)
        RoundButtonStyle.apply_shadow_to_widget(self.ui.other_settings_button, blur_radius=15, color_opacity=80, offset_x=3, offset_y=3)

    # 와이파이 버튼 클릭 시 호출되는 메서드
    def on_wifi_clicked(self):
        self.wifi_signal.emit()

    # 정보 버튼 클릭 시 호출되는 메서드
    def on_info_clicked(self):
        self.info_signal.emit()

    # 프로그램 종료 버튼 클릭 시 호출되는 메서드
    def on_exit_clicked(self):
        self.exit_program()

    # 프로그램 종료
    def exit_program(self):
        self.exit_signal.emit()

    # 뒤로가기 버튼 클릭 시 호출되는 메서드
    def go_back(self):
        self.back_signal.emit()

    # 이동 옵션 버튼 클릭 시 호출되는 메서드
    def on_move_option_clicked(self):
        self.move_option_signal.emit()

    # 맵 버튼 클릭 시 호출되는 메서드
    def on_map_clicked(self):
        self.map_signal.emit()

    # 터치 테스트 버튼 클릭 시 호출되는 메서드
    def on_touch_test_clicked(self):
        self.touch_test_signal.emit()
        
    # 개발자 모드 버튼 클릭 시 호출되는 메서드
    def on_developer_mode_clicked(self):
        self.developer_mode_signal.emit()

    # 기타 설정 버튼 클릭 시 호출되는 메서드
    def on_other_settings_clicked(self):
        self.other_settings_signal.emit()

    # 상태바의 설정 아이콘 반환
    def get_setting_icon(self):
        # QLabel을 버튼처럼 사용하기 위한 처리
        icon = self.status_bar.get_setting_icon()
        return icon