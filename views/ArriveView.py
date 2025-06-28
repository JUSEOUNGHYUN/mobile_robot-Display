from PySide6.QtWidgets import QWidget, QVBoxLayout, QSizePolicy, QPushButton, QLabel
from PySide6.QtCore import Signal, QFile, Qt, QSize
from PySide6.QtGui import QIcon
from PySide6.QtUiTools import QUiLoader
from views.StatusBar import StatusBar, StatusBarType
from Utils.MessageBox import MessageBox
import os

class ArriveView(QWidget):
    ok_signal = Signal()
    locate_select_signal = Signal()  # 목적지 선택 화면으로 이동하는 시그널 추가
    waiting_location_signal = Signal()  # 대기 장소 화면으로 이동하는 시그널 추가
    setting_signal = Signal()
    return_signal = Signal()  # 돌아가기 시그널 추가
    location_fixed_signal = Signal(bool)  # 위치 고정 상태 변경 시그널 (고정 여부 전달)

    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 타이틀바 제거
        self.setWindowFlags(Qt.FramelessWindowHint)
        
        # 크기 정책 추가
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # UI 파일 로드
        loader = QUiLoader()
        ui_file_path = "/home/dentium/Desktop/work/mobile_robot-Display/UI/ArriveView.ui"
        ui_file = QFile(ui_file_path)
        ui_file.open(QFile.ReadOnly)
        self.ui = loader.load(ui_file, self)
        ui_file.close()

        # 윈도우 설정
        self.setWindowTitle('Dentium')

        # 레이아웃 설정
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # 상태바 추가
        self.status_bar = StatusBar(self, StatusBarType.TITLE_SETTING, "도착")
        self.status_bar.setting_clicked.connect(self.on_setting_clicked)
        self.status_bar.back_signal.connect(self.on_return_clicked)  # back_signal 사용
        
        # status_bar_placeholder에 상태바 추가
        self.ui.status_bar_placeholder.setLayout(QVBoxLayout())
        self.ui.status_bar_placeholder.layout().setContentsMargins(0, 0, 0, 0)
        self.ui.status_bar_placeholder.layout().addWidget(self.status_bar)
        
        # 위치 고정 상태 초기화
        self.is_location_fixed = False
        self.update_fix_button_state()

        # 버튼 시그널 연결
        self.ui.locate_select_button.clicked.connect(self.on_locate_select_clicked)
        self.ui.waiting_location_button.clicked.connect(self.on_waiting_location_clicked)
        self.ui.fix_location_button.clicked.connect(self.on_fix_location_clicked)

        # fix_location_button에 Lock.png 아이콘 지정
        self.ui.fix_location_button.setIcon(QIcon(":/file/lock.ico"))
        self.ui.fix_location_button.setIconSize(QSize(24, 24))

    def set_destination_name(self, name):
        """목적지 이름 설정"""
        self.ui.goal_name_label.setText(name)
        
    def set_arrived(self, arrived):
        """도착 상태 설정"""
        # 도착 상태에 따라 UI 업데이트
        try:
            # arrived 상태에 따라 UI 변경
            if arrived:
                if hasattr(self.ui, 'arrival_label'):
                    self.ui.arrival_label.setText("도착했습니다")
            else:
                if hasattr(self.ui, 'arrival_label'):
                    self.ui.arrival_label.setText("도착 대기 중...")
        except Exception as e:
            print(f"도착 상태 설정 중 오류 발생: {str(e)}")
            
    def reset_view(self):
        """뷰 초기화"""
        self.set_arrived(False)
        self.set_destination_name("목적지")
        self.is_location_fixed = False
        self.update_fix_button_state()
        self.ui.travel_time_label.hide()  # 시간 정보 라벨 숨김

    def on_locate_select_clicked(self):
        """목적지 선택 화면으로 이동"""
        # 위치 고정 상태일 때는 아무 동작 안함
        if self.is_location_fixed:
            return
            
        # 위치 고정이 아닐 때만 시그널 발생
        self.locate_select_signal.emit()

    def on_waiting_location_clicked(self):
        """대기 장소 화면으로 이동"""
        # 위치 고정 상태일 때는 아무 동작 안함
        if self.is_location_fixed:
            return
            
        # 위치 고정이 아닐 때만 시그널 발생
        self.waiting_location_signal.emit()

    def on_setting_clicked(self):
        """설정 버튼 클릭 시 호출"""
        self.setting_signal.emit()

    def on_return_clicked(self):
        """돌아가기 버튼 클릭 시 호출"""
        self.return_signal.emit()
        
    def on_fix_location_clicked(self):
        """위치 고정 버튼 클릭 시 호출"""
        # 현재 상태를 반대로 변경
        self.is_location_fixed = not self.is_location_fixed
        
        # 버튼 상태 업데이트
        self.update_fix_button_state()
        
        # 상태 변경 시그널 발생
        self.location_fixed_signal.emit(self.is_location_fixed)
        
    def update_fix_button_state(self):
        """위치 고정 버튼 상태 업데이트"""
        if not self.ui.fix_location_button:
            return
            
        if self.is_location_fixed:
            self.ui.fix_location_button.setText("위치 고정됨")
            self.ui.fix_location_button.setStyleSheet("""
                QPushButton {
                    background-color: #1e1e1e;
                    color: white;
                    font-size: 16px;
                    font-weight: 500;
                    border-radius: 10px;
                    border: 2px solid #444444;
                    font-family: 'Noto Sans KR Black';
                    padding-left: 16px; /* 아이콘과 텍스트 간격 */
                }
                QPushButton:hover {
                    background-color: #333333;
                    border: 2px solid #666666;
                }
                QPushButton:pressed {
                    background-color: #2a2a2a;
                }
            """)
        else:
            self.ui.fix_location_button.setText("위치 고정")
            self.ui.fix_location_button.setStyleSheet("""
                QPushButton {
                    background-color: #00A3FF;
                    color: white;
                    font-size: 16px;
                    font-weight: 500;
                    border-radius: 10px;
                    border: none;
                    font-family: "Noto Sans KR Black";
                }

                QPushButton:hover {
                    background-color: #33B5FF;
                }

                QPushButton:pressed {
                    background-color: #0080DD;
                }
            """)
            
    def set_fixed_state(self, is_fixed):
        """위치 고정 상태 설정"""
        self.is_location_fixed = is_fixed
        self.update_fix_button_state()

    def is_fixed(self):
        """현재 위치 고정 상태 반환"""
        return self.is_location_fixed

    def set_destinations(self, destinations):
        """이동 가능한 목적지 목록 설정"""
        try:
            # 목적지 목록을 UI에 표시하는 로직
            # 구현이 필요한 경우 여기에 구현
            print(f"{len(destinations)}개의 이동 가능한 목적지가 로드되었습니다.")
        except Exception as e:
            print(f"목적지 목록 설정 중 오류: {e}")

    def set_travel_time_info(self, departure_time, arrival_time, travel_duration):
        """이동 시간 정보 설정
        Args:
            departure_time (str): 출발 시간 문자열
            arrival_time (str): 도착 시간 문자열
            travel_duration (str): 이동 소요 시간 문자열
        """
        # 이동 소요 시간만 표시
        time_info = f"{travel_duration}"
        
        self.ui.travel_time_label.setText(time_info)
        self.ui.travel_time_label.show()  # 라벨 표시