"""모듈화된 상태 바 컴포넌트

다양한 형태의 상태 바를 생성할 수 있는 모듈화된 클래스입니다.
"""

from PySide6.QtWidgets import QFrame, QLabel, QHBoxLayout, QPushButton
from PySide6.QtGui import QPixmap, QFont, QCursor
from PySide6.QtCore import Qt, Signal
from enum import Enum, auto
from resources_rc import *

class StatusBarType(Enum):
    """상태 바 타입을 정의하는 열거형"""
    FULL = auto()       # 모든 아이콘을 포함한 기본 상태 바
    SIMPLE = auto()     # 로고와 배터리만 있는 심플한 상태 바
    WITH_BACK = auto()  # 뒤로가기 버튼이 있는 상태 바
    TITLE_SETTING = auto()  # 타이틀, 설정, 배터리 아이콘이 있는 상태 바

class StatusBar(QFrame):
    """모듈화된 상태 바 클래스
    파라미터를 통해 다양한 형태의 상태 바를 생성할 수 있습니다.
    """
    
    # 시그널 정의
    back_signal = Signal()       # 뒤로가기 버튼 클릭 시그널
    setting_clicked = Signal()   # 설정 버튼 클릭 시그널
    wifi_clicked = Signal()      # 와이파이 버튼 클릭 시그널
    
    def __init__(self, parent=None, bar_type=StatusBarType.FULL, title="", with_setting=False):
        """
        Args:
            parent: 부모 위젯
            bar_type (StatusBarType): 상태 바 타입
            title (str): 화면 제목 (WITH_BACK 타입에서 사용)
            with_setting (bool): 설정 버튼 사용 여부 (SIMPLE 타입에서 사용)
        """
        super().__init__(parent)
        
        # 기본 속성 설정
        self.bar_type = bar_type
        self.title = title
        self.with_setting = with_setting
        
        # 상태 바 위젯
        self.back_button = None
        self.title_label = None
        self.dentium_label = None
        self.wifi_icon = None
        self.setting_icon = None
        self.battery_icon = None
        
        # UI 초기화
        self._setup_ui()
    
    def _setup_ui(self):
        """UI 초기화"""
        # 기본 속성 설정
        self.setFixedHeight(70)
        self.setStyleSheet("background-color: #FFFFFF;")
        
        # 레이아웃 설정
        self.status_layout = QHBoxLayout(self)
        self.status_layout.setContentsMargins(10, 0, 20, 0)
        
        # 상태 바 타입에 따라 UI 구성
        if self.bar_type == StatusBarType.WITH_BACK:
            # 뒤로가기 버튼 생성 및 추가
            self.back_button = self._create_back_button()
            self.status_layout.addWidget(self.back_button)
            
            # 타이틀 레이블 생성 및 추가
            self.title_label = self._create_title_label(self.title)
            self.status_layout.addWidget(self.title_label)
        elif self.bar_type == StatusBarType.TITLE_SETTING:
            # 타이틀 레이블 생성 및 추가
            self.title_label = self._create_title_label(self.title)
            self.status_layout.addWidget(self.title_label)
        else:
            # 로고 레이블 생성 및 추가
            self.dentium_label = self._create_logo_label()
            self.status_layout.addWidget(self.dentium_label)
        
        # 스페이서 추가 (오른쪽 정렬용)
        self.status_layout.addStretch()
        
        # 아이콘 추가
        self._add_icons()
    
    def _create_back_button(self):
        """뒤로가기 버튼 생성 및 설정
        
        Returns:
            QPushButton: 생성된 뒤로가기 버튼
        """
        back_button = QPushButton()
        back_button.setFixedSize(40, 40)
        back_button.setStyleSheet("""
            QPushButton {
                background-color: transparent;
                border: none;
                border-radius: 20px;
            }
        """)
        
        # 뒤로가기 아이콘 추가
        back_icon = QPixmap(":file/Back_Icon.png")
        back_button.setIcon(back_icon)
        back_button.setIconSize(back_icon.size())
        
        # 클릭 이벤트 연결
        back_button.clicked.connect(self._on_back_clicked)
        back_button.pressed.connect(self._on_back_clicked)
        
        
        return back_button
    
    def _create_title_label(self, text):
        """타이틀 레이블 생성
        
        Args:
            text (str): 표시할 타이틀 텍스트
            
        Returns:
            QLabel: 생성된 타이틀 레이블
        """
        title_label = QLabel(text)
        title_label.setFont(QFont('Arial', 20))
        title_label.setStyleSheet("color: white;")
        return title_label
    
    def _create_logo_label(self):
        """로고 레이블 생성
        
        Returns:
            QLabel: 생성된 로고 레이블
        """
        logo_label = QLabel("Dentium")
        logo_label.setFont(QFont('Arial', 24, QFont.Bold))
        logo_label.setStyleSheet("color: white;")
        return logo_label
    
    def _add_icons(self):
        """아이콘 추가"""
        icon_size = 48
        
        # 상태 바 타입에 따라 아이콘 추가
        if self.bar_type == StatusBarType.FULL:
            # Wi-Fi 아이콘
            self.wifi_icon = self._create_wifi_icon(icon_size)
            self.status_layout.addWidget(self.wifi_icon)
            
            # 아이콘 사이 간격
            self.status_layout.addWidget(self._create_spacer(15))
            
            # 설정 아이콘
            self.setting_icon = self._create_setting_icon(icon_size)
            self.status_layout.addWidget(self.setting_icon)
            
            # 아이콘 사이 간격
            self.status_layout.addWidget(self._create_spacer(15))
            
            # 배터리 아이콘 (FULL 타입에서는 크기 축소)
            self.battery_icon = self._create_battery_icon(icon_size - 12)  # 36x36 크기로 축소
            self.status_layout.addWidget(self.battery_icon)
        elif self.bar_type == StatusBarType.SIMPLE and self.with_setting:
            # 설정 아이콘
            self.setting_icon = self._create_setting_icon(icon_size)
            self.status_layout.addWidget(self.setting_icon)
            
            # 아이콘 사이 간격
            self.status_layout.addWidget(self._create_spacer(15))
            
            # 배터리 아이콘
            self.battery_icon = self._create_battery_icon(icon_size)
            self.status_layout.addWidget(self.battery_icon)
        elif self.bar_type == StatusBarType.TITLE_SETTING:
            # 설정 아이콘
            self.setting_icon = self._create_setting_icon(icon_size)
            self.status_layout.addWidget(self.setting_icon)
            
            # 아이콘 사이 간격
            self.status_layout.addWidget(self._create_spacer(15))
            
            # 배터리 아이콘
            self.battery_icon = self._create_battery_icon(icon_size)
            self.status_layout.addWidget(self.battery_icon)
        else:
            # 설정 아이콘이 없는 타입에서는 더미 아이콘 생성
            self.setting_icon = QLabel()
            
            # 배터리 아이콘
            self.battery_icon = self._create_battery_icon(icon_size)
            self.status_layout.addWidget(self.battery_icon)
    
    def _create_spacer(self, width):
        """간격 생성
        
        Args:
            width (int): 간격 너비
            
        Returns:
            QLabel: 간격으로 사용할 레이블
        """
        spacer = QLabel()
        spacer.setFixedWidth(width)
        return spacer
    
    def _create_setting_icon(self, size):
        """설정 아이콘 생성 및 설정
        
        Args:
            size (int): 아이콘 크기
            
        Returns:
            QLabel: 설정 아이콘 레이블
        """
        # 기본 아이콘 생성
        # Qt 리소스 시스템을 사용합니다.
        icon = QLabel()
        pixmap = QPixmap(":file/Setting_Icon.png")
        
        # QPixmap 로드 확인
        if pixmap.isNull():
            print("[StatusBar] 설정 아이콘을 불러올 수 없습니다.")
            icon.setText("설정")
            icon.setStyleSheet("color: white; font-size: 14px; background-color: transparent;")
        else:
            icon.setPixmap(pixmap.scaled(size, size))
            icon.setStyleSheet("background-color: transparent;")
            # 클릭 가능하도록 설정
            icon.setCursor(QCursor(Qt.PointingHandCursor))
            icon.mousePressEvent = self._on_setting_clicked
            
            # 호버 효과 추가
            icon.setStyleSheet("""
                QLabel {
                    background-color: transparent;
                }
                QLabel:hover {
                    background-color: rgba(255, 255, 255, 0.2);
                    border-radius: 20px;
                }
            """)
        
        return icon
    
    def _create_battery_icon(self, size):
        """배터리 아이콘 생성 및 설정
        
        Args:
            size (int): 아이콘 크기
            
        Returns:
            QLabel: 배터리 아이콘 레이블
        """
        # 기본 아이콘 생성
        icon = QLabel()
        
        # 아이콘 이미지 로드
        pixmap = QPixmap(":/file/Battery/battery_50.png")
        if pixmap.isNull():
            #print("[StatusBar] 배터리 아이콘을 불러올 수 없습니다.")
            icon.setText("Battery")
            icon.setStyleSheet("""
                color: white; 
                font-size: 14px;
                background-color: transparent;
            """)
        else:
            icon.setPixmap(pixmap.scaled(size, size))
            icon.setStyleSheet("""
                background-color: transparent;
            """)
        
        return icon
    
    def _create_wifi_icon(self, size):
        """와이파이 아이콘 생성 및 설정
        
        Args:
            size (int): 아이콘 크기
            
        Returns:
            QLabel: 와이파이 아이콘 레이블
        """
        # 기본 아이콘 생성
        icon = QLabel()
        
        # 아이콘 이미지 로드
        pixmap = QPixmap(":/file/Wifi_Icon.png")
        if pixmap.isNull():
            print("[StatusBar] 와이파이 아이콘을 불러올 수 없습니다.")
            icon.setText("WiFi")
            icon.setStyleSheet("""
                color: white; 
                font-size: 14px;
                background-color: transparent;
            """)
        else:
            icon.setPixmap(pixmap.scaled(size, size))
            icon.setStyleSheet("""
                background-color: transparent;
            """)
        
        # 클릭 가능하도록 설정
        icon.setCursor(QCursor(Qt.PointingHandCursor))
        icon.mousePressEvent = self._on_wifi_clicked
        
        return icon
    
    # 뒤로가기 버튼 클릭 이벤트 핸들러
    def _on_back_clicked(self):
        self.back_signal.emit()
    
    # 설정 아이콘 클릭 이벤트 핸들러
    def _on_setting_clicked(self, event):
        print("[StatusBar] 설정 아이콘 클릭됨")
        self.setting_clicked.emit()
    
    # 와이파이 아이콘 클릭 이벤트 핸들러
    def _on_wifi_clicked(self, event):
        print("[StatusBar] 와이파이 아이콘 클릭됨")
        self.wifi_clicked.emit()
    
    # 타이틀 설정
    def set_title(self, title):
        """ Args:
        title (str): 설정할 타이틀 텍스트
        """
        if self.title_label:
            self.title_label.setText(title)
            self.title = title
    
    def get_back_button(self):
        """뒤로가기 버튼 반환
        
        Returns:
            QPushButton or None: 뒤로가기 버튼 (없으면 None)
        """
        return self.back_button
    
    # 설정 아이콘 반환
    def get_setting_icon(self):
        """설정 아이콘 반환
        
        Returns:
            QLabel: 설정 아이콘 (또는 더미 레이블)
        """
        if hasattr(self, 'setting_icon'):
            return self.setting_icon
        return QLabel()  # 설정 아이콘이 없는 경우 더미 레이블 반환 
        
    def update_wifi_status(self, enabled):
        """와이파이 상태 업데이트
        
        Args:
            enabled (bool): 와이파이 활성화 여부
        """
        if hasattr(self, 'wifi_icon') and self.wifi_icon:
            if enabled:
                icon_path = ":/file/Wifi_Icon.png"
            else:
                icon_path = ":/file/Wifi_Off_Icon.png"
                
            pixmap = QPixmap(":/file/" + icon_path)
            if pixmap.isNull():
                print(f"[StatusBar] 와이파이 아이콘을 불러올 수 없습니다: {icon_path}")
                # 대체 텍스트 사용
                if enabled:
                    self.wifi_icon.setText("WiFi")
                else:
                    self.wifi_icon.setText("WiFi X")
                
                # 텍스트 스타일 설정
                self.wifi_icon.setStyleSheet("""
                    color: white; 
                    font-size: 14px;
                    background-color: transparent;
                """)
            else:
                # 기존 텍스트 제거하고 이미지로 설정
                self.wifi_icon.setText("")
                self.wifi_icon.setPixmap(pixmap.scaled(48, 48))
                self.wifi_icon.setStyleSheet("""
                    background-color: transparent;
                """)
                
            # 클릭 이벤트 재설정 (상태가 변경되어도 클릭 가능하도록)
            self.wifi_icon.mousePressEvent = self._on_wifi_clicked 

    def update_battery_status(self, percentage):
        """배터리 상태 업데이트
        
        Args:
            percentage (float): 배터리 잔량 (0-100)
        """
        if not hasattr(self, 'battery_icon') or not self.battery_icon:
            return
            
        try:
            # BatteryRenderer 사용하여 배터리 이미지 생성
            from Utils.BatteryRenderer import BatteryRenderer
            
            # 배터리 렌더러 생성 또는 재사용
            if not hasattr(self, 'battery_renderer'):
                self.battery_renderer = BatteryRenderer()
                
            # 배터리 이미지 렌더링
            battery_pixmap = self.battery_renderer.get_battery_pixmap(int(percentage))
            
            # 이미지 설정
            if not battery_pixmap.isNull():
                self.battery_icon.setText("")  # 기존 텍스트 제거
                self.battery_icon.setPixmap(battery_pixmap.scaled(48, 48))
                self.battery_icon.setStyleSheet("""
                    background-color: transparent;
                """)
                return
        except Exception as e:
            print(f"[StatusBar] 배터리 렌더러 사용 중 오류 발생: {str(e)}")
            
        # 렌더러 사용 실패 시 기존 방식으로 폴백
        # 배터리 레벨에 따른 아이콘 선택
        if 0 <= percentage <= 10:
            icon_path = ":/file/Battery/battery_0to10.png"
        elif 11 <= percentage <= 20:
            icon_path = ":/file/Battery/battery_11to19.png"
        elif 21 <= percentage <= 40:
            icon_path = ":/file/Battery/battery_20.png"
        elif 41 <= percentage <= 60:
            icon_path = ":/file/Battery/battery_40.png"
        elif 61 <= percentage <= 80:
            icon_path = ":/file/Battery/battery_60.png"
        elif 81 <= percentage <= 99:
            icon_path = ":/file/Battery/battery_80.png"
        elif percentage == 100:
            icon_path = ":/file/Battery/battery_Full.png"
        else:
            icon_path = ":/file/Battery/battery_0to10.png"  # 기본값

        # 아이콘 업데이트
        pixmap = QPixmap(icon_path)
        if pixmap.isNull():
            print(f"[StatusBar] 배터리 아이콘을 불러올 수 없습니다: {icon_path}")
            self.battery_icon.setText(f"{int(percentage)}%")
            self.battery_icon.setStyleSheet("""
                color: white; 
                font-size: 14px;
                background-color: transparent;
            """)
        else:
            self.battery_icon.setText("")  # 기존 텍스트 제거
            self.battery_icon.setPixmap(pixmap.scaled(48, 48))
            self.battery_icon.setStyleSheet("""
                background-color: transparent;
            """) 