from PySide6.QtWidgets import (
    QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QListWidget, QListWidgetItem, QFrame, QLineEdit, QDialog,
    QScrollArea
)
from PySide6.QtGui import QPixmap, QFont, QIcon
from PySide6.QtCore import Qt, Signal, QSize, QFile
from PySide6.QtUiTools import QUiLoader
from views.StatusBar import StatusBar, StatusBarType
from Utils.MessageBox import MessageBox
from Utils.PasswordDialog import PasswordDialog
import os

class WiFiNetworkItem(QFrame):
    """와이파이 네트워크 아이템 위젯"""

    clicked = Signal(str)  # 네트워크 SSID

    def __init__(self, network, parent=None):
        super().__init__(parent)
        self.ssid = network.get("ssid", "")
        self.bssid = network.get("bssid", "")
        self.is_connected = network.get("connected", False)
        self.is_secured = network.get("secured", False)
        self.signal_strength = network.get("strength", 0)
        self.channel = network.get("channel", 0)
        self.rate = network.get("rate", "")
        self.bars = network.get("bars", "")
        self.security = network.get("security", "")

        # 스타일 설정 (연결된 네트워크는 더 눈에 띄게)
        bg_color = "#222222"
        if self.is_connected:
            bg_color = "#333333"

        self.setStyleSheet(f"""
            QFrame {{
                background-color: {bg_color};
                border-radius: 8px;
                padding: 3px;
            }}
            QFrame:hover {{
                background-color: #3A3A3A;
            }}
        """)

        # 최소 높이 설정 (연결된 네트워크는 더 큰 높이)
        self.setFixedHeight(55 if self.is_connected else 45)

        # 레이아웃 설정
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 3, 15, 3)
        layout.setSpacing(10)

        # WiFi 아이콘 (신호 강도 표시)
        icon_label = QLabel()
        icon_pixmap = self._get_signal_icon(self.signal_strength)
        icon_label.setPixmap(icon_pixmap)
        icon_label.setFixedSize(24, 24)
        layout.addWidget(icon_label)

        # 네트워크 정보 영역
        info_layout = QVBoxLayout()
        info_layout.setSpacing(2)

        # SSID 레이블
        self.ssid_label = QLabel(self.ssid)
        self.ssid_label.setFont(QFont('Arial', 14, QFont.Bold if self.is_connected else QFont.Normal))
        self.ssid_label.setStyleSheet("color: white;")
        info_layout.addWidget(self.ssid_label)

        # 연결 상태 표시 (연결된 경우만)
        if self.is_connected:
            status_layout = QHBoxLayout()
            status_layout.setSpacing(3)
            status_layout.setContentsMargins(0, 0, 0, 0)

            # 연결됨 상태 표시
            connected_label = QLabel("연결됨")
            connected_label.setFont(QFont('Arial', 11))
            connected_label.setStyleSheet("color: #9E9E9E;")
            status_layout.addWidget(connected_label)

            # BSSID와 세부 정보
            details_text = f"{self.bssid} | 채널 {self.channel} | {self.rate} | {self.security}"
            details_label = QLabel(details_text)
            details_label.setFont(QFont('Arial', 11))
            details_label.setStyleSheet("color: #9E9E9E;")
            details_label.setWordWrap(True)
            details_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            status_layout.addWidget(details_label, 1)
            
            info_layout.addLayout(status_layout)
        else:
            # 미연결 상태 - MAC 주소와 세부 정보
            status_text = ""
            if self.bssid:
                status_text += self.bssid + " | "
            if self.channel:
                status_text += f"채널 {self.channel} | "
            if self.rate:
                status_text += f"{self.rate} | "
            if self.is_secured:
                status_text += self.security
        
        layout.addLayout(info_layout, 1)
        
        # 신호 강도 막대
        if self.bars:
            bars_label = QLabel(self.bars)
            bars_label.setFont(QFont('Monospace', 14))
            bars_label.setFixedWidth(60)
            bars_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            
            # 신호 강도에 따른 색상
            if self.signal_strength >= 70:
                bars_label.setStyleSheet("color: #4CAF50;")  # 강함 - 녹색
            elif self.signal_strength >= 40:
                bars_label.setStyleSheet("color: #FFC107;")  # 중간 - 노란색
            else:
                bars_label.setStyleSheet("color: #F44336;")  # 약함 - 빨간색
                
            layout.addWidget(bars_label)
            
        # 클릭 이벤트
        self.mousePressEvent = self.on_click
        
    def on_click(self, event):
        self.clicked.emit(self.ssid)
        
    def _get_signal_icon(self, strength):
        """신호 강도에 따른 아이콘 반환"""
        # 신호 강도에 따라 다른 아이콘 사용 가능
        icon_path = "resources/Wifi_Icon.png"
        pixmap = QPixmap(":file/" + icon_path)
        if pixmap.isNull():
            print(f"[WiFiView] 와이파이 아이콘을 불러올 수 없습니다: {icon_path}")
            # 더미 픽스맵 생성
            return QPixmap(30, 30)
        return pixmap.scaled(30, 30)

class WiFiView(QWidget):
    """와이파이 설정 화면
    
    와이파이 네트워크 목록을 보고 연결할 수 있는 화면입니다.
    """
    
    # 시그널 정의
    back_signal = Signal()
    scan_signal = Signal()
    connect_signal = Signal(str, str)  # SSID, 비밀번호
    disconnect_signal = Signal(str)    # SSID
    toggle_wifi_signal = Signal(bool)  # 와이파이 활성화 여부
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # UI 파일 로드
        loader = QUiLoader()
        ui_file_path = os.path.join(os.path.dirname(__file__), "..", "UI", "WiFiView.ui")
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
        
        # 상태바 추가 (StatusBar WITH_BACK 타입 사용)
        self.status_bar = StatusBar(self, StatusBarType.WITH_BACK, "와이파이 설정")
        self.status_bar.back_signal.connect(self.go_back)
        
        # status_bar_placeholder에 상태바 추가
        placeholder_layout = QVBoxLayout(self.ui.status_bar_placeholder)
        placeholder_layout.setContentsMargins(0, 0, 0, 0)
        placeholder_layout.addWidget(self.status_bar)
        
        # UI 위젯을 메인 레이아웃에 추가
        main_layout.addWidget(self.ui)
        
        # 버튼에 대한 시그널 연결
        self.ui.wifi_switch_button.clicked.connect(self.on_wifi_toggle_clicked)
        self.ui.scan_button.clicked.connect(self.on_scan_clicked)
        
        # 와이파이 활성화 상태
        self.wifi_enabled = True
        
    def go_back(self):
        """뒤로가기 버튼 클릭 처리"""
        self.back_signal.emit()
    
    def on_scan_clicked(self):
        """스캔 버튼 클릭 처리"""
        self.scan_signal.emit()
    
    def on_wifi_toggle_clicked(self):
        """와이파이 토글 버튼 클릭 처리"""
        # 현재 상태 반전
        new_state = not self.wifi_enabled
        # 시그널 발생
        self.toggle_wifi_signal.emit(new_state)
    
    def on_network_clicked(self, ssid):
        """네트워크 아이템 클릭 처리
        
        Args:
            ssid (str): 클릭된 네트워크의 SSID
        """
        # 현재 연결 상태 확인
        is_connected = False
        is_secured = False
        for i in range(self.ui.network_list.count()):
            item = self.ui.network_list.item(i)
            widget = self.ui.network_list.itemWidget(item)
            if widget.ssid == ssid:
                is_connected = widget.is_connected
                is_secured = widget.is_secured
                break
        
        if is_connected:
            # 연결 해제 확인
            result = MessageBox.show_message(
                self,
                title="연결 해제",
                message=f"{ssid}에서 연결을 해제하시겠습니까?",
                button_labels=["예", "아니오"]
            )
            
            if result == MessageBox.YES:
                self.disconnect_signal.emit(ssid)
        else:
            if is_secured:
                # 비밀번호 입력 다이얼로그 표시
                password, ok = PasswordDialog.show_dialog(
                    self, 
                    title="와이파이 연결", 
                    message=f"{ssid} 네트워크에 연결하려면 비밀번호를 입력하세요."
                )
                
                # 입력 확인 후 연결 시도
                if ok and password:
                    self.connect_signal.emit(ssid, password)
            else:
                # 보안되지 않은 네트워크에 직접 연결
                self.connect_signal.emit(ssid, "")