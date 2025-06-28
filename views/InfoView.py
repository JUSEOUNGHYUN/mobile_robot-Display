import os
from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QHBoxLayout, QScrollArea, QPushButton, QScroller
from PySide6.QtGui import QFont
from PySide6.QtCore import Qt, Signal
from PySide6.QtUiTools import QUiLoader
from views.StatusBar import StatusBar, StatusBarType
import os

class InfoView(QWidget):
    """정보 화면

    애플리케이션의 기술적 정보와 심볼릭 링크 관련 정보를 표시하는 화면입니다.
    """

    # 시그널 정의
    back_signal = Signal()  # 뒤로가기 시그널
    check_symlinks_signal = Signal()  # 심볼릭 링크 확인 버튼 클릭 시그널

    def __init__(self, parent=None):
        super().__init__(parent)
        
        # UI 파일 로드
        self.setup_ui()

        # 상태바 추가 (설정 버튼 없는 WITH_BACK 타입)
        self.status_bar = StatusBar(self.ui.status_bar_container, StatusBarType.WITH_BACK, "정보")
        self.status_bar.back_signal.connect(self.go_back)  # 뒤로가기 버튼 시그널 연결
        
        # 스크롤 영역 설정
        self.scroll_area = self.ui.scroll_area

        # 터치 스크롤 활성화
        QScroller.grabGesture(self.scroll_area.viewport(), QScroller.LeftMouseButtonGesture)

        # 버튼 연결
        self.ui.check_symlink_button.clicked.connect(self.on_check_symlinks)
        self.ui.check_symlink_button.pressed.connect(self.on_check_symlinks)  # 터치스크린용
        
        # 레이블 참조
        self.version_info = self.ui.version_info_label
        self.symlink_info = self.ui.symlink_info_label
        self.dev_info = self.ui.dev_info_label
        self.tech_stack_container = self.ui.tech_stack_layout
    
    def setup_ui(self):
        """UI 초기화"""
        loader = QUiLoader()
        ui_file_path = os.path.join(os.path.dirname(__file__), "..", "UI", "InfoView.ui")
        self.ui = loader.load(ui_file_path)
        
        # 메인 레이아웃 설정
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # UI 파일에서 로드한 위젯 추가
        layout.addWidget(self.ui)
    
    def on_check_symlinks(self):
        """심볼릭 링크 확인 버튼 클릭 시 호출"""
        self.check_symlinks_signal.emit()
    
    def go_back(self):
        """뒤로가기 버튼 클릭 시 호출되는 메서드"""
        self.back_signal.emit()
        
    def set_version_info(self, version):
        """버전 정보 설정"""
        self.version_info.setText(version)
        
    def set_tech_stack(self, tech_items):
        """기술 스택 정보 설정"""
        # 기존 항목 제거
        for i in reversed(range(self.tech_stack_container.count())):
            item = self.tech_stack_container.itemAt(i)
            if item.widget():
                item.widget().deleteLater()
        
        # 새 항목 추가
        for item in tech_items:
            label = QLabel(item)
            label.setFont(QFont('Arial', 16))
            label.setStyleSheet("color: white;")
            label.setWordWrap(True)
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.tech_stack_container.addWidget(label)
            
    def set_developer_info(self, developer):
        """개발자 정보 설정"""
        self.dev_info.setText(developer)
        
    def update_symlink_info(self, symlink_info):
        """심볼릭 링크 정보 업데이트"""
        self.symlink_info.setText("\n".join(symlink_info))