import os
from PySide6.QtWidgets import QWidget, QVBoxLayout
from PySide6.QtCore import Signal, QFile
from PySide6.QtUiTools import QUiLoader
from views.StatusBar import StatusBar, StatusBarType

class DeveloperModeView(QWidget):
    # 시그널 정의
    back_signal = Signal()
    exit_signal = Signal()
    monitoring_signal = Signal()
    system_check_signal = Signal()
    check_log_signal = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)

        # UI 파일 로드
        loader = QUiLoader()
        ui_file_path = os.path.join(os.path.dirname(__file__), "..", "UI", "DeveloperModeView.ui")
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
        self.status_bar = StatusBar(self, StatusBarType.WITH_BACK, "개발자 모드")
        self.status_bar.back_signal.connect(self.go_back)  # 뒤로가기 버튼 시그널 연결

        # status_bar_placeholder에 상태바 추가
        placeholder_layout = QVBoxLayout(self.ui.status_bar_placeholder)
        placeholder_layout.setContentsMargins(0, 0, 0, 0)
        placeholder_layout.addWidget(self.status_bar)

        # UI 위젯을 메인 레이아웃에 추가
        main_layout.addWidget(self.ui)

        # 버튼에 대한 시그널 연결
        self.ui.exit_button.clicked.connect(self.on_exit_clicked)
        self.ui.monitoring_button.clicked.connect(self.on_monitoring_clicked)
        self.ui.system_check_button.clicked.connect(self.on_system_check_clicked)
        self.ui.check_log_button.clicked.connect(self.on_check_log_clicked)

    # 모니터링 버튼 클릭 시 호출되는 메서드
    def on_monitoring_clicked(self):
        self.monitoring_signal.emit()

    # 시스템 체크 버튼 클릭 시 호출되는 메서드
    def on_system_check_clicked(self):
        self.system_check_signal.emit()

    # 로그 확인 버튼 클릭 시 호출되는 메서드
    def on_check_log_clicked(self):
        self.check_log_signal.emit()

    # 프로그램 종료 버튼 클릭 시 호출되는 메서드
    def on_exit_clicked(self):
        self.exit_program()

    # 프로그램 종료
    def exit_program(self):
        self.exit_signal.emit()

    # 뒤로가기 버튼 클릭 시 호출되는 메서드
    def go_back(self):
        self.back_signal.emit()