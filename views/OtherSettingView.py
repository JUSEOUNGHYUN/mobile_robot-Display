from PySide6.QtWidgets import QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout
from PySide6.QtGui import QPixmap, QFont
from PySide6.QtCore import Qt, Signal, QFile, QDir
from PySide6.QtUiTools import QUiLoader
from views.StatusBar import StatusBar, StatusBarType
from Utils.ButtonStyle import RoundButtonStyle
import os

class OtherSettingView(QWidget):
    # 시그널 정의
    back_signal = Signal()
    exit_signal = Signal()
    area_index_view_changed = Signal(bool)  # 영역 인덱스 표시 여부 변경 시그널

    def __init__(self, parent=None):
        super().__init__(parent)

        # UI 파일 로드
        loader = QUiLoader()

        ui_file_path = os.path.join(os.path.dirname(__file__), "..", "UI", "OtherSettingView.ui")
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
        self.status_bar = StatusBar(self, StatusBarType.WITH_BACK, "기타 설정")
        self.status_bar.back_signal.connect(self.go_back)  # 뒤로가기 버튼 시그널 연결

        # status_bar_placeholder에 상태바 추가
        placeholder_layout = QVBoxLayout(self.ui.status_bar_placeholder)
        placeholder_layout.setContentsMargins(0, 0, 0, 0)
        placeholder_layout.addWidget(self.status_bar)

        # UI 위젯을 메인 레이아웃에 추가
        main_layout.addWidget(self.ui)

        # 버튼에 대한 시그널 연결
        self.ui.area_index_view_cb.stateChanged.connect(self.on_area_index_view_changed)
        
        # 디버그용 초기 체크박스 상태 출력
        initial_state = self.ui.area_index_view_cb.isChecked()
        print(f"[OtherSettingView] 초기 체크박스 상태: {initial_state}")

    # 프로그램 종료 버튼 클릭 시 호출되는 메서드
    def on_exit_clicked(self):
        self.exit_program()

    # 프로그램 종료
    def exit_program(self):
        self.exit_signal.emit()

    # 뒤로가기 버튼 클릭 시 호출되는 메서드
    def go_back(self):
        self.back_signal.emit()

    # 영역 인덱스 표시 체크박스 상태 변경 시 호출되는 메서드
    def on_area_index_view_changed(self, state):
        is_checked = (state == Qt.Checked)
        print(f"[OtherSettingView] 체크박스 상태 변경 감지: state={state}, Qt.Checked={Qt.Checked}, is_checked={is_checked}")
        self.area_index_view_changed.emit(is_checked)
        print(f"[OtherSettingView] 영역 인덱스 표시 설정 변경 시그널 발생: {is_checked}") 