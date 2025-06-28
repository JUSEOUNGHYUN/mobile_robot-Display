from PySide6.QtWidgets import QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout
from PySide6.QtGui import QFont
from PySide6.QtCore import Qt, Signal, QTimer, QFile, QDir
from PySide6.QtUiTools import QUiLoader
from views.StatusBar import StatusBar, StatusBarType
import os

class PauseView(QWidget):
    """이동 일시정지 화면"""
    # 시그널 정의
    resume_signal = Signal(dict)    # 이동 계속 (목적지 정보 포함)
    cancel_signal = Signal()        # 이동 취소
    setting_signal = Signal()       # 설정 화면으로 이동

    def __init__(self, parent=None, destination=None):
        super().__init__(parent)

        # UI 파일 로드
        loader = QUiLoader()
        ui_file_path = os.path.join(os.path.dirname(__file__), "..", "UI", "PauseView.ui")
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

        # 목적지 정보 저장
        self.destination = destination or {}
        self.destination_name = self.destination.get('name', '알 수 없음')

        # 카운트다운 설정
        self.countdown_seconds = 30
        self.timer = QTimer(self)
        self.timer.setInterval(1000)  # 1초마다
        self.timer.timeout.connect(self.update_countdown)

        # 상태바 추가 (설정 아이콘 포함)
        self.status_bar = StatusBar(self, StatusBarType.FULL, with_setting=True)
        self.status_bar.setting_clicked.connect(self.on_setting_clicked)

        # status_bar_placeholder에 상태바 추가
        placeholder_layout = QVBoxLayout(self.ui.status_bar_placeholder)
        placeholder_layout.setContentsMargins(0, 0, 0, 0)
        placeholder_layout.addWidget(self.status_bar)

        # UI 위젯을 메인 레이아웃에 추가
        main_layout.addWidget(self.ui)

        # 레이블 및 버튼 참조
        self.countdown_label = self.ui.findChild(QLabel, "countdown_label")
        self.cancel_all_button = self.ui.findChild(QPushButton, "cancel_all_button")
        self.return_to_start_button = self.ui.findChild(QPushButton, "return_to_start_button")

        # 버튼에 대한 시그널 연결
        self.cancel_all_button.clicked.connect(self.on_cancel)
        self.return_to_start_button.clicked.connect(self.on_return_to_start)

        # 마우스 이벤트 트래킹 활성화
        self.setMouseTracking(True)

    def showEvent(self, event):
        """화면이 표시될 때 타이머 시작"""
        super().showEvent(event)
        self.countdown_seconds = 30
        self.countdown_label.setText(f"{self.countdown_seconds}초")
        self.timer.start()

    def hideEvent(self, event):
        """화면이 숨겨질 때 타이머 중지"""
        super().hideEvent(event)
        self.timer.stop()
    
    def update_countdown(self):
        """카운트다운 업데이트"""
        self.countdown_seconds -= 1
        self.countdown_label.setText(f"{self.countdown_seconds}초")
        
        if self.countdown_seconds <= 0:
            self.timer.stop()
            self.resume_signal.emit(self.destination)
    
    def mousePressEvent(self, event):
        """화면 터치 시 이동 계속"""
        self.timer.stop()
        self.resume_signal.emit(self.destination)
    
    def on_return_to_start(self):
        """출발지로 보내기 버튼 클릭 시"""
        # 아직 기능만 추가 (추후 구현)
        self.timer.stop()
        self.cancel_signal.emit()
    
    def on_cancel(self):
        """모든 이동 취소 버튼 클릭 시"""
        self.timer.stop()
        self.cancel_signal.emit()
    
    def update_destination(self, destination):
        """목적지 정보 업데이트"""
        if not destination:
            print("[PauseView] 경고: 빈 목적지 데이터가 전달되었습니다.")
            return
            
        print(f"[PauseView] 목적지 정보 업데이트: {destination.get('name', '이름 없음')}")
        
        # 목적지 정보 깊은 복사
        try:
            import copy
            self.destination = copy.deepcopy(destination)
        except:
            # 복사 실패 시 직접 복사 시도
            try:
                self.destination = {}
                for key, value in destination.items():
                    if key == 'position' and isinstance(value, dict):
                        self.destination['position'] = {
                            'x': float(value.get('x', 0.0)),
                            'y': float(value.get('y', 0.0)),
                            'yaw': float(value.get('yaw', 0.0))
                        }
                    else:
                        self.destination[key] = value
            except Exception as e:
                print(f"[PauseView] 목적지 정보 복사 중 오류: {str(e)}")
                self.destination = destination  # 오류 시 참조 복사라도 수행
        
        # 목적지 이름 업데이트
        if isinstance(destination, dict) and 'name' in destination:
            self.destination_name = destination['name']
            # 디버그용 목적지 정보 출력
            position = destination.get('position', {})
            print(f"[PauseView] 저장된 목적지: {self.destination_name}, "
                  f"X={position.get('x', 'None')}, "
                  f"Y={position.get('y', 'None')}, "
                  f"Yaw={position.get('yaw', 'None')}")
            
            # UI 업데이트 (목적지 이름 레이블이 있는 경우)
            if hasattr(self.ui, 'destination_name_label'):
                self.ui.destination_name_label.setText(self.destination_name)
    
    def on_setting_clicked(self):
        """설정 버튼 클릭 시 호출"""
        self.timer.stop()  # 타이머 중지
        self.setting_signal.emit()