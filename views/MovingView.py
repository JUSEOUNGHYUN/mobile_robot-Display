from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QHBoxLayout
from PySide6.QtGui import QFont
from PySide6.QtCore import Qt, Signal, QTimer, QFile, QDir
from PySide6.QtUiTools import QUiLoader
from views.StatusBar import StatusBar, StatusBarType
import os

class MovingView(QWidget):
    """이동 중 화면"""
    # 일시 정지 신호
    pause_signal = Signal(dict)  # 현재 목적지 정보 전달

    def __init__(self, parent=None, destination=None):
        super().__init__(parent)

        # UI 파일 로드
        loader = QUiLoader()
        ui_file_path = os.path.join(os.path.dirname(__file__), "..", "UI", "MovingView.ui")
        ui_file = QFile(ui_file_path)
        ui_file.open(QFile.ReadOnly)
        self.ui = loader.load(ui_file, self)
        ui_file.close()

        # 메인 윈도우 설정
        self.setWindowTitle('Dentium')
        self.setFixedSize(1024, 600)

        # 레이아웃 설정
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # 목적지 정보 저장
        self.destination = destination or {}
        self.destination_name = self.destination.get('name', '알 수 없음')

        # 상태바 설정
        self.status_bar = StatusBar(self, StatusBarType.SIMPLE)

        # 상태바 컨테이너에 상태바 추가
        placeholder_layout = QVBoxLayout(self.ui.status_bar_container)
        placeholder_layout.setContentsMargins(0, 0, 0, 0)
        placeholder_layout.addWidget(self.status_bar)

        # UI 위젯을 메인 레이아웃에 추가
        main_layout.addWidget(self.ui)

        # 레이블 참조
        self.name_label = self.ui.findChild(QLabel, "name_label")
        self.moving_label = self.ui.findChild(QLabel, "moving_label")

        # 목적지 이름 업데이트
        if self.name_label:
            self.name_label.setText(self.destination_name)

        # 전체 화면을 클릭 가능하도록 설정
        self.setMouseTracking(True)

    def mousePressEvent(self, event):
        """화면 터치 시 일시정지 신호 발생"""
        if self.destination:
            print(f"[MovingView] 화면 터치: 일시 정지 시그널 발생 (목적지: {self.destination.get('name', '알 수 없음')})")
        else:
            print("[MovingView] 화면 터치: 일시 정지 시그널 발생 (목적지 정보 없음)")
        self.pause_signal.emit(self.destination)

    def update_destination(self, destination):
        """목적지 정보 업데이트"""
        if not destination:
            print("[MovingView] 경고: 빈 목적지 데이터가 전달되었습니다.")
            return
            
        print(f"[MovingView] 목적지 정보 업데이트: {destination.get('name', '이름 없음')}")
        
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
                print(f"[MovingView] 목적지 정보 복사 중 오류: {str(e)}")
                self.destination = destination  # 오류 시 참조 복사라도 수행
        
        # 목적지 이름 업데이트
        if isinstance(destination, dict) and 'name' in destination:
            self.destination_name = destination['name']
            if self.name_label:
                self.name_label.setText(self.destination_name)
                
            # 디버그용 목적지 정보 출력
            position = destination.get('position', {})
            print(f"[MovingView] 저장된 목적지: {self.destination_name}, "
                  f"X={position.get('x', 'None')}, "
                  f"Y={position.get('y', 'None')}, "
                  f"Yaw={position.get('yaw', 'None')}")