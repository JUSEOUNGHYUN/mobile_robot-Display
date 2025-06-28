from PySide6.QtWidgets import QWidget, QVBoxLayout, QButtonGroup, QSlider, QTextEdit, QScrollArea, QFrame
from PySide6.QtCore import Qt, Signal, QFile, QPoint, QPropertyAnimation, QEasingCurve, QEvent
from PySide6.QtUiTools import QUiLoader
from PySide6.QtGui import QMouseEvent, QTouchEvent
from views.StatusBar import StatusBar, StatusBarType
import os

class MoveOptionView(QWidget):
    """이동 옵션 설정 화면 뷰"""
    
    # 시그널 정의
    back_signal = Signal()  # 뒤로가기
    save_signal = Signal(bool, int, int, bool, float, float)  # 저장 (매개변수: 무시 여부, 가속도, 감속도, 브레이크 타입, 직진 속도, 회전 속도)
    
    def __init__(self, parent=None):
        """초기화"""
        super().__init__(parent)
        
        # 드래그 관련 변수 초기화
        self.touch_start_pos = None
        self.last_touch_pos = None
        self.is_touching = False
        self.scroll_animation = None
        self.scroll_content = None
        self.scroll_area = None
        self.drag_threshold = 10  # 드래그로 인식할 최소 이동 거리
        self.is_dragging = False  # 드래그 중인지 여부
        
        # UI 파일 로드
        loader = QUiLoader()
        ui_file_path = os.path.join(os.path.dirname(__file__), "..", "UI", "MoveOptionView.ui")
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
        self.status_bar = StatusBar(self, StatusBarType.WITH_BACK, "이동 옵션")
        self.status_bar.back_signal.connect(self.go_back)  # 뒤로가기 버튼 시그널 연결
        
        # status_bar_placeholder에 상태바 추가
        placeholder_layout = QVBoxLayout(self.ui.status_bar_placeholder)
        placeholder_layout.setContentsMargins(0, 0, 0, 0)
        placeholder_layout.addWidget(self.status_bar)
        
        # 스크롤 영역 설정
        self.setup_scroll_area()
        
        # 스크롤 영역을 메인 레이아웃에 추가
        main_layout.addWidget(self.scroll_area)
        
        # 체크박스 스타일 설정
        if hasattr(self.ui, 'callbell_option_checkBox'):
            self.ui.callbell_option_checkBox.setText("다른 호출벨 무시")
        else:
            print("경고: callbell_option_checkBox를 찾을 수 없습니다. UI 파일을 확인해주세요.")
        
        # 슬라이더 설정
        self.setup_sliders()
        
        # 텍스트 편집 이벤트 연결
        self.setup_text_edits()
        
        # 버튼 시그널 연결
        self.ui.save_button.clicked.connect(self.on_save_clicked)
        
        # 터치 이벤트 설정
        self.setAttribute(Qt.WA_AcceptTouchEvents, True)
        
        # 이벤트 필터 설치 (스크롤 영역 초기화 후)
        self.setup_event_filters()
        
    def setup_scroll_area(self):
        """스크롤 영역 설정"""
        print("[MoveOptionView] 스크롤 영역 설정 시작")
        
        # 스크롤 영역 생성
        self.scroll_area = QScrollArea(self)
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setFrameShape(QFrame.NoFrame)
        self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)  # 스크롤바 표시 정책 변경
        self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        
        # 스크롤 내용 위젯 생성
        self.scroll_content = QWidget()
        
        # 스크롤 내용 레이아웃 설정
        content_layout = QVBoxLayout(self.scroll_content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(0)
        
        # UI 위젯을 스크롤 내용에 추가
        content_layout.addWidget(self.ui)
        
        # 스크롤 내용 위젯에 최소 높이 설정 (스크롤이 가능하도록 충분히 크게)
        self.scroll_content.setMinimumHeight(1200)  # 화면 높이의 2배로 설정
        
        # 스크롤 영역에 스크롤 내용 설정
        self.scroll_area.setWidget(self.scroll_content)
        
        # 스크롤 영역과 내용 위젯에 터치 이벤트 활성화
        self.scroll_area.setAttribute(Qt.WA_AcceptTouchEvents, True)
        self.scroll_area.viewport().setAttribute(Qt.WA_AcceptTouchEvents, True)
        self.scroll_content.setAttribute(Qt.WA_AcceptTouchEvents, True)
        
        # 스크롤 영역 스타일 설정
        self.scroll_area.setStyleSheet("""
            QScrollArea {
                background-color: #141414;
                border: none;
            }
            QScrollBar:vertical {
                background-color: #222222;
                width: 10px;
                margin: 0px;
            }
            QScrollBar::handle:vertical {
                background-color: #666666;
                min-height: 20px;
                border-radius: 5px;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
            }
        """)
        
        print(f"[MoveOptionView] 스크롤 영역 설정 완료: 최소={self.scroll_area.verticalScrollBar().minimum()}, 최대={self.scroll_area.verticalScrollBar().maximum()}")
        
    def setup_sliders(self):
        """슬라이더 설정"""
        # 가속도 슬라이더 설정
        if hasattr(self.ui, 'accel_rate_Slider'):
            self.ui.accel_rate_Slider.setMinimum(0)
            self.ui.accel_rate_Slider.setMaximum(1023)
            self.ui.accel_rate_Slider.setValue(0)
            self.ui.accel_rate_Slider.valueChanged.connect(self.update_accel_text)
        
        # 감속도 슬라이더 설정
        if hasattr(self.ui, 'decel_rate_Slider'):
            self.ui.decel_rate_Slider.setMinimum(0)
            self.ui.decel_rate_Slider.setMaximum(1023)
            self.ui.decel_rate_Slider.setValue(0)
            self.ui.decel_rate_Slider.valueChanged.connect(self.update_decel_text)
        
        # 직진 속도 슬라이더 설정 (0.1m/s ~ 1.57m/s)
        if hasattr(self.ui, 'linear_speed_usr_Slider'):
            # 슬라이더는 정수값만 처리하므로 100배로 설정
            self.ui.linear_speed_usr_Slider.setMinimum(10)  # 0.1 * 100
            self.ui.linear_speed_usr_Slider.setMaximum(157)  # 1.57 * 100
            self.ui.linear_speed_usr_Slider.setValue(10)  # 초기값 0.1
            self.ui.linear_speed_usr_Slider.valueChanged.connect(self.update_linear_speed_text)
        
        # 회전 속도 슬라이더 설정 (0.5rad/s ~ 7.85rad/s)
        if hasattr(self.ui, 'angular_speed_usr_Slider'):
            # 슬라이더는 정수값만 처리하므로 100배로 설정
            self.ui.angular_speed_usr_Slider.setMinimum(50)  # 0.5 * 100
            self.ui.angular_speed_usr_Slider.setMaximum(785)  # 7.85 * 100
            self.ui.angular_speed_usr_Slider.setValue(50)  # 초기값 0.5
            self.ui.angular_speed_usr_Slider.valueChanged.connect(self.update_angular_speed_text)
    
    def setup_text_edits(self):
        """텍스트 편집 설정"""
        # 가속도 텍스트 편집 설정
        if hasattr(self.ui, 'accel_rate_textEdit'):
            self.ui.accel_rate_textEdit.setReadOnly(False)
            self.ui.accel_rate_textEdit.textChanged.connect(self.on_accel_text_changed)
        
        # 감속도 텍스트 편집 설정
        if hasattr(self.ui, 'decel_rate_textEdit'):
            self.ui.decel_rate_textEdit.setReadOnly(False)
            self.ui.decel_rate_textEdit.textChanged.connect(self.on_decel_text_changed)
        
        # 직진 속도 텍스트 편집 설정
        if hasattr(self.ui, 'linear_speed_usr_textEdit'):
            self.ui.linear_speed_usr_textEdit.setReadOnly(False)
            self.ui.linear_speed_usr_textEdit.textChanged.connect(self.on_linear_speed_text_changed)
        
        # 회전 속도 텍스트 편집 설정
        if hasattr(self.ui, 'angular_speed_usr_textEdit'):
            self.ui.angular_speed_usr_textEdit.setReadOnly(False)
            self.ui.angular_speed_usr_textEdit.textChanged.connect(self.on_angular_speed_text_changed)
    
    def update_accel_text(self, value):
        """가속도 텍스트 업데이트"""
        if hasattr(self.ui, 'accel_rate_textEdit'):
            # 텍스트 변경 이벤트 임시 차단
            self.ui.accel_rate_textEdit.blockSignals(True)
            self.ui.accel_rate_textEdit.setHtml(f'<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" "http://www.w3.org/TR/REC-html40/strict.dtd">\n<html><head><meta name="qrichtext" content="1" /><style type="text/css">\np, li {{ white-space: pre-wrap; }}\n</style></head><body style=" font-family:\'Noto Sans KR Black\'; font-size:10px; font-weight:400; font-style:normal;"><p style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:22pt;">{value}</span></p></body></html>')
            self.ui.accel_rate_textEdit.blockSignals(False)
    
    def update_decel_text(self, value):
        """감속도 텍스트 업데이트"""
        if hasattr(self.ui, 'decel_rate_textEdit'):
            # 텍스트 변경 이벤트 임시 차단
            self.ui.decel_rate_textEdit.blockSignals(True)
            self.ui.decel_rate_textEdit.setHtml(f'<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" "http://www.w3.org/TR/REC-html40/strict.dtd">\n<html><head><meta name="qrichtext" content="1" /><style type="text/css">\np, li {{ white-space: pre-wrap; }}\n</style></head><body style=" font-family:\'Noto Sans KR Black\'; font-size:10px; font-weight:400; font-style:normal;"><p style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:22pt;">{value}</span></p></body></html>')
            self.ui.decel_rate_textEdit.blockSignals(False)
    
    def update_linear_speed_text(self, value):
        """직진 속도 텍스트 업데이트 (소수점 둘째자리까지)"""
        if hasattr(self.ui, 'linear_speed_usr_textEdit'):
            # 슬라이더 값을 실제 값으로 변환 (100으로 나누어 소수점 표시)
            real_value = value / 100.0
            formatted_value = f"{real_value:.2f}"  # 소수점 둘째자리까지 포맷팅
            
            # 텍스트 변경 이벤트 임시 차단
            self.ui.linear_speed_usr_textEdit.blockSignals(True)
            self.ui.linear_speed_usr_textEdit.setHtml(f'<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" "http://www.w3.org/TR/REC-html40/strict.dtd">\n<html><head><meta name="qrichtext" content="1" /><style type="text/css">\np, li {{ white-space: pre-wrap; }}\n</style></head><body style=" font-family:\'Noto Sans KR Black\'; font-size:10px; font-weight:400; font-style:normal;"><p style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:22pt;">{formatted_value}</span></p></body></html>')
            self.ui.linear_speed_usr_textEdit.blockSignals(False)
    
    def update_angular_speed_text(self, value):
        """회전 속도 텍스트 업데이트 (소수점 둘째자리까지)"""
        if hasattr(self.ui, 'angular_speed_usr_textEdit'):
            # 슬라이더 값을 실제 값으로 변환 (100으로 나누어 소수점 표시)
            real_value = value / 100.0
            formatted_value = f"{real_value:.2f}"  # 소수점 둘째자리까지 포맷팅
            
            # 텍스트 변경 이벤트 임시 차단
            self.ui.angular_speed_usr_textEdit.blockSignals(True)
            self.ui.angular_speed_usr_textEdit.setHtml(f'<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" "http://www.w3.org/TR/REC-html40/strict.dtd">\n<html><head><meta name="qrichtext" content="1" /><style type="text/css">\np, li {{ white-space: pre-wrap; }}\n</style></head><body style=" font-family:\'Noto Sans KR Black\'; font-size:10px; font-weight:400; font-style:normal;"><p style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:22pt;">{formatted_value}</span></p></body></html>')
            self.ui.angular_speed_usr_textEdit.blockSignals(False)
    
    def on_accel_text_changed(self):
        """가속도 텍스트 변경 이벤트 처리"""
        if hasattr(self.ui, 'accel_rate_textEdit'):
            text = self.ui.accel_rate_textEdit.toPlainText().strip()
            try:
                value = int(text)
                if 0 <= value <= 1023:
                    # 슬라이더 이벤트 임시 차단
                    self.ui.accel_rate_Slider.blockSignals(True)
                    self.ui.accel_rate_Slider.setValue(value)
                    self.ui.accel_rate_Slider.blockSignals(False)
            except ValueError:
                # 숫자가 아닌 경우 슬라이더 값으로 텍스트 복원
                self.update_accel_text(self.ui.accel_rate_Slider.value())
    
    def on_decel_text_changed(self):
        """감속도 텍스트 변경 이벤트 처리"""
        if hasattr(self.ui, 'decel_rate_textEdit'):
            text = self.ui.decel_rate_textEdit.toPlainText().strip()
            try:
                value = int(text)
                if 0 <= value <= 1023:
                    # 슬라이더 이벤트 임시 차단
                    self.ui.decel_rate_Slider.blockSignals(True)
                    self.ui.decel_rate_Slider.setValue(value)
                    self.ui.decel_rate_Slider.blockSignals(False)
            except ValueError:
                # 숫자가 아닌 경우 슬라이더 값으로 텍스트 복원
                self.update_decel_text(self.ui.decel_rate_Slider.value())
    
    def on_linear_speed_text_changed(self):
        """직진 속도 텍스트 변경 이벤트 처리"""
        if hasattr(self.ui, 'linear_speed_usr_textEdit'):
            text = self.ui.linear_speed_usr_textEdit.toPlainText().strip()
            try:
                value = float(text)
                if 0.1 <= value <= 1.57:
                    # 슬라이더 이벤트 임시 차단
                    self.ui.linear_speed_usr_Slider.blockSignals(True)
                    self.ui.linear_speed_usr_Slider.setValue(int(value * 100))
                    self.ui.linear_speed_usr_Slider.blockSignals(False)
            except ValueError:
                # 숫자가 아닌 경우 슬라이더 값으로 텍스트 복원
                self.update_linear_speed_text(self.ui.linear_speed_usr_Slider.value())
    
    def on_angular_speed_text_changed(self):
        """회전 속도 텍스트 변경 이벤트 처리"""
        if hasattr(self.ui, 'angular_speed_usr_textEdit'):
            text = self.ui.angular_speed_usr_textEdit.toPlainText().strip()
            try:
                value = float(text)
                if 0.5 <= value <= 7.85:
                    # 슬라이더 이벤트 임시 차단
                    self.ui.angular_speed_usr_Slider.blockSignals(True)
                    self.ui.angular_speed_usr_Slider.setValue(int(value * 100))
                    self.ui.angular_speed_usr_Slider.blockSignals(False)
            except ValueError:
                # 숫자가 아닌 경우 슬라이더 값으로 텍스트 복원
                self.update_angular_speed_text(self.ui.angular_speed_usr_Slider.value())
    
    def set_ignore_state(self, ignore_enabled):
        """무시 상태 설정"""
        if hasattr(self.ui, 'callbell_option_checkBox'):
            self.ui.callbell_option_checkBox.setChecked(ignore_enabled)
        else:
            print("경고: callbell_option_checkBox를 찾을 수 없습니다.")
    
    def set_accel_rate(self, value):
        """가속도 값 설정"""
        if hasattr(self.ui, 'accel_rate_Slider'):
            self.ui.accel_rate_Slider.setValue(value)
    
    def set_decel_rate(self, value):
        """감속도 값 설정"""
        if hasattr(self.ui, 'decel_rate_Slider'):
            self.ui.decel_rate_Slider.setValue(value)
    
    def set_brake_mode(self, enabled):
        """브레이크 모드 설정"""
        if hasattr(self.ui, 'brake_mode_checkBox'):
            self.ui.brake_mode_checkBox.setChecked(enabled)
    
    def get_ignore_state(self):
        """현재 선택된 무시 상태 반환"""
        if hasattr(self.ui, 'callbell_option_checkBox'):
            return self.ui.callbell_option_checkBox.isChecked()
        else:
            print("경고: callbell_option_checkBox를 찾을 수 없습니다.")
            return False
    
    def get_accel_rate(self):
        """현재 가속도 값 반환"""
        if hasattr(self.ui, 'accel_rate_Slider'):
            return self.ui.accel_rate_Slider.value()
        return 0
    
    def get_decel_rate(self):
        """현재 감속도 값 반환"""
        if hasattr(self.ui, 'decel_rate_Slider'):
            return self.ui.decel_rate_Slider.value()
        return 0
    
    def get_brake_mode(self):
        """현재 브레이크 모드 반환"""
        if hasattr(self.ui, 'brake_mode_checkBox'):
            return self.ui.brake_mode_checkBox.isChecked()
        return False
    
    def get_linear_speed(self):
        """현재 직진 속도 값 반환 (실제 값)"""
        if hasattr(self.ui, 'linear_speed_usr_Slider'):
            return self.ui.linear_speed_usr_Slider.value() / 100.0
        return 0.1  # 기본값
    
    def get_angular_speed(self):
        """현재 회전 속도 값 반환 (실제 값)"""
        if hasattr(self.ui, 'angular_speed_usr_Slider'):
            return self.ui.angular_speed_usr_Slider.value() / 100.0
        return 0.5  # 기본값
    
    def on_save_clicked(self):
        """저장 버튼 클릭 처리"""
        # 현재 선택된 상태를 전달
        ignore_state = self.get_ignore_state()
        accel_rate = self.get_accel_rate()
        decel_rate = self.get_decel_rate()
        brake_mode = self.get_brake_mode()
        linear_speed = self.get_linear_speed()
        angular_speed = self.get_angular_speed()
        self.save_signal.emit(ignore_state, accel_rate, decel_rate, brake_mode, linear_speed, angular_speed)
    
    def go_back(self):
        """뒤로가기 버튼 클릭 처리"""
        self.back_signal.emit()
    
    def setup_event_filters(self):
        """이벤트 필터 설정"""
        if self.scroll_area:
            self.scroll_area.viewport().installEventFilter(self)
            self.scroll_area.installEventFilter(self)
            self.scroll_content.installEventFilter(self)
            print("[MoveOptionView] 이벤트 필터 설치 완료")
        else:
            print("[MoveOptionView] 경고: 스크롤 영역이 초기화되지 않아 이벤트 필터를 설치할 수 없습니다.")

    def eventFilter(self, obj, event):
        """이벤트 필터"""
        # 스크롤 영역 또는 뷰포트의 이벤트만 처리
        if obj == self.scroll_area or obj == self.scroll_area.viewport():
            # 마우스 이벤트 처리
            if event.type() == QEvent.MouseButtonPress:
                self.handle_press_event(event)
                # 이벤트를 소비하지 않고 전달 (return False)
                return False
            elif event.type() == QEvent.MouseMove:
                if self.is_touching:
                    # 드래그 상태 확인
                    if not self.is_dragging:
                        # 드래그 시작 여부 확인
                        current_pos = event.position().toPoint()
                        delta = current_pos - self.touch_start_pos
                        # 일정 거리 이상 이동했을 때만 드래그로 간주
                        if abs(delta.y()) > self.drag_threshold:
                            self.is_dragging = True
                    
                    # 드래그 중일 때만 스크롤 처리
                    if self.is_dragging:
                        self.handle_move_event(event)
                        return True
                    return False
            elif event.type() == QEvent.MouseButtonRelease:
                if self.is_touching:
                    # 드래그 중이었다면 스크롤 종료 처리
                    if self.is_dragging:
                        self.handle_release_event(event)
                        self.is_dragging = False
                        return True
                    # 드래그가 아니었다면 클릭 이벤트로 전달
                    self.is_touching = False
                    self.touch_start_pos = None
                    self.last_touch_pos = None
                    return False
            
            # 터치 이벤트 처리
            elif event.type() == QEvent.TouchBegin:
                self.handle_touch_begin(event)
                # 이벤트를 소비하지 않고 전달 (return False)
                return False
            elif event.type() == QEvent.TouchUpdate:
                if self.is_touching:
                    # 드래그 상태 확인
                    if not self.is_dragging:
                        # 드래그 시작 여부 확인
                        touch_event = QTouchEvent(event)
                        touch_points = touch_event.points()
                        if touch_points:
                            current_pos = touch_points[0].position().toPoint()
                            delta = current_pos - self.touch_start_pos
                            # 일정 거리 이상 이동했을 때만 드래그로 간주
                            if abs(delta.y()) > self.drag_threshold:
                                self.is_dragging = True
                    
                    # 드래그 중일 때만 스크롤 처리
                    if self.is_dragging:
                        self.handle_touch_update(event)
                        return True
                    return False
            elif event.type() == QEvent.TouchEnd:
                if self.is_touching:
                    # 드래그 중이었다면 스크롤 종료 처리
                    if self.is_dragging:
                        self.handle_touch_end(event)
                        self.is_dragging = False
                        return True
                    # 드래그가 아니었다면 탭 이벤트로 전달
                    self.is_touching = False
                    self.touch_start_pos = None
                    self.last_touch_pos = None
                    return False
        
        # 기본 이벤트 처리
        return super().eventFilter(obj, event)

    def handle_press_event(self, event):
        """마우스 버튼 누름 이벤트 처리"""
        print("[MoveOptionView] 마우스 버튼 누름 이벤트")
        self.touch_start_pos = event.position().toPoint()
        self.last_touch_pos = self.touch_start_pos

    def handle_move_event(self, event):
        """마우스 드래그 이벤트 처리"""
        print("[MoveOptionView] 마우스 드래그 이벤트")
        current_pos = event.position().toPoint()
        delta = current_pos - self.last_touch_pos
        self.scroll_area.verticalScrollBar().setValue(self.scroll_area.verticalScrollBar().value() - delta.y())
        self.last_touch_pos = current_pos

    def handle_release_event(self, event):
        """마우스 버튼 떼기 이벤트 처리"""
        print("[MoveOptionView] 마우스 버튼 떼기 이벤트")
        self.is_touching = False
        self.touch_start_pos = None
        self.last_touch_pos = None

    def handle_touch_begin(self, event):
        """터치 시작 이벤트 처리"""
        print("[MoveOptionView] 터치 시작 이벤트")
        touch_event = QTouchEvent(event)
        touch_points = touch_event.points()
        if touch_points:
            self.touch_start_pos = touch_points[0].position().toPoint()
            self.last_touch_pos = self.touch_start_pos
            self.is_touching = True

    def handle_touch_update(self, event):
        """터치 업데이트 이벤트 처리"""
        print("[MoveOptionView] 터치 업데이트 이벤트")
        touch_event = QTouchEvent(event)
        touch_points = touch_event.points()
        if touch_points:
            current_pos = touch_points[0].position().toPoint()
            delta = current_pos - self.touch_start_pos
            self.scroll_area.verticalScrollBar().setValue(self.scroll_area.verticalScrollBar().value() - delta.y())
            self.touch_start_pos = current_pos

    def handle_touch_end(self, event):
        """터치 종료 이벤트 처리"""
        print("[MoveOptionView] 터치 종료 이벤트")
        self.is_touching = False
        self.touch_start_pos = None
        self.last_touch_pos = None

    def handle_press_event(self, event):
        """마우스 버튼 누름 이벤트 처리"""
        print("[MoveOptionView] 마우스 버튼 누름 이벤트")
        self.touch_start_pos = event.position().toPoint()
        self.last_touch_pos = self.touch_start_pos

    def handle_move_event(self, event):
        """마우스 드래그 이벤트 처리"""
        print("[MoveOptionView] 마우스 드래그 이벤트")
        current_pos = event.position().toPoint()
        delta = current_pos - self.last_touch_pos
        self.scroll_area.verticalScrollBar().setValue(self.scroll_area.verticalScrollBar().value() - delta.y())
        self.last_touch_pos = current_pos

    def handle_release_event(self, event):
        """마우스 버튼 떼기 이벤트 처리"""
        print("[MoveOptionView] 마우스 버튼 떼기 이벤트")
        self.is_touching = False
        self.touch_start_pos = None
        self.last_touch_pos = None

    def handle_touch_begin(self, event):
        """터치 시작 이벤트 처리"""
        print("[MoveOptionView] 터치 시작 이벤트")
        touch_event = QTouchEvent(event)
        touch_points = touch_event.points()
        if touch_points:
            self.touch_start_pos = touch_points[0].position().toPoint()
            self.last_touch_pos = self.touch_start_pos
            self.is_touching = True

    def handle_touch_update(self, event):
        """터치 업데이트 이벤트 처리"""
        print("[MoveOptionView] 터치 업데이트 이벤트")
        touch_event = QTouchEvent(event)
        touch_points = touch_event.points()
        if touch_points:
            current_pos = touch_points[0].position().toPoint()
            delta = current_pos - self.touch_start_pos
            self.scroll_area.verticalScrollBar().setValue(self.scroll_area.verticalScrollBar().value() - delta.y())
            self.touch_start_pos = current_pos

    def handle_touch_end(self, event):
        """터치 종료 이벤트 처리"""
        print("[MoveOptionView] 터치 종료 이벤트")
        self.is_touching = False
        self.touch_start_pos = None
        self.last_touch_pos = None 