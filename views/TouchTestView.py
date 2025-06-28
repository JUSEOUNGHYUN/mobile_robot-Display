from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGraphicsScene, QGraphicsEllipseItem
from PySide6.QtGui import QColor, QPen, QBrush, QFont, QPainter
from PySide6.QtCore import Qt, Signal, QFile, QEvent, QPointF, QTimer
from PySide6.QtUiTools import QUiLoader
from views.StatusBar import StatusBar, StatusBarType
import os
import time

class TouchTestView(QWidget):
    # 시그널 정의
    back_signal = Signal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # UI 파일 로드
        loader = QUiLoader()
        ui_file_path = os.path.join(os.path.dirname(__file__), "..", "UI", "TouchTestView.ui")
        ui_file = QFile(ui_file_path)
        ui_file.open(QFile.ReadOnly)
        self.ui = loader.load(ui_file, self)
        ui_file.close()
        
        # 윈도우 설정
        self.setWindowTitle('터치 테스트')
        self.setFixedSize(1024, 600)
        
        # 레이아웃 설정
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 상태바 추가 (StatusBar WITH_BACK 타입 사용, 설정 버튼 없음)
        self.status_bar = StatusBar(self, StatusBarType.WITH_BACK, "터치 테스트")
        self.status_bar.back_signal.connect(self.go_back)  # 뒤로가기 버튼 시그널 연결
        
        # status_bar_placeholder에 상태바 추가
        placeholder_layout = QVBoxLayout(self.ui.status_bar_placeholder)
        placeholder_layout.setContentsMargins(0, 0, 0, 0)
        placeholder_layout.addWidget(self.status_bar)
        
        # UI 위젯을 메인 레이아웃에 추가
        main_layout.addWidget(self.ui)
        
        # 그래픽스 씬 설정
        self.scene = QGraphicsScene(self)
        self.ui.touch_area.setScene(self.scene)
        self.ui.touch_area.setRenderHint(QPainter.Antialiasing)
        
        # 터치 이벤트 활성화
        self.ui.touch_area.viewport().setAttribute(Qt.WA_AcceptTouchEvents)
        self.ui.touch_area.viewport().grabGesture(Qt.PinchGesture)
        self.ui.touch_area.viewport().grabGesture(Qt.SwipeGesture)
        self.ui.touch_area.viewport().grabGesture(Qt.PanGesture)
        
        # 이벤트 필터 설정
        self.ui.touch_area.viewport().installEventFilter(self)
        
        # 터치 포인트 저장 변수
        self.touch_points = {}
        self.touch_items = {}
        self.touch_text_items = {}
        
        # 드래그 관련 변수
        self.drag_start_pos = None
        self.drag_path_item = None
        self.drag_paths = []
        
        # 멀티터치 카운트
        self.max_touch_count = 0
        self.last_touch_time = time.time()
        
        # 초기화 버튼 연결
        self.ui.clear_button.clicked.connect(self.reset)
        
    def go_back(self):
        """뒤로가기 버튼 클릭 시 호출되는 메서드"""
        self.back_signal.emit()
    
    def eventFilter(self, obj, event):
        """이벤트 필터 - 터치 이벤트 처리"""
        if obj == self.ui.touch_area.viewport():
            # 터치 이벤트 처리
            if event.type() == QEvent.TouchBegin:
                self.handleTouchBegin(event)
                return True
            elif event.type() == QEvent.TouchUpdate:
                self.handleTouchUpdate(event)
                return True
            elif event.type() == QEvent.TouchEnd:
                self.handleTouchEnd(event)
                return True
            
            # 마우스 이벤트 처리 (터치 대체용)
            elif event.type() == QEvent.MouseButtonPress:
                self.handleMousePress(event)
                return True
            elif event.type() == QEvent.MouseMove:
                self.handleMouseMove(event)
                return True
            elif event.type() == QEvent.MouseButtonRelease:
                self.handleMouseRelease(event)
                return True
                
        return super().eventFilter(obj, event)
    
    def handleTouchBegin(self, event):
        """터치 시작 이벤트 처리"""
        touch_points = event.points()
        current_touch_count = len(touch_points)
        
        # 최대 터치 수 업데이트
        if current_touch_count > self.max_touch_count:
            self.max_touch_count = current_touch_count
            self.ui.touch_info_label.setText(f"감지된 최대 터치 수: {self.max_touch_count}")
        
        # 각 터치 포인트 처리
        for touch_point in touch_points:
            point_id = touch_point.id()
            pos = self.ui.touch_area.mapToScene(touch_point.pos().toPoint())
            
            # 터치 포인트 위치 저장
            self.touch_points[point_id] = pos
            
            # 터치 포인트 시각화
            ellipse = QGraphicsEllipseItem(pos.x() - 15, pos.y() - 15, 30, 30)
            ellipse.setPen(QPen(QColor(0, 0, 255), 2))
            ellipse.setBrush(QBrush(QColor(0, 0, 255, 100)))
            self.scene.addItem(ellipse)
            self.touch_items[point_id] = ellipse
            
            # 터치 ID 표시
            text_item = self.scene.addText(str(point_id))
            text_item.setPos(pos.x() - 5, pos.y() - 10)
            text_item.setDefaultTextColor(Qt.white)
            text_item.setFont(QFont("Arial", 10, QFont.Bold))
            self.touch_text_items[point_id] = text_item
            
            # 터치 정보 업데이트
            self.ui.touch_info_label.setText(f"터치 시작: ID {point_id}, 위치 ({pos.x():.1f}, {pos.y():.1f})")
            
            # 다른 터치 포인트와 연결선 그리기
            self.connectTouchPoints(point_id, pos)
        
        event.accept()
    
    def connectTouchPoints(self, new_point_id, new_pos):
        """새 터치 포인트와 기존 터치 포인트들을 연결하는 선 그리기"""
        for point_id, pos in self.touch_points.items():
            if point_id != new_point_id:
                # 두 터치 포인트 사이에 연결선 그리기
                line = self.scene.addLine(
                    new_pos.x(), new_pos.y(), 
                    pos.x(), pos.y(), 
                    QPen(QColor(0, 255, 0, 150), 2, Qt.DashLine)
                )
                self.drag_paths.append(line)
    
    def handleTouchUpdate(self, event):
        """터치 업데이트 이벤트 처리"""
        touch_points = event.points()
        
        # 이전 연결선 제거 (멀티터치 포인트 간 연결선)
        for path in self.drag_paths[:]:
            if path.pen().style() == Qt.DashLine:
                self.scene.removeItem(path)
                self.drag_paths.remove(path)
        
        for touch_point in touch_points:
            point_id = touch_point.id()
            pos = self.ui.touch_area.mapToScene(touch_point.pos().toPoint())
            
            if point_id in self.touch_points and point_id in self.touch_items:
                # 이전 위치
                prev_pos = self.touch_points[point_id]
                
                # 터치 포인트 이동
                ellipse = self.touch_items[point_id]
                ellipse.setRect(pos.x() - 15, pos.y() - 15, 30, 30)
                
                # 텍스트 아이템도 이동
                if point_id in self.touch_text_items:
                    self.touch_text_items[point_id].setPos(pos.x() - 5, pos.y() - 10)
                
                # 드래그 경로 그리기 (이동 경로)
                line = self.scene.addLine(prev_pos.x(), prev_pos.y(), pos.x(), pos.y(), 
                                          QPen(QColor(255, 0, 0, 150), 3))
                self.drag_paths.append(line)
                
                # 위치 업데이트
                self.touch_points[point_id] = pos
                
                # 터치 정보 업데이트
                self.ui.touch_info_label.setText(f"터치 이동: ID {point_id}, 위치 ({pos.x():.1f}, {pos.y():.1f})")
                
                # 다른 터치 포인트와 연결선 그리기
                self.connectTouchPoints(point_id, pos)
        
        event.accept()
    
    def handleTouchEnd(self, event):
        """터치 종료 이벤트 처리"""
        touch_points = event.points()
        
        for touch_point in touch_points:
            point_id = touch_point.id()
            
            if point_id in self.touch_items:
                # 터치 포인트 제거
                self.scene.removeItem(self.touch_items[point_id])
                del self.touch_items[point_id]
                
            if point_id in self.touch_text_items:
                # 텍스트 아이템 제거
                self.scene.removeItem(self.touch_text_items[point_id])
                del self.touch_text_items[point_id]
                
            if point_id in self.touch_points:
                del self.touch_points[point_id]
                
        # 모든 터치가 끝나면 경로 정리 타이머 시작
        if not self.touch_points:
            self.last_touch_time = time.time()
            QTimer.singleShot(3000, self.checkPathClear)
            self.ui.touch_info_label.setText(f"감지된 최대 터치 수: {self.max_touch_count}")
        
        event.accept()
    
    def handleMousePress(self, event):
        """마우스 버튼 누름 이벤트 처리 (터치 대체용)"""
        pos = self.ui.touch_area.mapToScene(event.pos())
        self.drag_start_pos = pos
        
        # 마우스 클릭 위치 시각화
        ellipse = QGraphicsEllipseItem(pos.x() - 15, pos.y() - 15, 30, 30)
        ellipse.setPen(QPen(QColor(0, 0, 255), 2))
        ellipse.setBrush(QBrush(QColor(0, 0, 255, 100)))
        self.scene.addItem(ellipse)
        self.touch_items[-1] = ellipse  # 마우스는 ID -1로 지정
        
        # 마우스 ID 표시
        text_item = self.scene.addText("M")
        text_item.setPos(pos.x() - 5, pos.y() - 10)
        text_item.setDefaultTextColor(Qt.white)
        text_item.setFont(QFont("Arial", 10, QFont.Bold))
        self.touch_text_items[-1] = text_item
        
        # 터치 포인트 위치 저장
        self.touch_points[-1] = pos
        
        # 터치 정보 업데이트
        self.ui.touch_info_label.setText(f"마우스 클릭: 위치 ({pos.x():.1f}, {pos.y():.1f})")
        
        event.accept()
    
    def handleMouseMove(self, event):
        """마우스 이동 이벤트 처리 (터치 대체용)"""
        if event.buttons() & Qt.LeftButton and self.drag_start_pos:
            pos = self.ui.touch_area.mapToScene(event.pos())
            
            # 마우스 포인터 이동
            if -1 in self.touch_items:
                ellipse = self.touch_items[-1]
                ellipse.setRect(pos.x() - 15, pos.y() - 15, 30, 30)
                
                # 텍스트 아이템도 이동
                if -1 in self.touch_text_items:
                    self.touch_text_items[-1].setPos(pos.x() - 5, pos.y() - 10)
            
            # 드래그 경로 그리기
            line = self.scene.addLine(self.drag_start_pos.x(), self.drag_start_pos.y(), 
                                      pos.x(), pos.y(), QPen(QColor(255, 0, 0, 150), 3))
            self.drag_paths.append(line)
            
            # 시작 위치 업데이트
            self.drag_start_pos = pos
            
            # 터치 포인트 위치 업데이트
            self.touch_points[-1] = pos
            
            # 터치 정보 업데이트
            self.ui.touch_info_label.setText(f"마우스 드래그: 위치 ({pos.x():.1f}, {pos.y():.1f})")
        
        event.accept()
    
    def handleMouseRelease(self, event):
        """마우스 버튼 뗌 이벤트 처리 (터치 대체용)"""
        if -1 in self.touch_items:
            # 마우스 포인터 제거
            self.scene.removeItem(self.touch_items[-1])
            del self.touch_items[-1]
            
            # 텍스트 아이템 제거
            if -1 in self.touch_text_items:
                self.scene.removeItem(self.touch_text_items[-1])
                del self.touch_text_items[-1]
            
            # 터치 포인트 제거
            if -1 in self.touch_points:
                del self.touch_points[-1]
        
        self.drag_start_pos = None
        
        # 경로 정리 타이머 시작 (3초 후 정리)
        self.last_touch_time = time.time()
        QTimer.singleShot(3000, self.checkPathClear)
        
        # 터치 정보 업데이트
        self.ui.touch_info_label.setText("마우스 드래그 완료")
        
        event.accept()
    
    def checkPathClear(self):
        """경로 정리 여부 확인"""
        if time.time() - self.last_touch_time >= 3:
            self.clearPaths()
    
    def clearPaths(self):
        """모든 드래그 경로 정리"""
        for path in self.drag_paths:
            self.scene.removeItem(path)
        self.drag_paths = []
        
        # 모든 터치 포인트 정리
        for item in self.touch_items.values():
            self.scene.removeItem(item)
        self.touch_items = {}
        
        # 모든 텍스트 아이템 정리
        for item in self.touch_text_items.values():
            self.scene.removeItem(item)
        self.touch_text_items = {}
        
        self.touch_points = {}
        
    def reset(self):
        """모든 데이터 초기화"""
        self.clearPaths()
        self.max_touch_count = 0
        self.ui.touch_info_label.setText("터치 정보가 여기에 표시됩니다.") 