from PySide6.QtWidgets import QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout
from PySide6.QtGui import QFont, QColor
from PySide6.QtCore import Qt, Signal, QPropertyAnimation, QEasingCurve, QPoint, QParallelAnimationGroup
from PySide6.QtUiTools import QUiLoader
from views.StatusBar import StatusBar, StatusBarType
from Utils.MessageBox import MessageBox
from Utils.ButtonStyle import RoundButtonStyle
import os

class LocationDragButton(QPushButton):
    """드래그 가능한 위치 버튼 컴포넌트"""
    
    drag_started = Signal(object)  # 드래그 시작 시그널 (버튼 객체 전달)
    drag_ended = Signal(object, QPoint)  # 드래그 종료 시그널 (버튼 객체, 위치 전달)
    
    # 버튼 스타일 정의 함수
    @staticmethod
    def generate_style(font_size=40):
        return f"""
        QPushButton {{
            background-color: #c2db3a;       /* 밝은 연두색 */
            color: white;                    /* 텍스트 흰색 */
            font-size: {font_size}px;        /* 글자 크기 */
            font-weight: 500;                /* 글자 두께 중간 */
            border-radius: 3px;              /* 살짝만 둥글게 */
            height: 40px;                    /* 고정 높이 */
            border: none;
            }}

        QPushButton:hover {{
            background-color: #b5c732;       /* 호버 시 더 진한 연두 */
        }}

        QPushButton:pressed {{
            background-color: #a5b52a;       /* 클릭 시 연두색 어둡게 */
        }}
        """
    
    def __init__(self, text, location_data=None, width=230, height=130, parent=None):
        """
        Args:
            text (str): 버튼에 표시할 텍스트
            location_data (dict): 위치 데이터
            width (int): 버튼 너비
            height (int): 버튼 높이
        """
        super().__init__(text, parent)
        self.location_data = location_data
        self.setFixedSize(width, height)
        
        # 버튼 크기에 따라 폰트 크기 조절
        if width >= 350:  # 큰 버튼
            font_size = 40
            self.setFont(QFont('Arial', 32))
        elif width >= 300:  # 중간 큰 버튼
            font_size = 35
            self.setFont(QFont('Arial', 28))
        elif width >= 250:  # 중간 버튼
            font_size = 30
            self.setFont(QFont('Arial', 24))
        else:  # 작은 버튼
            font_size = 25
            self.setFont(QFont('Arial', 20))
        
        # 버튼 스타일 설정
        self.normal_style = self.generate_style(font_size)
        self.setStyleSheet(self.normal_style)
        
        # 드래그 관련 변수
        self.dragging = False
        self.drag_start_position = None
        self.original_position = None
        self.grid_position = None  # (row, col) 튜플로 저장
    
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragging = True
            self.drag_start_position = event.position()
            self.original_position = self.pos()
            self.raise_()  # 드래그 중인 버튼을 최상위로
            self.drag_started.emit(self)
        super().mousePressEvent(event)
    
    def mouseMoveEvent(self, event):
        if not self.dragging:
            return
        
        # 새 위치 계산
        new_pos = self.mapToParent(event.position().toPoint() - self.drag_start_position.toPoint())
        self.move(new_pos)
        
        # 부모 위젯 경계 내에서만 이동하도록 제한
        parent = self.parent()
        if parent:
            new_x = max(0, min(new_pos.x(), parent.width() - self.width()))
            new_y = max(0, min(new_pos.y(), parent.height() - self.height()))
            self.move(new_x, new_y)
    
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self.dragging:
            self.dragging = False
            self.drag_ended.emit(self, self.pos())
        super().mouseReleaseEvent(event)
    
    def get_location_data(self):
        return self.location_data

class ChangeLayoutView(QWidget):
    """목적지 레이아웃 변경 화면"""
    
    # 시그널 정의
    layout_changed_signal = Signal(list)  # 레이아웃 변경 시그널 (변경된 위치 목록 전달)
    cancel_signal = Signal()  # 취소 시그널
    save_signal = Signal(list)  # 저장 시그널 (변경된 위치 목록 전달)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # UI 파일 로드
        loader = QUiLoader()
        ui_file_path = os.path.join(os.path.dirname(__file__), "..", "UI", "ChangeLayoutView.ui")
        # UI 파일이 없는 경우 직접 UI 생성
        try:
            self.ui = loader.load(ui_file_path)
            
            # 메인 레이아웃 설정
            layout = QVBoxLayout(self)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(0)
            
            # 상태바 추가
            self.status_bar = StatusBar(self, StatusBarType.FULL)
            layout.addWidget(self.status_bar)
            
            # UI 파일에서 로드한 위젯 추가
            layout.addWidget(self.ui)
            
            # UI 요소 연결
            self.title_label = self.ui.findChild(QLabel, "title_label")
            self.grid_layout = self.ui.findChild(QGridLayout, "grid_layout")
            self.cancel_button = self.ui.findChild(QPushButton, "cancel_button")
            self.save_button = self.ui.findChild(QPushButton, "save_button")
            
        except Exception as e:
            print(f"UI 파일 로드 실패: {str(e)}")
            # UI 파일이 없는 경우 직접 UI 생성
            self._create_ui_manually()
        
        # 버튼 시그널 연결
        self.cancel_button.clicked.connect(self.on_cancel_button_clicked)
        self.save_button.clicked.connect(self.on_save_button_clicked)
        
        # 레이아웃 매핑 정보 (버튼과 그리드 위치)
        self.button_positions = {}  # {button: (row, col)} 형태로 저장
        self.grid_positions = {}    # {(row, col): button} 형태로 저장
        self.layout_data = []       # [{name, position_index}] 형태로 저장
        
        # 애니메이션 그룹
        self.animation_group = QParallelAnimationGroup(self)
        self.animation_group.finished.connect(self.on_animation_finished)
        
        # 현재 드래그 중인 버튼
        self.current_drag_button = None
        self.target_position = None
    
    def _create_ui_manually(self):
        """UI 파일이 없는 경우 수동으로 UI 생성"""
        # 메인 레이아웃
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # 상태바 추가
        self.status_bar = StatusBar(self, StatusBarType.FULL)
        layout.addWidget(self.status_bar)
        
        # 콘텐츠 컨테이너
        content_widget = QWidget()
        content_layout = QVBoxLayout(content_widget)
        
        # 타이틀 레이블
        self.title_label = QLabel("목적지 레이아웃 변경")
        self.title_label.setFont(QFont('Arial', 24, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("color: #c2db3a; margin: 10px 0;")
        content_layout.addWidget(self.title_label)
        
        # 설명 레이블
        description_label = QLabel("Drag the button to your desired position to rearrange the layout.")
        description_label.setFont(QFont('Arial', 16))
        description_label.setAlignment(Qt.AlignCenter)
        description_label.setStyleSheet("color: white; margin-bottom: 20px;")
        content_layout.addWidget(description_label)
        
        # 그리드 레이아웃 (버튼 배치용)
        grid_container = QWidget()
        self.grid_layout = QGridLayout(grid_container)
        self.grid_layout.setSpacing(10)
        content_layout.addWidget(grid_container)
        
        # 버튼 컨테이너
        button_container = QWidget()
        button_layout = QHBoxLayout(button_container)
        
        # 취소 버튼
        self.cancel_button = QPushButton("취소")
        self.cancel_button.setFont(QFont('Arial', 18))
        self.cancel_button.setFixedSize(200, 60)
        self.cancel_button.setStyleSheet("""
            QPushButton {
                background-color: #555555;
                color: white;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #666666;
            }
        """)
        
        # 저장 버튼
        self.save_button = QPushButton("저장")
        self.save_button.setFont(QFont('Arial', 18))
        self.save_button.setFixedSize(200, 60)
        self.save_button.setStyleSheet("""
            QPushButton {
                background-color: #c2db3a;
                color: white;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #b5c732;
            }
        """)
        
        # 버튼 레이아웃에 추가
        button_layout.addStretch()
        button_layout.addWidget(self.cancel_button)
        button_layout.addSpacing(20)
        button_layout.addWidget(self.save_button)
        button_layout.addStretch()
        
        # 버튼 컨테이너 추가
        content_layout.addWidget(button_container)
        content_layout.addStretch()
        
        # 콘텐츠 위젯 추가
        layout.addWidget(content_widget)
    
    def update_locations(self, locations):
        """위치 목록 업데이트"""
        self.locations = locations or []
        self.setup_location_buttons()
    
    def setup_location_buttons(self):
        """위치 버튼 초기화 및 배치"""
        # 기존 버튼 제거
        for button in self.button_positions.keys():
            self.grid_layout.removeWidget(button)
            button.deleteLater()
        
        self.button_positions = {}
        self.grid_positions = {}
        
        # 목적지 개수에 따라 그리드 설정 (유동적 레이아웃)
        total_locations = len(self.locations)
        
        # 그리드 크기 계산
        # 기본은 1행 또는 2행으로 설정, 목적지가 많으면 열 수 증가
        if total_locations <= 4:
            max_cols = total_locations
            max_rows = 1
        elif total_locations <= 8:
            max_cols = 4
            max_rows = 2
        else:
            # 8개 초과 시 열 수를 조절하여 3행 이상으로 표시
            max_cols = 4
            max_rows = (total_locations + 3) // 4  # 올림 나눗셈
            
        print(f"레이아웃 설정: {max_rows}행 x {max_cols}열, 총 {total_locations}개 목적지")
        
        # 버튼 크기 통일 (화면 해상도에 맞게 조절)
        button_width = 180  # 기본 크기
        button_height = 120  # 기본 크기
        
        # 버튼 생성 및 배치
        for i, location in enumerate(self.locations):
            # 위치 계산
            row = i // max_cols
            col = i % max_cols
            
            # 버튼 텍스트 설정
            if isinstance(location, dict) and 'name' in location:
                button_text = location['name']
            else:
                button_text = str(location)
            
            # 버튼 생성
            button = LocationDragButton(button_text, location, button_width, button_height, self)
            
            # 드래그 이벤트 연결
            button.drag_started.connect(self.on_button_drag_started)
            button.drag_ended.connect(self.on_button_drag_ended)
            
            # 버튼과 그리드 위치 매핑
            self.button_positions[button] = (row, col)
            self.grid_positions[(row, col)] = button
            
            # 그리드에 추가
            self.grid_layout.addWidget(button, row, col)
        
        # 레이아웃 데이터 초기화
        self.update_layout_data()
    
    def update_layout_data(self):
        """현재 레이아웃 데이터 업데이트"""
        # 레이아웃 데이터 초기화
        self.layout_data = []
        
        # 현재 그리드의 최대 열 수 계산
        max_cols = 4  # 기본값
        for row, col in self.grid_positions.keys():
            max_cols = max(max_cols, col + 1)
        
        # 모든 버튼에 대해 위치 데이터 저장
        for button, position in self.button_positions.items():
            location_data = button.get_location_data()
            if location_data:
                # 행, 열 위치를 단일 인덱스로 변환 (행 * 열개수 + 열)
                position_index = position[0] * max_cols + position[1]
                
                self.layout_data.append({
                    'name': location_data.get('name', ''),
                    'position_index': position_index,
                    'location_data': location_data
                })
    
    def on_button_drag_started(self, button):
        """버튼 드래그 시작 시 처리"""
        self.current_drag_button = button
    
    def on_button_drag_ended(self, button, position):
        """버튼 드래그 종료 시 처리"""
        # 드래그 종료된 위치에서 가장 가까운 그리드 셀 찾기
        closest_cell = self.find_closest_grid_cell(position)
        
        if closest_cell is None:
            # 원래 위치로 되돌리기
            self.animate_button_to_grid(button, self.button_positions[button])
            return
        
        # 드롭된 셀에 이미 다른 버튼이 있는지 확인
        if closest_cell in self.grid_positions and self.grid_positions[closest_cell] != button:
            # 위치 교환 애니메이션 시작
            target_button = self.grid_positions[closest_cell]
            source_pos = self.button_positions[button]
            
            # 위치 교환
            self.button_positions[target_button] = source_pos
            self.grid_positions[source_pos] = target_button
            
            # 현재 버튼 위치 업데이트
            self.button_positions[button] = closest_cell
            self.grid_positions[closest_cell] = button
            
            # 애니메이션 시작
            self.animate_swap(button, target_button, closest_cell, source_pos)
        else:
            # 빈 셀로 이동
            old_pos = self.button_positions[button]
            
            # 위치 업데이트
            self.button_positions[button] = closest_cell
            self.grid_positions[closest_cell] = button
            
            if old_pos in self.grid_positions:
                del self.grid_positions[old_pos]
            
            # 애니메이션 시작
            self.animate_button_to_grid(button, closest_cell)
        
        # 레이아웃 데이터 업데이트
        self.update_layout_data()
    
    def find_closest_grid_cell(self, position):
        """위치에서 가장 가까운 그리드 셀 좌표 찾기"""
        # 현재 그리드의 행과 열 개수 파악
        rows = 0
        cols = 0
        
        for row, col in self.grid_positions.keys():
            rows = max(rows, row + 1)
            cols = max(cols, col + 1)
        
        # 버튼 크기 계산 (첫 번째 버튼 기준)
        if self.button_positions:
            first_button = next(iter(self.button_positions.keys()))
            button_width = first_button.width()
            button_height = first_button.height()
        else:
            return None
        
        # 그리드 내 각 셀의 중심 좌표 계산 및 거리 비교
        min_distance = float('inf')
        closest_cell = None
        
        for row in range(rows):
            for col in range(cols):
                # 그리드 셀의 중심 좌표 계산
                cell_center_x = col * (button_width + 10) + button_width / 2
                cell_center_y = row * (button_height + 10) + button_height / 2
                
                # 중심 좌표와 드롭 위치 사이의 거리 계산
                distance = ((position.x() + button_width/2 - cell_center_x) ** 2 + 
                           (position.y() + button_height/2 - cell_center_y) ** 2) ** 0.5
                
                if distance < min_distance:
                    min_distance = distance
                    closest_cell = (row, col)
        
        return closest_cell
    
    def animate_button_to_grid(self, button, grid_pos):
        """버튼을 그리드 위치로 애니메이션"""
        if not grid_pos:
            return
        
        row, col = grid_pos
        
        # 그리드 내 실제 픽셀 위치 계산
        if self.button_positions:
            first_button = next(iter(self.button_positions.keys()))
            button_width = first_button.width()
            button_height = first_button.height()
            
            target_x = col * (button_width + 10)
            target_y = row * (button_height + 10)
            
            # 애니메이션 생성
            animation = QPropertyAnimation(button, b"pos")
            animation.setDuration(300)  # 300ms
            animation.setStartValue(button.pos())
            animation.setEndValue(QPoint(target_x, target_y))
            animation.setEasingCurve(QEasingCurve.OutCubic)
            
            # 애니메이션 그룹에 추가
            self.animation_group.clear()
            self.animation_group.addAnimation(animation)
            self.animation_group.start()
    
    def animate_swap(self, button1, button2, pos1, pos2):
        """두 버튼의 위치 교환 애니메이션"""
        if not pos1 or not pos2:
            return
        
        # 첫 번째 버튼 위치 계산
        row1, col1 = pos1
        if self.button_positions:
            first_button = next(iter(self.button_positions.keys()))
            button_width = first_button.width()
            button_height = first_button.height()
            
            target_x1 = col1 * (button_width + 10)
            target_y1 = row1 * (button_height + 10)
        
        # 두 번째 버튼 위치 계산
        row2, col2 = pos2
        target_x2 = col2 * (button_width + 10)
        target_y2 = row2 * (button_height + 10)
        
        # 애니메이션 생성
        animation1 = QPropertyAnimation(button1, b"pos")
        animation1.setDuration(300)
        animation1.setStartValue(button1.pos())
        animation1.setEndValue(QPoint(target_x1, target_y1))
        animation1.setEasingCurve(QEasingCurve.OutCubic)
        
        animation2 = QPropertyAnimation(button2, b"pos")
        animation2.setDuration(300)
        animation2.setStartValue(button2.pos())
        animation2.setEndValue(QPoint(target_x2, target_y2))
        animation2.setEasingCurve(QEasingCurve.OutCubic)
        
        # 애니메이션 그룹에 추가
        self.animation_group.clear()
        self.animation_group.addAnimation(animation1)
        self.animation_group.addAnimation(animation2)
        self.animation_group.start()
    
    def on_animation_finished(self):
        """애니메이션 완료 시 처리"""
        # 그리드 레이아웃에 버튼 재배치
        for button, (row, col) in self.button_positions.items():
            # 그리드 레이아웃에서 버튼 제거
            self.grid_layout.removeWidget(button)
            # 그리드 레이아웃에 버튼 추가
            self.grid_layout.addWidget(button, row, col)
    
    def on_cancel_button_clicked(self):
        """취소 버튼 클릭 시 처리"""
        self.cancel_signal.emit()
    
    def on_save_button_clicked(self):
        """저장 버튼 클릭 시 처리"""
        # 변경된 레이아웃 저장
        self.save_signal.emit(self.layout_data) 