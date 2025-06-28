from PySide6.QtCore import QObject, Signal

class TouchTestModel(QObject):
    """
    터치 테스트 모델 클래스
    
    터치 이벤트 정보를 저장하고 관리합니다.
    """
    
    # 시그널 정의
    touch_data_updated = Signal(dict)  # 터치 데이터 업데이트 시그널
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 터치 데이터 저장 변수
        self.touch_data = {
            'max_touch_count': 0,  # 최대 동시 터치 수
            'touch_points': {},    # 현재 터치 포인트 정보
            'drag_paths': [],      # 드래그 경로 정보
            'gesture_events': []   # 제스처 이벤트 기록
        }
    
    def update_max_touch_count(self, count):
        """최대 터치 수 업데이트"""
        if count > self.touch_data['max_touch_count']:
            self.touch_data['max_touch_count'] = count
            self.touch_data_updated.emit(self.touch_data)
    
    def add_touch_point(self, point_id, x, y):
        """터치 포인트 추가"""
        self.touch_data['touch_points'][point_id] = {'x': x, 'y': y}
        self.touch_data_updated.emit(self.touch_data)
    
    def update_touch_point(self, point_id, x, y):
        """터치 포인트 위치 업데이트"""
        if point_id in self.touch_data['touch_points']:
            # 이전 위치 저장
            prev_x = self.touch_data['touch_points'][point_id]['x']
            prev_y = self.touch_data['touch_points'][point_id]['y']
            
            # 새 위치 업데이트
            self.touch_data['touch_points'][point_id] = {'x': x, 'y': y}
            
            # 드래그 경로 추가
            self.touch_data['drag_paths'].append({
                'point_id': point_id,
                'from_x': prev_x,
                'from_y': prev_y,
                'to_x': x,
                'to_y': y
            })
            
            self.touch_data_updated.emit(self.touch_data)
    
    def remove_touch_point(self, point_id):
        """터치 포인트 제거"""
        if point_id in self.touch_data['touch_points']:
            del self.touch_data['touch_points'][point_id]
            self.touch_data_updated.emit(self.touch_data)
    
    def add_gesture_event(self, gesture_type, data):
        """제스처 이벤트 추가"""
        self.touch_data['gesture_events'].append({
            'type': gesture_type,
            'data': data
        })
        self.touch_data_updated.emit(self.touch_data)
    
    def clear_data(self):
        """모든 터치 데이터 초기화"""
        self.touch_data = {
            'max_touch_count': self.touch_data['max_touch_count'],  # 최대 터치 수는 유지
            'touch_points': {},
            'drag_paths': [],
            'gesture_events': []
        }
        self.touch_data_updated.emit(self.touch_data)
    
    def reset_all(self):
        """모든 데이터 완전 초기화"""
        self.touch_data = {
            'max_touch_count': 0,
            'touch_points': {},
            'drag_paths': [],
            'gesture_events': []
        }
        self.touch_data_updated.emit(self.touch_data)
    
    def get_touch_data(self):
        """현재 터치 데이터 반환"""
        return self.touch_data 