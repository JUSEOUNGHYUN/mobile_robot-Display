from PySide6.QtCore import QObject, Slot
from views.TouchTestView import TouchTestView
from models.TouchTestModel import TouchTestModel

class TouchTestController(QObject):
    """
    터치 테스트 컨트롤러 클래스
    
    터치 테스트 화면과 모델을 관리합니다.
    """
    
    def __init__(self, stack, main_controller):
        super().__init__()
        self.stack = stack
        self.main_controller = main_controller
        
        # 모델 초기화
        self.touch_test_model = TouchTestModel()
        
        # 뷰 초기화
        self.touch_test_view = TouchTestView()
        
        # 화면을 스택에 추가
        print("[TouchTestController] 터치 테스트 화면을 스택에 추가합니다...")
        self.stack.addWidget(self.touch_test_view)
        print(f"[TouchTestController] 현재 스택에 있는 위젯 수: {self.stack.count()}")
        
        # 시그널 연결
        self._connect_signals()
    
    def _connect_signals(self):
        """시그널 연결"""
        # 뷰 -> 컨트롤러 시그널 연결
        self.touch_test_view.back_signal.connect(self.show_previous_view)
        
        # 모델 -> 뷰 시그널 연결
        self.touch_test_model.touch_data_updated.connect(self._on_touch_data_updated)
    
    def show(self):
        """터치 테스트 화면 표시"""
        # 현재 화면 저장
        self.main_controller.set_previous_view(self.stack.currentWidget())
        
        # 터치 테스트 화면이 스택에 있는지 확인
        found = False
        for i in range(self.stack.count()):
            if self.stack.widget(i) == self.touch_test_view:
                found = True
                break
                
        if not found:
            print("[TouchTestController] 터치 테스트 화면이 스택에 없습니다. 다시 추가합니다.")
            # 이미 스택에 없는 경우 다시 추가
            self.stack.addWidget(self.touch_test_view)
        
        # 터치 테스트 화면 표시
        print("[TouchTestController] 터치 테스트 화면으로 전환합니다.")
        self.stack.setCurrentWidget(self.touch_test_view)
        
        # 화면 초기화
        self.touch_test_view.reset()
        self.touch_test_model.reset_all()
    
    def show_previous_view(self):
        """이전 화면으로 돌아가기"""
        print("[TouchTestController] 이전 화면으로 돌아갑니다.")
        self.main_controller.show_previous_view()
    
    @Slot(dict)
    def _on_touch_data_updated(self, touch_data):
        """터치 데이터 업데이트 시 호출"""
        # 필요한 경우 여기서 뷰를 업데이트할 수 있음
        pass
    
    def update_max_touch_count(self, count):
        """최대 터치 수 업데이트"""
        self.touch_test_model.update_max_touch_count(count)
    
    def add_touch_point(self, point_id, x, y):
        """터치 포인트 추가"""
        self.touch_test_model.add_touch_point(point_id, x, y)
    
    def update_touch_point(self, point_id, x, y):
        """터치 포인트 위치 업데이트"""
        self.touch_test_model.update_touch_point(point_id, x, y)
    
    def remove_touch_point(self, point_id):
        """터치 포인트 제거"""
        self.touch_test_model.remove_touch_point(point_id)
    
    def add_gesture_event(self, gesture_type, data):
        """제스처 이벤트 추가"""
        self.touch_test_model.add_gesture_event(gesture_type, data)
    
    def clear_data(self):
        """모든 터치 데이터 초기화"""
        self.touch_test_model.clear_data()
        self.touch_test_view.clearPaths()
    
    def reset_all(self):
        """모든 데이터 완전 초기화"""
        self.touch_test_model.reset_all()
        self.touch_test_view.reset() 