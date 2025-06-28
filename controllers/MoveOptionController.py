from PySide6.QtCore import QObject, Slot
from views.MoveOptionView import MoveOptionView

class MoveOptionController(QObject):
    """이동 옵션 설정 컨트롤러 클래스"""
    
    def __init__(self, stack, main_controller=None):
        """초기화"""
        super().__init__()
        self.stack = stack  # 스택 위젯 추가
        self.main_controller = main_controller
        self.view = None
        
        # 설정 데이터
        self.ignore_callbell = False
        self.accel_rate = 0
        self.decel_rate = 0
        self.brake_mode = False
        self.linear_speed = 0.1
        self.angular_speed = 0.5
        
        # 뷰 초기화 및 설정
        self.setup_view()
    
    def setup_view(self):
        """뷰 설정"""
        self.view = MoveOptionView()
        
        # 뷰 시그널 연결
        self.view.back_signal.connect(self.handle_back)
        self.view.save_signal.connect(self.handle_save)
        
        # 초기 설정값 로드
        self.load_settings()
        
        # 스택에 뷰 추가
        self.stack.addWidget(self.view)
        
        return self.view
    
    def show(self):
        """뷰 표시 (SettingsController와 호환성 유지)"""
        if self.view and self.stack:
            self.stack.setCurrentWidget(self.view)
    
    def show_view(self):
        """뷰 표시 (별칭)"""
        self.show()
    
    def load_settings(self):
        """설정 로드 및 뷰에 적용"""
        # 여기서 설정을 로드하는 코드가 들어갈 수 있습니다.
        # 예: 파일에서 설정 로드 또는 ROS 파라미터에서 로드
        
        # 뷰에 설정 적용 - 메서드가 있는지 확인 후 호출
        if self.view:
            # 호출벨 무시 설정
            if hasattr(self.view, 'set_ignore_state'):
                self.view.set_ignore_state(self.ignore_callbell)
            
            # 가속도 설정
            if hasattr(self.view, 'set_accel_rate'):
                self.view.set_accel_rate(self.accel_rate)
            
            # 감속도 설정
            if hasattr(self.view, 'set_decel_rate'):
                self.view.set_decel_rate(self.decel_rate)
            
            # 브레이크 모드 설정
            if hasattr(self.view, 'set_brake_mode'):
                self.view.set_brake_mode(self.brake_mode)
            
            # 직진 속도 설정
            if hasattr(self.view, 'set_linear_speed'):
                self.view.set_linear_speed(self.linear_speed)
            # 대체 메서드 시도
            elif hasattr(self.view.ui, 'linear_speed_usr_Slider'):
                self.view.ui.linear_speed_usr_Slider.setValue(int(self.linear_speed * 100))
            
            # 회전 속도 설정
            if hasattr(self.view, 'set_angular_speed'):
                self.view.set_angular_speed(self.angular_speed)
            # 대체 메서드 시도
            elif hasattr(self.view.ui, 'angular_speed_usr_Slider'):
                self.view.ui.angular_speed_usr_Slider.setValue(int(self.angular_speed * 100))
    
    @Slot()
    def handle_back(self):
        """뒤로가기 처리"""
        if self.main_controller:
            self.main_controller.show_previous_view()
    
    @Slot(bool, int, int, bool, float, float)
    def handle_save(self, ignore_callbell, accel_rate, decel_rate, brake_mode, linear_speed, angular_speed):
        """저장 처리"""
        # 설정 저장
        self.ignore_callbell = ignore_callbell
        self.accel_rate = accel_rate
        self.decel_rate = decel_rate
        self.brake_mode = brake_mode
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        
        # 설정 저장 로직 (파일 저장, ROS 파라미터 설정 등)
        self.save_settings()
        
        # 메인 컨트롤러에 알림
        if self.main_controller:
            self.main_controller.show_previous_view()
    
    def save_settings(self):
        """설정 저장"""
        # 여기서 설정을 저장하는 코드가 들어갈 수 있습니다.
        # 예: 파일에 설정 저장 또는 ROS 파라미터로 설정
        
        print("[MoveOptionController] 설정 저장:")
        print(f"  - 호출벨 무시: {self.ignore_callbell}")
        print(f"  - 가속도: {self.accel_rate}")
        print(f"  - 감속도: {self.decel_rate}")
        print(f"  - 브레이크 모드: {self.brake_mode}")
        print(f"  - 직진 속도: {self.linear_speed}")
        print(f"  - 회전 속도: {self.angular_speed}")
        
        # 여기에 실제 저장 로직 구현
        # 예: ROS 파라미터 설정 또는 설정 파일 저장