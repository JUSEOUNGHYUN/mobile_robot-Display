from PySide6.QtCore import QObject
from views.SettingsView import SettingsView
from PySide6.QtWidgets import QLabel
from PySide6.QtGui import QPixmap, QMouseEvent

class SettingsController(QObject):
    def __init__(self, stack, main_controller):
        super().__init__()
        self.stack = stack
        self.main_controller = main_controller
        
        # 설정 화면 초기화
        self.settings_view = SettingsView()
        
        # 화면을 스택에 추가
        print("[SettingsController] 설정 화면을 스택에 추가합니다...")
        self.stack.addWidget(self.settings_view)
        print(f"[SettingsController] 현재 스택에 있는 위젯 수: {self.stack.count()}")
        
        # 시그널 연결
        print("[SettingsController] settings_view.back_signal 연결 시작")
        self.settings_view.back_signal.connect(self.show_previous_view)
        print("[SettingsController] settings_view.back_signal 연결 완료")
        
        # 정보 버튼 시그널 연결
        self.settings_view.info_signal.connect(self.show_info_view)
        print("[SettingsController] settings_view.info_signal 연결 완료")
        
        # 와이파이 버튼 시그널 연결
        self.settings_view.wifi_signal.connect(self.show_wifi_view)
        print("[SettingsController] settings_view.wifi_signal 연결 완료")
        
        # 상태바의 뒤로가기 버튼 직접 연결
        self.settings_view.status_bar.back_button.clicked.connect(self.show_previous_view)
        print("[SettingsController] 뒤로가기 버튼 직접 연결 완료")
        
        # 종료 시그널 연결 추가
        self.settings_view.exit_signal.connect(self.main_controller.quit_application)
        print("[SettingsController] settings_view.exit_signal 연결 완료")
        
        # 이동 옵션 버튼 시그널 연결
        self.settings_view.move_option_signal.connect(self.show_move_option_view)
        print("[SettingsController] settings_view.move_option_signal 연결 완료")
        
        # 맵 버튼 시그널 연결
        self.settings_view.map_signal.connect(self.show_map_image_control)
        print("[SettingsController] settings_view.map_signal 연결 완료")
        
        # 터치 테스트 버튼 시그널 연결
        self.settings_view.touch_test_signal.connect(self.show_touch_test_view)
        print("[SettingsController] settings_view.touch_test_signal 연결 완료")
        
        # 개발자 모드 버튼 시그널 연결
        self.settings_view.developer_mode_signal.connect(self.show_developer_mode_view)
        print("[SettingsController] settings_view.developer_mode_signal 연결 완료")
        
        # 기타 설정 버튼 시그널 연결
        self.settings_view.other_settings_signal.connect(self.show_other_settings_view)
        print("[SettingsController] settings_view.other_settings_signal 연결 완료")
        
    def show_settings_view(self):
        """설정 화면 표시"""
        # 현재 화면 저장
        self.main_controller.set_previous_view(self.stack.currentWidget())
        
        # 설정 화면이 스택에 있는지 확인
        found = False
        for i in range(self.stack.count()):
            if self.stack.widget(i) == self.settings_view:
                found = True
                break
                
        if not found:
            print("[SettingsController] 설정 화면이 스택에 없습니다. 다시 추가합니다.")
            # 이미 스택에 없는 경우 다시 추가
            self.stack.addWidget(self.settings_view)
        
        # 설정 화면 표시
        print("[SettingsController] 설정 화면으로 전환합니다.")
        self.stack.setCurrentWidget(self.settings_view)
    
    def show_info_view(self):
        """정보 화면 표시"""
        print("[SettingsController] 정보 화면으로 전환합니다.")
        
        # 현재 설정 화면을 이전 화면으로 저장
        self.main_controller.set_previous_view(self.settings_view)
        
        # InfoController에 위임
        if hasattr(self.main_controller, 'info_controller'):
            self.main_controller.info_controller.show()
        
    def show_previous_view(self):
        """이전 화면으로 돌아가기"""
        print("[SettingsController] 이전 화면으로 돌아갑니다. main_controller.show_previous_view() 호출")
        self.main_controller.show_previous_view()
        print("[SettingsController] 이전 화면으로 돌아가기 완료")
        
    def show_wifi_view(self):
        """와이파이 설정 화면 표시"""
        print("[SettingsController] 와이파이 설정 화면으로 전환합니다.")
        
        # 현재 설정 화면을 이전 화면으로 저장
        self.main_controller.set_previous_view(self.settings_view)
        
        # WiFiController에 위임
        if hasattr(self.main_controller, 'wifi_controller'):
            self.main_controller.wifi_controller.show()
        
    def show_move_option_view(self):
        """이동 옵션 설정 화면 표시"""
        print("[SettingsController] 이동 옵션 화면으로 전환합니다.")
        
        # 현재 설정 화면을 이전 화면으로 저장
        self.main_controller.set_previous_view(self.settings_view)
        
        # MoveOptionController에 위임
        if hasattr(self.main_controller, 'move_option_controller'):
            self.main_controller.move_option_controller.show()
        
    def show_map_image_control(self):
        """맵 이미지 컨트롤 화면 표시"""
        print("[SettingsController] 맵 이미지 컨트롤 화면으로 전환합니다.")
        
        # 현재 설정 화면을 이전 화면으로 저장
        self.main_controller.set_previous_view(self.settings_view)
        
        # MapImageControlController에 위임
        if hasattr(self.main_controller, 'map_image_control_controller'):
            # 저장/취소 시그널 연결
            self.main_controller.map_image_control_controller.save_complete_signal.connect(self.on_map_image_save_complete)
            self.main_controller.map_image_control_controller.cancel_signal.connect(self.on_map_image_cancel)
            
            # 화면 표시
            self.main_controller.map_image_control_controller.show()
            
    def show_touch_test_view(self):
        """터치 테스트 화면 표시"""
        print("[SettingsController] 터치 테스트 화면으로 전환합니다.")
        
        # 현재 설정 화면을 이전 화면으로 저장
        self.main_controller.set_previous_view(self.settings_view)
        
        # TouchTestController에 위임
        if hasattr(self.main_controller, 'touch_test_controller'):
            self.main_controller.touch_test_controller.show()
    
    def show_developer_mode_view(self):
        """개발자 모드 화면 표시"""
        print("[SettingsController] 개발자 모드 화면으로 전환합니다.")
        
        # 현재 설정 화면을 이전 화면으로 저장
        self.main_controller.set_previous_view(self.settings_view)
        
        # DeveloperModeController에 위임
        if hasattr(self.main_controller, 'developer_mode_controller'):
            self.main_controller.developer_mode_controller.show()

    def show_other_settings_view(self):
        """기타 설정 화면 표시"""
        print("[SettingsController] 기타 설정 화면으로 전환합니다.")
        
        # 현재 설정 화면을 이전 화면으로 저장
        self.main_controller.set_previous_view(self.settings_view)
        
        # OtherSettingController에 위임
        if hasattr(self.main_controller, 'other_setting_controller'):
            self.main_controller.other_setting_controller.show()

    def on_map_image_save_complete(self, success, message):
        """맵 이미지 변환 정보 저장 완료 시 호출"""
        # 시그널 연결 해제
        self.main_controller.map_image_control_controller.save_complete_signal.disconnect(self.on_map_image_save_complete)
        self.main_controller.map_image_control_controller.cancel_signal.disconnect(self.on_map_image_cancel)
        
        # 설정 화면으로 돌아가기
        self.stack.setCurrentWidget(self.settings_view)

    def on_map_image_cancel(self):
        """맵 이미지 변환 정보 저장 취소 시 호출"""
        # 시그널 연결 해제
        self.main_controller.map_image_control_controller.save_complete_signal.disconnect(self.on_map_image_save_complete)
        self.main_controller.map_image_control_controller.cancel_signal.disconnect(self.on_map_image_cancel)
        
        # 설정 화면으로 돌아가기
        self.stack.setCurrentWidget(self.settings_view)

    def connect_setting_icons(self, views):
        """설정 아이콘 클릭 시그널 연결
        
        각 뷰의 설정 아이콘 클릭 시그널을 이 컨트롤러의 show_settings_view 메소드에 연결합니다.
        MVC 패턴에 따라 뷰의 시그널만 처리하고 뷰의 내부 구현에는 간섭하지 않습니다.
        
        Args:
            views (list): 설정 아이콘 시그널을 연결할 뷰 객체 리스트
        """
        for view in views:
            if hasattr(view, 'status_bar') and hasattr(view.status_bar, 'setting_clicked'):
                # 뷰의 상태바에 setting_clicked 시그널이 있으면 연결
                view.status_bar.setting_clicked.connect(self.show_settings_view)
                print(f"[SettingsController] {view.__class__.__name__}의 설정 아이콘 시그널 연결 완료")

    def _connect_signals(self):
        """뷰의 시그널을 컨트롤러의 메소드에 연결"""
        # 뷰의 버튼 클릭 시그널 연결
        self.settings_view.move_option_btn.clicked.connect(self.main_controller.move_option_controller.show)
        # 여기에 맵 이미지 제어 버튼 클릭 시그널 연결
        if hasattr(self.settings_view, 'map_image_control_btn') and self.settings_view.map_image_control_btn is not None:
            self.settings_view.map_image_control_btn.clicked.connect(self.main_controller.show_map_image_control)
        # 뒤로가기 버튼 클릭 시그널 연결
        self.settings_view.back_button.clicked.connect(self.show_previous_view) 