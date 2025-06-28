from PySide6.QtCore import QObject
from views.DeveloperModeView import DeveloperModeView
import os
import subprocess

class DeveloperModeController(QObject):
    def __init__(self, stack, main_controller):
        super().__init__()
        self.stack = stack
        self.main_controller = main_controller
        
        # 개발자 모드 화면 초기화
        self.developer_mode_view = DeveloperModeView()
        
        # 화면을 스택에 추가
        print("[DeveloperModeController] 개발자 모드 화면을 스택에 추가합니다...")
        self.stack.addWidget(self.developer_mode_view)
        print(f"[DeveloperModeController] 현재 스택에 있는 위젯 수: {self.stack.count()}")
        
        # 시그널 연결
        self._connect_signals()
    
    def _connect_signals(self):
        """뷰의 시그널을 컨트롤러의 메소드에 연결"""
        # 뒤로가기 버튼 시그널 연결
        self.developer_mode_view.back_signal.connect(self.show_previous_view)
        
        # 종료 버튼 시그널 연결
        self.developer_mode_view.exit_signal.connect(self.main_controller.quit_application)
        
        # 모니터링 버튼 시그널 연결
        self.developer_mode_view.monitoring_signal.connect(self.show_monitoring)
        
        # 시스템 체크 버튼 시그널 연결
        self.developer_mode_view.system_check_signal.connect(self.run_system_check)
        
        # 로그 확인 버튼 시그널 연결
        self.developer_mode_view.check_log_signal.connect(self.show_logs)
    
    def show(self):
        """개발자 모드 화면 표시"""
        # 현재 화면 저장
        self.main_controller.set_previous_view(self.stack.currentWidget())
        
        # 개발자 모드 화면이 스택에 있는지 확인
        found = False
        for i in range(self.stack.count()):
            if self.stack.widget(i) == self.developer_mode_view:
                found = True
                break
                
        if not found:
            print("[DeveloperModeController] 개발자 모드 화면이 스택에 없습니다. 다시 추가합니다.")
            # 이미 스택에 없는 경우 다시 추가
            self.stack.addWidget(self.developer_mode_view)
        
        # 개발자 모드 화면 표시
        print("[DeveloperModeController] 개발자 모드 화면으로 전환합니다.")
        self.stack.setCurrentWidget(self.developer_mode_view)
    
    def show_previous_view(self):
        """이전 화면으로 돌아가기"""
        print("[DeveloperModeController] 이전 화면으로 돌아갑니다.")
        self.main_controller.show_previous_view()
    
    def show_monitoring(self):
        """시스템 모니터링 화면 표시"""
        print("[DeveloperModeController] 시스템 모니터링 기능이 호출되었습니다.")
        # 여기에 모니터링 화면 표시 로직 구현
        # 예: 시스템 리소스 사용량, 센서 상태 등 표시
        pass
    
    def run_system_check(self):
        """시스템 자가 진단 실행"""
        print("[DeveloperModeController] 시스템 자가 진단 기능이 호출되었습니다.")
        # 여기에 시스템 자가 진단 로직 구현
        # 예: 하드웨어 상태 확인, 연결 테스트 등
        pass
    
    def show_logs(self):
        """시스템 로그 확인"""
        print("[DeveloperModeController] 로그 확인 기능이 호출되었습니다.")
        # 여기에 로그 확인 로직 구현
        # 예: 로그 파일 열기, 로그 내용 표시 등
        pass 