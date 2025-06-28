from PySide6.QtCore import QObject, QTimer
from views.OtherSettingView import OtherSettingView
from Utils.MessageBox import MessageBox
import os
import json

class OtherSettingController(QObject):
    def __init__(self, stack, main_controller):
        super().__init__()
        self.stack = stack
        self.main_controller = main_controller
        
        # 기타 설정 화면 초기화
        self.other_setting_view = OtherSettingView()
        
        # 화면을 스택에 추가
        print("[OtherSettingController] 기타 설정 화면을 스택에 추가합니다...")
        self.stack.addWidget(self.other_setting_view)
        print(f"[OtherSettingController] 현재 스택에 있는 위젯 수: {self.stack.count()}")
        
        # 시그널 연결
        self.other_setting_view.back_signal.connect(self.show_previous_view)
        self.other_setting_view.exit_signal.connect(self.main_controller.quit_application)
        self.other_setting_view.area_index_view_changed.connect(self.on_area_index_view_changed)
        
        # 설정 파일 경로
        self.settings_file_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                            "data", "other_settings.json")
        
        # 설정 불러오기
        self.load_settings()
        
        # 초기화 시 LocationAddView에 설정 적용 (지연 실행)
        QTimer.singleShot(500, lambda: self.init_apply_settings())
        
    def init_apply_settings(self):
        """초기화 시 LocationAddView에 설정 적용"""
        try:
            # 설정 파일에서 영역 인덱스 표시 여부 확인
            show_area_index = False  # 기본값은 False (인덱스 비표시)
            
            if os.path.exists(self.settings_file_path):
                with open(self.settings_file_path, 'r') as f:
                    settings = json.load(f)
                    if 'show_area_index' in settings:
                        show_area_index = settings['show_area_index']
            
            # LocationAddView에 설정 적용
            self.apply_area_index_visibility_to_views(show_area_index)
            print(f"[OtherSettingController] 초기화 시 영역 인덱스 표시 설정 적용: {show_area_index}")
        except Exception as e:
            print(f"[OtherSettingController] 초기화 시 설정 적용 오류: {e}")
            
    def show(self):
        """기타 설정 화면 표시"""
        # 현재 화면 저장
        self.main_controller.set_previous_view(self.stack.currentWidget())
        
        # 기타 설정 화면이 스택에 있는지 확인
        found = False
        for i in range(self.stack.count()):
            if self.stack.widget(i) == self.other_setting_view:
                found = True
                break
                
        if not found:
            print("[OtherSettingController] 기타 설정 화면이 스택에 없습니다. 다시 추가합니다.")
            # 이미 스택에 없는 경우 다시 추가
            self.stack.addWidget(self.other_setting_view)
        
        # 설정 불러오기
        self.load_settings()
        
        # 현재 설정 값을 LocationAddView에 적용
        if hasattr(self.other_setting_view.ui, 'area_index_view_cb'):
            is_checked = self.other_setting_view.ui.area_index_view_cb.isChecked()
            print(f"[OtherSettingController] show 메서드에서 현재 체크박스 상태: {is_checked}")
            self.apply_area_index_visibility_to_views(is_checked)
        
        # 기타 설정 화면 표시
        print("[OtherSettingController] 기타 설정 화면으로 전환합니다.")
        self.stack.setCurrentWidget(self.other_setting_view)
    
    def show_previous_view(self):
        """이전 화면으로 돌아가기"""
        print("[OtherSettingController] 이전 화면으로 돌아갑니다.")
        self.main_controller.show_previous_view()
        
    def on_area_index_view_changed(self, is_checked):
        """영역 인덱스 표시 설정 변경"""
        print(f"[OtherSettingController] 영역 인덱스 표시 설정 변경 수신: {is_checked}, 타입: {type(is_checked)}")
        
        # 설정 저장
        self.save_settings({"show_area_index": is_checked})
        print(f"[OtherSettingController] 설정 저장 완료: show_area_index={is_checked}")
        
        # 모든 LocationAddView에 설정 적용
        self.apply_area_index_visibility_to_views(is_checked)
        
    def load_settings(self):
        """설정 파일 불러오기"""
        try:
            if os.path.exists(self.settings_file_path):
                with open(self.settings_file_path, 'r') as f:
                    settings = json.load(f)
                    
                # 영역 인덱스 표시 설정 적용
                if 'show_area_index' in settings:
                    show_area_index = settings['show_area_index']
                    self.other_setting_view.ui.area_index_view_cb.setChecked(show_area_index)
                    print(f"[OtherSettingController] 영역 인덱스 표시 설정 불러옴: {show_area_index}")
                    
                    # 설정 적용
                    self.apply_area_index_visibility_to_views(show_area_index)
            else:
                # 기본 설정 저장
                self.save_settings({"show_area_index": True})
                print("[OtherSettingController] 설정 파일이 없어 기본 설정을 저장합니다.")
        except Exception as e:
            print(f"[OtherSettingController] 설정 불러오기 오류: {e}")
            # 오류 발생 시 기본 설정 적용
            self.other_setting_view.ui.area_index_view_cb.setChecked(True)
            
    def save_settings(self, settings_to_update):
        """설정 파일 저장"""
        try:
            # 기존 설정 불러오기
            current_settings = {}
            if os.path.exists(self.settings_file_path):
                with open(self.settings_file_path, 'r') as f:
                    current_settings = json.load(f)
            
            # 설정 업데이트
            current_settings.update(settings_to_update)
            
            # 설정 저장
            os.makedirs(os.path.dirname(self.settings_file_path), exist_ok=True)
            with open(self.settings_file_path, 'w') as f:
                json.dump(current_settings, f, indent=4)
                
            print(f"[OtherSettingController] 설정 저장 완료: {settings_to_update}")
        except Exception as e:
            print(f"[OtherSettingController] 설정 저장 오류: {e}")
            MessageBox.show_error(self.other_setting_view, f"설정 저장 중 오류가 발생했습니다: {e}")
            
    def apply_area_index_visibility_to_views(self, show_index):
        """모든 LocationAddView에 영역 인덱스 표시 설정 적용
        
        Args:
            show_index (bool): 인덱스 표시 여부
        """
        print(f"[OtherSettingController] 모든 LocationAddView에 영역 인덱스 표시 설정 적용: {show_index}")
        
        # LocationAddController가 있는지 확인
        if hasattr(self.main_controller, 'location_add_controller'):
            # LocationAddView에 설정 적용
            location_add_view = self.main_controller.location_add_controller.location_add_view
            if hasattr(location_add_view, 'set_area_index_visibility'):
                location_add_view.set_area_index_visibility(show_index)
                print("[OtherSettingController] LocationAddView에 영역 인덱스 표시 설정 적용 완료")
            else:
                print("[OtherSettingController] LocationAddView에 set_area_index_visibility 메서드가 없습니다.")
        else:
            print("[OtherSettingController] main_controller에 location_add_controller가 없습니다.") 