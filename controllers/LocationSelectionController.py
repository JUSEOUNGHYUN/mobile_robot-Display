from PySide6.QtCore import QObject, Signal
from views.LocationSelectionView import LocationSelectionView
from Utils.MessageBox import MessageBox
from models.RingoBellManager import RingoBellManager

class LocationSelectionController(QObject):
    def __init__(self, stack, selection_model, main_controller):
        super().__init__()
        self.stack = stack
        self.selection_model = selection_model
        self.main_controller = main_controller
        
        # 목적지 선택 화면 초기화
        self.selection_view = LocationSelectionView()
        self.stack.addWidget(self.selection_view)
        
        # View 시그널 연결
        self.selection_view.add_location_signal.connect(self.on_add_location)
        self.selection_view.edit_location_signal.connect(self.on_edit_location)
        self.selection_view.delete_location_signal.connect(self.on_delete_location)
        self.selection_view.start_moving_signal.connect(self.on_start_moving)
        self.selection_view.location_selected_signal.connect(self.on_location_selected)
        self.selection_view.change_layout_signal.connect(self.on_change_layout)
        
        # 모델 시그널 연결
        self.selection_model.locations_updated.connect(self.on_locations_updated)
        self.selection_model.delete_result.connect(self.on_delete_result)
        
        # 초기 데이터 로드
        self.update_locations()
    
    def show(self):
        """목적지 선택 화면 표시"""
        self.update_locations()
        self.stack.setCurrentWidget(self.selection_view)
    
    def update_locations(self):
        """목적지 목록 업데이트"""
        self.selection_model.refresh()
    
    def on_locations_updated(self, locations):
        """위치 목록이 업데이트되었을 때 처리"""
        self.selection_view.update_locations(locations)
    
    def on_location_selected(self, location_data):
        """위치가 선택되었을 때 처리"""
        # 모델에 선택된 위치 정보 전달
        location_index = self.selection_model.find_location_id(location_data)
        if location_index is not None:
            self.selection_model.select_location(location_index)
    
    def on_add_location(self):
        """목적지 추가 화면으로 이동"""
        self.main_controller.show_add_location()
    
    def on_edit_location(self, location_data):
        """목적지 수정 화면으로 이동"""
        self.main_controller.show_edit_location(location_data)
    
    def on_delete_location(self, location_data):
        """목적지 삭제"""
        location_id = self.selection_model.find_location_id(location_data)
        if location_id is not None:
            # 삭제 작업 수행
            self.selection_model.delete_location(location_id)
    
    def on_delete_result(self, success, message):
        """삭제 결과 처리"""
        if success:
            # 삭제 성공 시 선택 상태 초기화
            self.selection_view.clear_selection()
            # UUID 캐시 새로고침
            RingoBellManager.instance().refresh_uuids()
        else:
            # 삭제 실패 시 에러 메시지 표시
            MessageBox.show_error(self.selection_view, message)
    
    def on_start_moving(self, location):
        """이동 시작"""
        if hasattr(self.main_controller, 'moving_controller'):
            # 도착 컨트롤러의 위치 고정 상태만 확인 (현재 화면과 무관하게)
            if (hasattr(self.main_controller, 'arrive_controller') and 
                self.main_controller.arrive_controller.is_fixed()):
                return
                
            print(f"[LocationSelectionController] 목적지로 이동 요청: {location.get('name', '목적지')}")
            self.main_controller.moving_controller.start_moving(location)
    
    def search_locations(self, search_text):
        """위치 검색"""
        self.selection_model.search_locations(search_text)
            
    def connect_to_moving_controller(self, moving_controller):
        """이동 컨트롤러 연결"""
        self.moving_controller = moving_controller
        print(f"[LocationSelectionController] 이동 컨트롤러 연결됨: {moving_controller is not None}")
        
        # 움직임 관련 시그널 연결 - 기존 연결을 끊고 새로 연결
        try:
            # 기존 연결이 있으면 제거
            self.selection_view.start_moving_signal.disconnect()
        except:
            # 연결이 없으면 무시
            pass
        
        # 새로 연결
        self.selection_view.start_moving_signal.connect(moving_controller.start_moving)
        
    def connect_to_settings_controller(self, settings_controller):
        """설정 컨트롤러 연결"""
        self.settings_controller = settings_controller
        print(f"[LocationSelectionController] 설정 컨트롤러 연결됨: {settings_controller is not None}")
        
    def on_change_layout(self):
        """레이아웃 변경 화면으로 이동"""
        if hasattr(self.main_controller, 'change_layout_controller'):
            self.main_controller.change_layout_controller.show()
        else:
            print("[LocationSelectionController] change_layout_controller가 없습니다.")