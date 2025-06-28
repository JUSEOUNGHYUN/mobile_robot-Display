from PySide6.QtCore import QObject, Signal
from views.ChangeLayoutView import ChangeLayoutView
from Utils.MessageBox import MessageBox

class ChangeLayoutController(QObject):
    """목적지 레이아웃 변경 컨트롤러"""
    
    # 레이아웃 변경 완료 시그널
    layout_changed = Signal(list)
    
    def __init__(self, stack, selection_model):
        super().__init__()
        self.stack = stack
        self.selection_model = selection_model
        
        # 레이아웃 변경 화면 초기화
        self.change_layout_view = ChangeLayoutView()
        self.stack.addWidget(self.change_layout_view)
        
        # 뷰 시그널 연결
        self.change_layout_view.cancel_signal.connect(self.on_cancel)
        self.change_layout_view.save_signal.connect(self.on_save)
    
    def show(self):
        """레이아웃 변경 화면 표시"""
        # 현재 목적지 목록 가져오기
        locations = self.selection_model.get_all_locations()
        # 화면에 위치 목록 전달
        self.change_layout_view.update_locations(locations)
        # 화면 전환
        self.stack.setCurrentWidget(self.change_layout_view)
    
    def on_cancel(self):
        """취소 버튼 처리"""
        # 목적지 선택 화면으로 돌아가기
        self.hide()
    
    def on_save(self, layout_data):
        """저장 버튼 처리"""
        try:
            # 위치 데이터 정렬
            sorted_locations = sorted(layout_data, key=lambda x: x['position_index'])
            
            # 정렬된 위치 데이터로 위치 목록 업데이트
            locations = self.selection_model.get_all_locations()
            layout_map = {item['location_data'].get('name', ''): item['position_index'] for item in layout_data}
            
            # 위치 데이터를 새 순서로 재배열
            locations.sort(key=lambda loc: layout_map.get(loc.get('name', ''), 999))
            
            # 파일에 저장
            with open(self.selection_model.file_path, 'w') as file:
                import json
                json.dump(locations, file, indent=2)
            
            # 모델 데이터 갱신
            self.selection_model.refresh()
            
            # 저장 성공 메시지
            MessageBox.show_message(
                self.change_layout_view,
                title="저장 완료",
                message="목적지 배치가 성공적으로 저장되었습니다.",
                button_labels=["확인"]
            )
            
            # 변경 완료 시그널 발생
            self.layout_changed.emit(locations)
            
            # 화면 닫기
            self.hide()
            
        except Exception as e:
            # 오류 메시지
            MessageBox.show_error(
                self.change_layout_view,
                f"저장 중 오류가 발생했습니다: {str(e)}"
            )
    
    def hide(self):
        """레이아웃 변경 화면 숨기기"""
        # 현재 화면이 레이아웃 변경 화면인 경우에만 처리
        if self.stack.currentWidget() == self.change_layout_view:
            # 목적지 선택 화면으로 돌아가기
            for i in range(self.stack.count()):
                widget = self.stack.widget(i)
                if widget.__class__.__name__ == "LocationSelectionView":
                    self.stack.setCurrentWidget(widget)
                    print("[ChangeLayoutController] 목적지 선택 화면으로 돌아갑니다.")
                    return
                    
            # LocationSelectionView를 찾지 못한 경우, 대체 로직
            print("[ChangeLayoutController] 목적지 선택 화면을 찾을 수 없습니다. 기본 화면으로 돌아갑니다.")
            # 스택의 첫 번째 위젯으로 돌아가기
            self.stack.setCurrentIndex(0) 