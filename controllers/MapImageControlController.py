"""
MapImageControlController 모듈은 맵 이미지 변환 모델과 뷰를 연결하는 컨트롤러를 제공합니다.
"""

import os
from PySide6.QtCore import QObject, Signal, QPointF, Qt
from PySide6.QtGui import QPixmap
from models.MapImageControlModel import MapImageControlModel
from views.MapImageControlView import MapImageControlView
from Utils.MessageBox import MessageBox
from PySide6.QtWidgets import QMessageBox

class MapImageControlController(QObject):
    """맵 이미지 컨트롤 컨트롤러
    
    맵 이미지 변환 모델과 뷰를 연결하는 컨트롤러 클래스입니다.
    """
    # 시그널 정의
    save_complete_signal = Signal(bool, str)  # 저장 완료 시그널 (성공 여부, 메시지)
    cancel_signal = Signal()  # 취소 시그널
    
    def __init__(self, stack):
        super().__init__()
        
        # 스택 위젯 참조 저장
        self.stack = stack
        
        # 모델 및 뷰 초기화
        self.model = MapImageControlModel()
        self.view = MapImageControlView()
        
        # 스택에 뷰 추가
        self.stack.addWidget(self.view)
        
        # 이벤트 연결
        self.setup_connections()
    
    def setup_connections(self):
        """모델과 뷰 연결"""
        # 뷰 -> 컨트롤러 시그널 연결
        self.view.save_clicked.connect(self.on_save_clicked)
        self.view.cancel_clicked.connect(self.on_cancel_clicked)
        self.view.move_image_signal.connect(self.on_move_image)
        self.view.scale_changed_signal.connect(self.on_scale_changed)
        self.view.rotation_changed_signal.connect(self.on_rotation_changed)
        self.view.refresh_clicked.connect(self.on_refresh_clicked)
        
        # 모델 -> 컨트롤러 시그널 연결
        self.model.data_loaded.connect(self.on_data_loaded)
        self.model.data_saved.connect(self.on_data_saved)
        
        # 스택 위젯 시그널 연결
        self.stack.currentChanged.connect(self.on_stack_changed)
    
    def show(self):
        """화면 표시"""
        try:
            print("MapImageControl 화면 표시 시작...")
            
            # ROS2 초기화 및 맵 구독 시작
            if not hasattr(self.view, 'map_subscriber') or self.view.map_subscriber is None:
                print("ROS2 초기화 및 맵 구독 시작...")
                self.view.setup_ros()
            
            # 맵 이미지(오버레이) 로드
            map_image_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                        "resources", "map_original_image.png")
            map_image = QPixmap(map_image_path)
            
            if map_image.isNull():
                print(f"이미지를 로드할 수 없음: {map_image_path}")
                MessageBox.show_error(None, "맵 이미지를 로드할 수 없습니다.")
                return False
            
            # 맵 이미지 설정
            self.view.set_map_image(map_image)
            
            # 저장된 맵 변환 정보 로드
            transform_data = self.model.load_transform_data()
            
            # 변환 정보 적용 (transform_data가 None이어도 기본값 사용)
            self.view.set_transform_data(transform_data)
            
            # 스택 전환
            self.stack.setCurrentWidget(self.view)
            
            # 맵 구독 활성화
            if hasattr(self.view, 'map_subscriber'):
                self.view.map_subscriber.set_view_active(True)
            
            print("MapImageControl 화면 표시 완료")
            return True
            
        except Exception as e:
            print(f"맵 이미지 컨트롤러 화면 표시 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
            MessageBox.show_error(None, "맵 이미지 조정 화면을 표시할 수 없습니다.")
            return False

    def on_stack_changed(self, index):
        """스택 위젯 변경 시 호출"""
        try:
            # 현재 위젯이 MapImageControlView인지 확인
            current_widget = self.stack.widget(index)
            is_map_control_view = isinstance(current_widget, MapImageControlView)
            
            # 맵 구독 상태 업데이트
            if hasattr(self.view, 'map_subscriber'):
                self.view.map_subscriber.set_view_active(is_map_control_view)
                print(f"맵 구독 상태 변경: {'활성화' if is_map_control_view else '비활성화'}")
                
        except Exception as e:
            print(f"스택 변경 처리 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()

    def on_save_clicked(self, transform_data):
        """저장 버튼 클릭 시 호출"""
        # 변환 정보 저장
        self.model.save_transform_data(transform_data)
        # 이전 화면으로 돌아가기
        self.cancel_signal.emit()
    
    def on_cancel_clicked(self):
        """취소 버튼 클릭 시 호출"""
        # 맵 구독 비활성화
        if hasattr(self.view, 'map_subscriber'):
            self.view.map_subscriber.set_view_active(False)
        
        # 이전 화면으로 돌아가기
        self.cancel_signal.emit()
    
    def on_move_image(self, dx, dy):
        """이미지 이동 시 호출"""
        # 현재 변환 위치 가져오기
        current_transform = self.view.get_transform_data()
        current_translation = QPointF(
            current_transform['translate_x'],
            current_transform['translate_y']
        )
        
        # 새 위치 계산
        new_translation = QPointF(
            current_translation.x() + dx,
            current_translation.y() + dy
        )
        
        # 뷰 업데이트
        self.view.update_transform(translation=new_translation)
    
    def on_scale_changed(self, scale):
        """크기 변경 시 호출"""
        # 뷰 업데이트
        self.view.update_transform(scale=scale)
    
    def on_rotation_changed(self, angle):
        """회전 변경 시 호출"""
        # 뷰 업데이트
        self.view.update_transform(rotation=angle)
    
    def on_refresh_clicked(self):
        """새로고침 버튼 클릭 시 호출"""
        try:
            # 뷰의 변환만 초기화 (저장하지 않음)
            self.view.update_transform(
                translation=QPointF(0, 0),
                scale=1.0,
                rotation=0
            )
            print("맵 이미지 변환이 초기화되었습니다.")
            
        except Exception as e:
            print(f"맵 이미지 초기화 중 오류 발생: {str(e)}")
            MessageBox.show_error(self.view, "맵 이미지 초기화 중 오류가 발생했습니다.")
    
    # 이벤트 핸들러 - 모델에서 발생한 이벤트
    def on_data_loaded(self, transform_data):
        """변환 데이터 로드 완료 시 호출"""
        self.view.set_transform_data(transform_data)
    
    def on_data_saved(self, success, message):
        """데이터 저장 완료 처리"""
        if success:
            self.view.close()
        else:
            QMessageBox.warning(self.view, "저장 실패", message)
            
            # 저장 완료 시그널 발생
            self.save_complete_signal.emit(False, message) 