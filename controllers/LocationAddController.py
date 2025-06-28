from PySide6.QtCore import QObject, QTimer, QPropertyAnimation, QEasingCurve
from views.LocationAddView import LocationAddView
from controllers.MapImageControlController import MapImageControlController
from Utils.MessageBox import MessageBox
from Utils.RosActions import ROS2ActionManager
from models.LocationAddModel import LocationAddModel
from models.RingoBellManager import RingoBellManager
from Utils.RosTopic import KeepoutZonePublisher
from Utils.Toast import ToastManager
from PySide6.QtGui import QPixmap
import os
import json

class LocationAddController(QObject):
    def __init__(self, stack, location_model, main_controller):
        super().__init__()
        self.stack = stack
        self.selection_model = location_model  # 선택 모델은 유지
        self.model = LocationAddModel()  # 추가 모델 생성
        self.main_controller = main_controller
        
        # View 초기화
        self.view = LocationAddView()
        self.stack.addWidget(self.view)
        
        # 맵 변환 정보 파일 경로
        self.map_transform_file = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                             "data", "map_transform.json")
        
        # 맵 이미지 파일 경로
        self.map_image_path = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                                         "resources", "map_original_image.png")
        
        # ROS2 액션 매니저 초기화
        self.action_manager = None
        self.ros2_initialized = False
        
        # 금지 구역 발행자 초기화
        self.keepout_zone_publisher = KeepoutZonePublisher()
        
        # 시그널 연결
        self.setup_connections()

    def setup_connections(self):
        """시그널 연결"""
        # View 시그널
        self.view.save_signal.connect(self.on_save_location)
        self.view.cancel_signal.connect(self.on_cancel)
        self.view.position_clicked.connect(self.on_position_clicked)
        
        # Model 시그널
        self.model.save_result.connect(self.on_save_result)
        self.model.file_updated.connect(self.on_file_updated)
        
        # 맵 컨트롤 시그널 연결
        self.view.map_zoom_in_signal.connect(self.on_map_zoom_in)
        self.view.map_zoom_out_signal.connect(self.on_map_zoom_out)
        self.view.map_move_up_signal.connect(self.on_map_move_up)
        self.view.map_move_down_signal.connect(self.on_map_move_down)
        self.view.map_move_left_signal.connect(self.on_map_move_left)
        self.view.map_move_right_signal.connect(self.on_map_move_right)
        
        # 로봇 컨트롤 시그널 연결
        self.view.robot_move_up_signal.connect(self.on_robot_move_up)
        self.view.robot_move_down_signal.connect(self.on_robot_move_down)
        self.view.robot_move_left_signal.connect(self.on_robot_move_left)
        self.view.robot_move_right_signal.connect(self.on_robot_move_right)
        self.view.robot_rotate_right_signal.connect(self.on_robot_rotate_right)
        self.view.robot_rotate_left_signal.connect(self.on_robot_rotate_left)
        
    def initialize_ros(self):
        """ROS2 초기화"""
        try:
            if self.action_manager:
                self.cleanup()
            
            # 새로운 연결 설정
            self.action_manager = ROS2ActionManager('location_add_node')
            # 위치 업데이트 구독 제거 - 더 이상 로봇 실시간 위치를 받아오지 않음
            
            self.ros2_initialized = self.action_manager.start()
            if self.ros2_initialized:
                print("ROS2 초기화 성공")
            return self.ros2_initialized
        except Exception as e:
            print(f"ROS2 초기화 중 오류 발생: {str(e)}")
            self.cleanup()
            return False
        
    def on_robot_position_updated(self, x, y, yaw):
        """로봇 위치 업데이트 시 호출되는 콜백 - 더 이상 사용하지 않음"""
        # 로봇 실시간 위치 업데이트 요청이 제거되었으므로 이 함수는 더 이상 호출되지 않습니다.
        pass
        
    def on_position_clicked(self, x, y):
        """맵에서 위치 클릭 시 호출"""
        print(f"맵 클릭 위치 - X: {x:.3f}, Y: {y:.3f}")
        # 맵에서 클릭한 위치를 저장하고 콘솔에 출력
        print(f"[LocationAddController] 맵 클릭 위치 저장 확인: x={x}, y={y}")
        # 모델 업데이트는 유지 (필요시 다른 작업에 사용될 수 있음)
        self.model.update_position(x, y, 0.0)  # yaw는 0으로 설정
        
    def show_add(self):
        """목적지 추가 화면 표시"""
        try:
            # 모델 초기화
            self.model.load_empty_location()
            
            # View 초기화
            self.view.reset_view()
            self.view.set_mode_add()
            
            # ROS2 초기화
            if not self.initialize_ros():
                MessageBox.show_error(self.view, "ROS2 초기화에 실패했습니다.")
                return
            
            # 위치 값 0으로 설정 (디버그 텍스트 업데이트 용)
            self.view.clicked_x = 0.0
            self.view.clicked_y = 0.0
            self.view.robot_yaw = 0.0
            self.view.update_debug_text(0.0, 0.0, 0.0)
            
            # 이전 화면 설정
            self.main_controller.set_previous_view(self.stack.currentWidget())
            
            # 화면 전환
            self.stack.setCurrentWidget(self.view)

            QTimer.singleShot(100, lambda: self.view.setup_map() if not hasattr(self.view, 'map_manager') or self.view.map_manager is None else None)
            
        except Exception as e:
            print(f"화면 표시 중 오류 발생: {str(e)}")
            MessageBox.show_error(self.view, "화면 표시 중 오류가 발생했습니다.")
        
    def show_edit(self, location_data):
        """목적지 수정 화면 표시"""
        try:
            print(f"수정할 목적지 데이터: {location_data}")
            
            # 모델에 데이터 로드
            edit_id = self.selection_model.find_location_id(location_data)
            self.model.load_location_for_edit(location_data, edit_id)
            
            # View 초기화 및 데이터 설정
            self.view.reset_view()
            self.view.set_mode_edit(True)
            
            # ROS2 초기화 (맵 로드를 위해)
            self.initialize_ros()
            
            # 맵이 로드된 후 위치 데이터 설정 (약간의 지연)
            QTimer.singleShot(300, lambda: self.view.set_location_data(location_data))
            
            # 이전 화면 설정
            self.main_controller.set_previous_view(self.stack.currentWidget())
            
            # 화면 전환
            self.stack.setCurrentWidget(self.view)

        except Exception as e:
            print(f"목적지 수정 화면 표시 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
        
    def on_save_location(self):
        """위치 저장 처리"""
        # View에서 데이터 가져오기
        location_data = self.view.get_location_data()
        print(f"LocationAddController - 받은 데이터: {location_data}")
        
        # 이름 검증
        if not location_data['name']:
            ToastManager.instance().show_toast("이름을 입력해주세요.", self.view)
            return
            
        # 맵에서 클릭한 위치 정보만 사용 (map_data_debug_TextEdit에 표시된 좌표)
        if location_data.get('clicked_x') is not None and location_data.get('clicked_y') is not None:
            # 맵 클릭 위치를 우선적으로 사용 (ROS 실제 좌표계)
            x = location_data['clicked_x']
            y = location_data['clicked_y']
            theta = location_data.get('clicked_theta', 0.0)
            
            # 좌표 정보 로그 출력
            print(f"[LocationAddController] 맵 클릭 위치 사용 (실제 좌표계): x={x:.3f}, y={y:.3f}, yaw={theta:.3f}")
            
            # 좌표 변환 검증 (맵 원점과 해상도 사용하여 변환 확인)
            if hasattr(self.view, 'map_manager') and self.view.map_manager.map_info:
                # 맵 정보 얻기
                map_info = self.view.map_manager.map_info
                map_resolution = map_info.resolution
                map_origin_x = map_info.origin.position.x
                map_origin_y = map_info.origin.position.y
                
                # 실제 좌표 → 화면 좌표 변환 정보
                scene_x = (x - map_origin_x) / map_resolution
                scene_y = (y - map_origin_y) / map_resolution
                
                print(f"[LocationAddController] 좌표 변환 검증:")
                print(f"  - 맵 해상도: {map_resolution}")
                print(f"  - 맵 원점: ({map_origin_x}, {map_origin_y})")
                print(f"  - 실제 좌표 → 화면 좌표: ({x}, {y}) → ({scene_x:.1f}, {scene_y:.1f})")
        else:
            ToastManager.instance().show_toast("맵에서 위치를 클릭해주세요.", self.view)
            return

        # 요청된 형식으로 데이터 구성
        current_data = {
            'name': location_data['name'],
            'position': {
                'x': x,
                'y': y,
                'yaw': theta
            }
        }
        
        # 링고벨 UUID가 있는 경우 추가
        if location_data.get('ringo_uuid'):
            print(f"LocationAddController - UUID 설정: {location_data['ringo_uuid']}")
            current_data['unique_id'] = location_data['ringo_uuid']
            current_data['label'] = 'Yes'
        else:
            current_data['unique_id'] = ""
            current_data['label'] = 'No'
            
        print(f"LocationAddController - 맵 클릭 위치로 저장할 최종 데이터: {current_data}")
            
        # 모델에 데이터 설정
        self.model._current_location_data = current_data
        
        # 저장 실행
        self.model.save_location()
        
    def on_save_result(self, success, message):
        """저장 결과 처리"""
        if success:
            # 목적지 이름 가져오기
            location_name = self.model._current_location_data.get('name', '목적지')
            
            # Toast 메시지 표시
            from Utils.Toast import ToastManager
            ToastManager.instance().show_toast(f"'{location_name}'이 생성되었습니다.", self.view)
            
            self.selection_model.refresh()  # 선택 모델 새로고침
            # UUID 캐시 새로고침
            RingoBellManager.instance().refresh_uuids()
            self.main_controller.selection_controller.show()
        else:
            from Utils.Toast import ToastManager
            ToastManager.instance().show_toast(f"저장 실패: {message}", self.view)
            
    def on_file_updated(self, locations):
        """파일 업데이트 처리"""
        # 선택 모델 새로고침
        self.selection_model.refresh()
        
    def on_cancel(self):
        """취소 처리"""
        try:
            self.view.cleanup_resources()
        except Exception as e:
            print(f"뷰 리소스 정리 중 오류 발생: {str(e)}")

        # 이전 화면으로 돌아가기
        self.main_controller.show_previous_view()
        
    def cleanup(self):
        """리소스 정리"""
        try:
            if self.action_manager:
                self.action_manager.cleanup()
                self.action_manager = None
            self.ros2_initialized = False
            
            # 금지 구역 발행자 정리
            if hasattr(self, 'keepout_zone_publisher'):
                try:
                    self.keepout_zone_publisher.cleanup()
                except Exception as e:
                    print(f"금지 구역 발행자 정리 중 오류 발생: {str(e)}")
        except Exception as e:
            print(f"ROS2 정리 중 오류 발생: {str(e)}")
            self.action_manager = None
            self.ros2_initialized = False

    # 맵 컨트롤 핸들러 함수
    def on_map_zoom_in(self):
        """맵 확대"""
        if hasattr(self.view, 'map_manager'):
            self.view.map_manager.zoom_in()
        
    def on_map_zoom_out(self):
        """맵 축소"""
        if hasattr(self.view, 'map_manager'):
            self.view.map_manager.zoom_out()
        
    def on_map_move_up(self):
        """맵 위로 이동"""
        if hasattr(self.view, 'map_manager'):
            self.view.map_manager.move_map('up')
        
    def on_map_move_down(self):
        """맵 아래로 이동"""
        if hasattr(self.view, 'map_manager'):
            self.view.map_manager.move_map('down')
        
    def on_map_move_left(self):
        """맵 왼쪽으로 이동"""
        if hasattr(self.view, 'map_manager'):
            self.view.map_manager.move_map('left')
        
    def on_map_move_right(self):
        """맵 오른쪽으로 이동"""
        if hasattr(self.view, 'map_manager'):
            self.view.map_manager.move_map('right')
   
    # 로봇 컨트롤 핸들러 함수
    def on_robot_move_up(self):
        """로봇 위로 이동"""
        if hasattr(self.view, 'map_manager'):
            result = self.view.map_manager.move_robot('up', self.view.ROBOT_MOVE_STEP)
            if result and hasattr(self.view, 'update_position_from_model'):
                self.view.update_position_from_model()
       
    def on_robot_move_down(self):
        """로봇 아래로 이동"""
        if hasattr(self.view, 'map_manager'):
            result = self.view.map_manager.move_robot('down', self.view.ROBOT_MOVE_STEP)
            if result and hasattr(self.view, 'update_position_from_model'):
                self.view.update_position_from_model()
       
    def on_robot_move_left(self):
        """로봇 왼쪽으로 이동"""
        if hasattr(self.view, 'map_manager'):
            # 좌우 반전된 맵에서는 'left' 방향으로 이동명령 그대로 전달
            result = self.view.map_manager.move_robot('left', self.view.ROBOT_MOVE_STEP)
            if result and hasattr(self.view, 'update_position_from_model'):
                self.view.update_position_from_model()
       
    def on_robot_move_right(self):
        """로봇 오른쪽으로 이동"""
        if hasattr(self.view, 'map_manager'):
            # 좌우 반전된 맵에서는 'right' 방향으로 이동명령 그대로 전달
            result = self.view.map_manager.move_robot('right', self.view.ROBOT_MOVE_STEP)
            if result and hasattr(self.view, 'update_position_from_model'):
                self.view.update_position_from_model()

    def on_robot_rotate_right(self):
        """로봇 오른쪽으로 회전"""
        if hasattr(self.view, 'map_manager'):
            print(f"회전 전 Yaw: {self.view.robot_yaw:.5f}")
            result = self.view.map_manager.rotate_robot('right', self.view.ROBOT_ROTATE_STEP)
            if result and hasattr(self.view, 'update_position_from_model'):
                self.view.update_position_from_model()
                print(f"회전 후 Yaw: {self.view.robot_yaw:.5f}")

    def on_robot_rotate_left(self):
        """로봇 왼쪽으로 회전"""
        if hasattr(self.view, 'map_manager'):
            print(f"회전 전 Yaw: {self.view.robot_yaw:.5f}")
            result = self.view.map_manager.rotate_robot('left', self.view.ROBOT_ROTATE_STEP)
            if result and hasattr(self.view, 'update_position_from_model'):
                self.view.update_position_from_model()
                print(f"회전 후 Yaw: {self.view.robot_yaw:.5f}")