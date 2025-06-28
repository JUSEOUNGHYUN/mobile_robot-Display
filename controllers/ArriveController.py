"""
ArriveController 모듈은 도착 화면을 관리하는 컨트롤러를 제공합니다.
"""

import os
import math
import json
from datetime import datetime, timedelta
from PySide6.QtCore import QTimer, QObject
from views.ArriveView import ArriveView
from Utils.RosActions import ROS2ActionManager
from Utils.MessageBox import MessageBox

class ArriveController(QObject):
    """도착 화면을 관리하는 컨트롤러 클래스"""
    def __init__(self, stack, main_controller):
        super().__init__()
        self.stack = stack
        self.main_controller = main_controller
        
        # 도착 위치 정보
        self.target_position = {
            'x': 0.0,
            'y': 0.0,
            'yaw': 0.0,
            'name': '목적지'
        }
        
        # 시간 기록 변수
        self.departure_time = None
        self.arrival_time = None
        self.travel_time = None
        
        # View 초기화
        self.arrive_view = ArriveView()
        
        # 스택에 뷰 추가
        self.stack.addWidget(self.arrive_view)
        
        # ROS2 액션 매니저 초기화
        self.action_manager = ROS2ActionManager('arrive_node')
        self.action_manager.position_updated.connect(self._on_position_updated)
        self.ros2_initialized = False
        
        # 목적지 정보
        self.target_position = {
            'x': 0.0,
            'y': 0.0,
            'yaw': 0.0
        }
        self.target_tolerance = 0.1  # 도착 판단 허용 오차 (미터)
        
        # 위치 고정 상태
        self.position_fixed = False
        
        # 시그널 연결
        self._connect_signals()

    def _connect_signals(self):
        """시그널 연결"""
        # 도착 화면 시그널 연결
        self.arrive_view.return_signal.connect(self.go_home)
        self.arrive_view.setting_signal.connect(
            self.main_controller.settings_controller.show_settings_view)
        self.arrive_view.location_fixed_signal.connect(self.toggle_fixed_position)
        self.arrive_view.locate_select_signal.connect(self.go_to_selection)
        self.arrive_view.waiting_location_signal.connect(self.go_to_waiting_location)

    def initialize_ros(self):
        """ROS2 초기화"""
        if not self.ros2_initialized:
            try:
                # 액션 매니저가 이미 초기화되어 있는지 확인
                if self.action_manager:
                    print("[ArriveController] ROS2 액션 매니저 이미 초기화됨")
                else:
                    self.action_manager = ROS2ActionManager('arrive_node')
                    self.action_manager.position_updated.connect(
                        self._on_position_updated)
                    print("[ArriveController] ROS2 액션 매니저 초기화 완료")
                
                self.ros2_initialized = True
            except Exception as e:
                print(f"[ArriveController] ROS2 초기화 중 오류 발생: {str(e)}")
                import traceback
                traceback.print_exc()

    def set_target_position(self, x, y, yaw):
        """도착 지점 설정"""
        self.target_position['x'] = x
        self.target_position['y'] = y
        self.target_position['yaw'] = yaw

    def _on_position_updated(self, x, y, yaw):
        """로봇 위치 업데이트 시 호출되는 콜백"""
        try:
            if self.target_position is None:
                print("목적지 위치가 설정되지 않았습니다.")
                return
                
            # 현재 위치와 목적지 위치의 차이 계산
            dx = self.target_position['x'] - x
            dy = self.target_position['y'] - y
            distance = math.sqrt(dx * dx + dy * dy)
            
            print(f"현재 위치: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
            print(f"목적지 위치: x={self.target_position['x']}, y={self.target_position['y']}")
            print(f"거리 차이: {distance:.2f}m")
            
            # 거리가 0.1m 이내이면 도착으로 판단
            if distance < 0.1:
                print("목적지에 도착했습니다.")
                self.arrive_view.set_arrived(True)
        except Exception as e:
            print(f"위치 업데이트 처리 중 오류 발생: {str(e)}")

    def set_travel_time_info(self, departure_time, arrival_time, travel_time):
        """이동 시간 정보 설정"""
        self.departure_time = departure_time
        self.arrival_time = arrival_time
        self.travel_time = travel_time
        print(f"[ArriveController] 이동 시간 정보 설정 - 출발: {departure_time}, 도착: {arrival_time}, 소요: {travel_time}")

    def show_arrive_view(self):
        """도착 화면 표시"""
        # 화면 초기화 및 현재 위치 정보 업데이트
        self.arrive_view.set_destination_name(self.target_position.get('name', '목적지'))
        self.arrive_view.set_fixed_state(self.position_fixed)
        
        # 이동 시간 정보 설정
        if self.departure_time and self.arrival_time and self.travel_time:
            # 이동 시간 정보를 화면에 표시
            self.arrive_view.set_travel_time_info(
                self.departure_time.strftime("%Y-%m-%d %H:%M:%S"),
                self.arrival_time.strftime("%Y-%m-%d %H:%M:%S"),
                str(self.travel_time).split('.')[0]  # 밀리초 제거
            )
        
        # fixed 상태 초기화 (도착 시에는 항상 고정 해제)
        self.position_fixed = False
        self.arrive_view.set_fixed_state(False)
        
        # 저장된 모든 위치 정보 불러오기
        self._load_destination_list()
        
        # 현재 화면을 도착 화면으로 전환
        self.stack.setCurrentWidget(self.arrive_view)

    def _load_destination_list(self):
        """저장된 목적지 목록 로드"""
        try:
            # 파일이 존재하는 경우만 로드
            if not os.path.exists('data/locations.txt'):
                print("저장된 목적지가 없습니다.")
                return
            
            # 목적지 파일 불러오기
            with open('data/locations.txt', 'r', encoding='utf-8') as f:
                locations = json.load(f)
            
            # 현재 위치와 동일한 목적지는 제외하고 목록에 추가
            available_destinations = []
            current_x = self.target_position.get('x')
            current_y = self.target_position.get('y')
            
            for location in locations:
                # 이름이 없는 위치는 제외
                if not location.get('name'):
                    continue
                
                # 현재 위치와 같은 위치는 제외 (간단한 비교)
                loc_x = location.get('position', {}).get('x')
                loc_y = location.get('position', {}).get('y')
                
                # 위치가 동일하거나 현재 목적지 이름과 동일한 경우 제외
                if (loc_x == current_x and loc_y == current_y) or \
                   location.get('name') == self.target_position.get('name'):
                    continue
                
                available_destinations.append(location)
            
            # 도착 화면에 목적지 목록 설정
            self.arrive_view.set_destinations(available_destinations)
            
        except Exception as e:
            print(f"목적지 로드 중 오류: {e}")
    
    def go_home(self):
        """홈 위치로 이동"""
        # MainController를 통해 홈 화면으로 이동
        if hasattr(self.main_controller, 'home_controller'):
            self.main_controller.home_controller.show()
    
    def go_to_selection(self):
        """선택 화면으로 이동"""
        # MainController를 통해 선택 화면으로 이동
        if hasattr(self.main_controller, 'selection_controller'):
            self.main_controller.selection_controller.show()
    
    def toggle_fixed_position(self, fixed):
        """위치 고정 상태 토글"""
        # 화면에서 fixed 버튼 상태 변경 시 호출
        self.position_fixed = fixed
        print(f"위치 고정 상태 변경: {fixed}")
        
        # UI 업데이트
        self.arrive_view.set_fixed_state(fixed)
    
    def is_fixed(self):
        """현재 위치 고정 상태 반환"""
        return self.position_fixed
    
    def get_current_position(self):
        """현재 도착한 위치 정보 반환"""
        return {
            'position': {
                'x': self.target_position.get('x', 0.0),
                'y': self.target_position.get('y', 0.0),
                'yaw': self.target_position.get('yaw', 0.0)
            },
            'name': self.target_position.get('name', '목적지'),
            'unique_id': self.get_location_unique_id(self.target_position.get('name', '목적지'))
        }
        
    def get_location_unique_id(self, location_name):
        """위치 이름으로 저장된 unique_id 찾기"""
        try:
            if os.path.exists('data/locations.txt'):
                with open('data/locations.txt', 'r', encoding='utf-8') as f:
                    locations = json.load(f)
                
                for location in locations:
                    if location.get('name') == location_name and 'unique_id' in location:
                        return location['unique_id']
            return None
        except Exception as e:
            print(f"위치 ID 찾기 오류: {e}")
            return None
    
    def cleanup(self):
        """리소스 정리"""
        try:
            print("[ArriveController] 리소스 정리 시작...")
            
            # ROS2 액션 매니저 정리
            if self.action_manager:
                self.action_manager.cleanup()
            
            # 화면 상태 초기화
            self.arrive_view.set_arrived(False)
            self.target_position = {
                'x': 0.0,
                'y': 0.0,
                'yaw': 0.0,
                'name': ''
            }
            
            # 위치 고정 상태 초기화
            self.position_fixed = False
            
            # 시간 정보 초기화
            self.departure_time = None
            self.arrival_time = None
            self.travel_time = None
            
            print("[ArriveController] 리소스 정리 완료")
        except Exception as e:
            print(f"[ArriveController] 리소스 정리 중 오류 발생: {str(e)}")

    def show(self, location_data):
        """도착 화면 표시"""
        try:
            # 목적지 위치 정보 설정
            if location_data:
                self.target_position = {
                    'x': float(location_data.get('x', 0.0)),
                    'y': float(location_data.get('y', 0.0)),
                    'yaw': float(location_data.get('yaw', 0.0))
                }
            else:
                self.target_position = None
                print("목적지 위치 정보가 없습니다.")
                return
            
            # 화면 초기화
            self.arrive_view.reset_view()
            
            # ROS2 초기화
            self.initialize_ros()
            
            # 화면 전환
            self.stack.setCurrentWidget(self.arrive_view)
            
        except Exception as e:
            print(f"도착 화면 표시 중 오류 발생: {str(e)}")
            MessageBox.show_error(f"도착 화면 표시 중 오류 발생: {str(e)}")            

    def go_to_waiting_location(self):
        """대기 장소 화면으로 이동"""
        # MainController를 통해 대기 장소 화면으로 이동
        if hasattr(self.main_controller, 'waiting_location_controller'):
            self.main_controller.waiting_location_controller.show()
        else:
            # 대기 장소 컨트롤러가 없는 경우 선택 화면으로 이동
            print("대기 장소 컨트롤러가 없습니다. 선택 화면으로 이동합니다.")
            self.go_to_selection()            
