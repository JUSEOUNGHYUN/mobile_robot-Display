from PySide6.QtCore import QObject, QTimer
from views.MovingView import MovingView
from views.PauseView import PauseView
import math
from Utils.RosActions import ROS2ActionManager
import time
from datetime import datetime
from models.RingoBellManager import RingoBellManager

class MovingController(QObject):
    def __init__(self, stack, main_controller):
        super().__init__()
        self.stack = stack
        self.main_controller = main_controller
        
        # View 초기화
        self.moving_view = MovingView()
        self.pause_view = PauseView()

        # 스택에 뷰 추가
        self.stack.addWidget(self.moving_view)
        self.stack.addWidget(self.pause_view)

        # ROS2 초기화
        self.action_manager = None
        self.ros2_initialized = False

        # 시간 기록 변수
        self.departure_time = None

        # 시그널 연결
        self._connect_signals()

        # 현재 목적지 정보
        self.current_destination = None
        
        # 일시 정지 상태 추적
        self.is_paused = False
        
        # 목적지 변경 상태 추적
        self.destination_changed = False
        
        # 링고벨 매니저에 자신을 등록
        self.ringo_bell_manager = RingoBellManager.instance()

    # ===== 향후 개발 로봇 이동 진행 상황 모니터링 =====
    def _initialize_ros(self):
        """ROS2 초기화"""
        try:
            # ROS2ActionManager 초기화
            self.action_manager = ROS2ActionManager('moving_controller_node')
            
            # 연결 시도 (최대 3번, 1초 간격)
            max_retries = 3
            for attempt in range(max_retries):
                self.ros2_initialized = self.action_manager.start()
                if self.ros2_initialized:
                    print(f"ROS2 초기화 성공 (시도 {attempt + 1}/{max_retries})")
                    if self._wait_for_action_server(timeout_sec=5.0):
                        return
                print(f"ROS2 초기화 실패 (시도 {attempt + 1}/{max_retries})")
                time.sleep(1)
            
            print("ROS2 초기화 최종 실패")
            self.ros2_initialized = False
            
        except Exception as e:
            print(f"ROS2 초기화 오류: {e}")
            self.ros2_initialized = False

    def _wait_for_action_server(self, timeout_sec=5.0):
        """액션 서버 연결 대기"""
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self.action_manager.wait_for_action_server(timeout_sec=1.0):
                print("액션 서버에 연결됨")
                return True
            print("액션 서버에 연결 중...")
        print("액션 서버 연결 타임아웃")
        return False

    def _connect_signals(self):
        # MovingView 시그널 연결
        self.moving_view.pause_signal.connect(self.show_pause_view)

        # PauseView 시그널 연결
        self.pause_view.resume_signal.connect(self.resume_moving)
        self.pause_view.cancel_signal.connect(self.cancel_moving)
        self.pause_view.setting_signal.connect(self.main_controller.settings_controller.show_settings_view)

    def start_moving(self, location_data):
        """이동 중 화면으로 전환"""
        # 위치 고정 상태 확인 (모든 이동 요청에 대해)
        if hasattr(self.main_controller, 'arrive_controller') and self.main_controller.arrive_controller.is_fixed():
            # 호출벨 직접 호출 등으로 이 메서드가 직접 호출된 경우도 위치 고정 상태 확인
            return

        # ROS2 초기화
        self._initialize_ros()
        
        if not self.ros2_initialized:
            print("ROS2가 초기화되지 않았습니다. 이동을 시작할 수 없습니다.")
            return

        # 출발 시간 기록
        self.departure_time = datetime.now()
        print(f"이동 시작 시간: {self.departure_time}")

        print(f"이동 시작: {location_data}")
        self.current_destination = location_data
        self.moving_view.update_destination(location_data)
        
        # 도착지 화면에서 직접 이동 명령이 내려진 경우를 처리
        if self.stack.currentWidget() == self.main_controller.arrive_controller.arrive_view:
            print("도착지 화면에서 새로운 이동 명령 감지됨. 상태 초기화...")
            # 도착 컨트롤러 상태 초기화
            self.main_controller.arrive_controller.cleanup()
        
        self.send_navigate_goal(location_data)
        self.stack.setCurrentWidget(self.moving_view)

    def send_navigate_goal(self, location_data):
        """NavigateToPose 액션 목표 전송"""
        if not self.action_manager or not location_data:
            print("액션 매니저가 초기화되지 않았거나 위치 정보가 없습니다.")
            return

        try:
            # 위치 정보 추출
            position = location_data.get('position', {})
            x = float(position.get('x', 0.0))
            y = float(position.get('y', 0.0))
            yaw = float(position.get('yaw', 0.0))

            print(f"목적지로 이동 명령 전송: x={x}, y={y}, yaw={yaw}")

            # 액션 매니저를 통해 목표 전송
            self.action_manager.send_goal(
                x, y, yaw,
                self._send_goal_done_callback,
                self._feedback_callback
            )

        except Exception as e:
            print(f"목표 전송 중 오류 발생: {e}")
    
    def _send_goal_done_callback(self, future):
        """액션 목표 완료 콜백"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                print("목표가 거부되었습니다")
                return
            
            print("목표가 수락되었습니다")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._get_result_callback)
        except Exception as e:
            print(f"목표 응답 처리 중 오류 발생: {e}")
    
    def _get_result_callback(self, future):
        """액션 결과 콜백"""
        try:
            # 현재 스택의 위젯 확인
            current_widget = self.stack.currentWidget()
            is_moving_screen = current_widget == self.moving_view
            
            result = future.result().result
            print("NavigateToPose 액션 완료!")
            print(f"[MovingController] 목적지 변경 플래그: {self.destination_changed}, 일시정지 플래그: {self.is_paused}")
            
            # 일시 정지나 목적지 변경으로 인한 취소인 경우 도착 화면으로 전환하지 않음
            if self.is_paused or self.destination_changed:
                print("[MovingController] 이동 취소: 일시 정지 또는 목적지 변경 상태이므로 도착 화면으로 전환하지 않습니다.")
                self.destination_changed = False  # 플래그 초기화
                return
            
            # 도착 시간 기록
            arrival_time = datetime.now()
            travel_time = arrival_time - self.departure_time
            print(f"이동 완료 시간: {arrival_time}, 소요 시간: {travel_time}")
            
            # 목적지 정보가 있고, 이동 완료 상태라면 도착 화면으로 전환
            if self.current_destination:
                print("[MovingController] 이동 완료: 도착 화면으로 전환합니다.")
                self.main_controller.arrive_controller.set_target_position(
                    self.current_destination.get('position', {}).get('x', 0.0),
                    self.current_destination.get('position', {}).get('y', 0.0),
                    self.current_destination.get('position', {}).get('yaw', 0.0)
                )
                self.main_controller.arrive_controller.target_position['name'] = self.current_destination.get('name', '목적지')
                
                # 출발 시간과 소요 시간 정보 전달
                self.main_controller.arrive_controller.set_travel_time_info(
                    self.departure_time, 
                    arrival_time, 
                    travel_time
                )
                
                # 도착 화면으로 전환
                self.main_controller.arrive_controller.show_arrive_view()
            else:
                print("[MovingController] 이동 완료: 목적지 정보가 없어 도착 화면으로 전환할 수 없습니다.")
            
            # ROS2 정리
            self.cleanup()
            
        except Exception as e:
            print(f"결과 처리 중 오류 발생: {e}")
    
    def _feedback_callback(self, feedback_msg):
        """액션 피드백 콜백"""
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        
        # 피드백 정보 출력
        x = current_pose.position.x
        y = current_pose.position.y
        
        # 쿼터니언에서 yaw 추출
        q = current_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        print(f"로봇 현재 위치: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

    # 이동 중 모드와 일시 정지 화면 상호 간 처리
    def show_pause_view(self, destination):
        """일시정지 화면 표시"""
        # 일시 정지 상태로 설정
        self.is_paused = True
        
        # 현재 이동 목표 취소 (로봇 멈춤)
        if self.action_manager and self.ros2_initialized:
            try:
                print("[MovingController] 일시 정지: 로봇 이동 목표 취소 중...")
                self.action_manager.cancel_goal()
                print("[MovingController] 로봇 이동 목표 취소 완료")
            except Exception as e:
                print(f"[MovingController] 로봇 이동 취소 중 오류: {str(e)}")
        
        self.pause_view.update_destination(destination)
        self.stack.setCurrentWidget(self.pause_view)
    
    def resume_moving(self, destination):
        """이동 계속"""
        # 일시 정지 상태 해제
        self.is_paused = False
        
        print(f"[MovingController] 이동 재개: {destination.get('name', '알 수 없는 목적지')}")
        
        # destination이 비어있거나 올바르지 않은 경우 현재 저장된 목적지 정보 사용
        if not destination or 'position' not in destination:
            print("[MovingController] 전달받은 목적지 정보가 유효하지 않습니다. 저장된 목적지 정보를 사용합니다.")
            if self.current_destination and 'position' in self.current_destination:
                destination = self.current_destination
            else:
                print("[MovingController] 유효한 목적지 정보가 없습니다. 이동을 재개할 수 없습니다.")
                self.stack.setCurrentWidget(self.moving_view)
                return
        
        # 이동을 다시 시작
        if self.action_manager and self.ros2_initialized:
            try:
                # ROS2 연결 상태 확인 (필요시 재연결)
                if not self._ensure_ros2_connection():
                    print("[MovingController] ROS2 연결을 확인할 수 없습니다. 재초기화를 시도합니다.")
                    self._initialize_ros()
                
                # 저장된 목적지 정보 사용하여 이동 재개
                position = destination.get('position', {})
                x = float(position.get('x', 0.0))
                y = float(position.get('y', 0.0))
                yaw = float(position.get('yaw', 0.0))
                
                print(f"[MovingController] 목적지로 이동 재개: x={x}, y={y}, yaw={yaw}")
                
                # 액션 매니저를 통해 목표 다시 전송
                success = self.action_manager.send_goal(
                    x, y, yaw,
                    self._send_goal_done_callback,
                    self._feedback_callback
                )
                
                if success:
                    print("[MovingController] 목적지로 이동 명령이 성공적으로 전송되었습니다.")
                else:
                    print("[MovingController] 목적지로 이동 명령 전송에 실패했습니다.")
                
            except Exception as e:
                print(f"[MovingController] 이동 재개 중 오류 발생: {e}")
        else:
            print("[MovingController] ROS2가 초기화되지 않았습니다. 이동을 재개할 수 없습니다.")
        
        # 이동 중 화면으로 돌아가기
        self.stack.setCurrentWidget(self.moving_view)
        
    def _ensure_ros2_connection(self):
        """ROS2 연결 상태 확인"""
        if not self.action_manager or not self.ros2_initialized:
            return False
            
        try:
            # 액션 서버에 연결되어 있는지 확인
            if self.action_manager.wait_for_action_server(timeout_sec=1.0):
                return True
                
            return False
        except Exception as e:
            print(f"[MovingController] ROS2 연결 확인 중 오류: {e}")
            return False
    
    def cancel_moving(self):
        """이동 취소하고 목적지 선택 화면으로 돌아가기"""
        # 일시 정지 상태 해제
        self.is_paused = False
        
        # ROS2 정리
        self.cleanup()
        
        # 목적지 선택 화면으로 돌아가기
        self.main_controller.selection_controller.show()
        
        # 현재 목적지 정보 초기화
        self.current_destination = None
        
    def cleanup(self):
        """리소스 정리"""
        print("[MovingController] 리소스 정리 시작...")
        
        try:
            # ROS2 액션 매니저 정리
            if self.action_manager:
                print("[MovingController] ROS2 액션 매니저 정리 중...")
                self.action_manager.cleanup()
                self.ros2_initialized = False
                self.action_manager = None
            
            # 기타 리소스 정리
            self.current_destination = None
            self.is_paused = False
            self.destination_changed = False
            
            print("[MovingController] 리소스 정리 완료")
        except Exception as e:
            print(f"[MovingController] 리소스 정리 중 오류: {e}")

    def change_destination(self, new_location):
        """새로운 호출벨 목적지로 이동 변경"""
        if not new_location:
            print("[MovingController] 새 목적지 정보가 없습니다.")
            return False
            
        print(f"[MovingController] 목적지 변경: {new_location.get('name', '알 수 없음')}")
        
        # 링고벨 매니저의 무시 설정 확인 (디버깅 용도)
        ringo_manager = self.ringo_bell_manager
        if hasattr(ringo_manager, 'ignore_other_bell'):
            print(f"[MovingController] RingoBellManager ignore_other_bell 설정: {ringo_manager.ignore_other_bell}")
        else:
            print("[MovingController] RingoBellManager에 ignore_other_bell 속성이 없습니다.")
        
        try:
            # 목적지 변경 상태로 설정
            self.destination_changed = True
            print(f"[MovingController] 목적지 변경 플래그 설정: {self.destination_changed}")
            
            # 현재 이동 중이거나 일시 정지 상태인 경우 취소
            if self.action_manager and self.ros2_initialized:
                print("[MovingController] 기존 이동 취소 중...")
                self.action_manager.cancel_goal()
                
            # 새로운 목적지 정보 설정
            self.current_destination = new_location
            
            # 일시 정지 상태였다면 해제
            self.is_paused = False
            
            # 이동 화면 업데이트
            self.moving_view.update_destination(new_location)
            
            # 이동 화면으로 전환
            self.stack.setCurrentWidget(self.moving_view)
            
            # 새 목적지로 이동 시작
            self.send_navigate_goal(new_location)
            
            return True
        except Exception as e:
            print(f"[MovingController] 목적지 변경 중 오류: {str(e)}")
            return False