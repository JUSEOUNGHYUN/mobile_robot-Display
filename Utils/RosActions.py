import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from PySide6.QtCore import QObject, Signal
from Utils.RosUtils import ROS2Thread
import math
import time

class ROS2ActionManager(QObject):
    # 위치 업데이트 시그널
    position_updated = Signal(float, float, float)  # x, y, yaw

    def __init__(self, node_name='action_manager_node'):
        super().__init__()
        
        # ROS2 노드 초기화
        self.node = None
        self.action_client = None
        self.pose_subscriber = None
        self.goal_pose_subscriber = None  # NavigateToPose 목표 구독자 추가
        self.is_initialized = False
        self.ros2_thread = None
        
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                
            # 노드 생성
            self.node = Node(node_name)
            
            # NavigateToPose 액션 클라이언트 생성
            self.action_client = ActionClient(
                self.node,
                NavigateToPose,
                '/navigate_to_pose'
            )
            
            # 위치 구독
            self.pose_subscriber = self.node.create_subscription(
                PoseWithCovarianceStamped,
                '/mcl_pose',
                self.pose_callback,
                10
            )
            
            # 테스트 프로그램에서 보내는 goal_pose 구독 추가
            self.goal_pose_subscriber = self.node.create_subscription(
                PoseStamped,
                '/goal_pose',
                self.goal_pose_callback,
                10
            )
            
            # ROS2 스레드 초기화 (아직 시작하지 않음)
            self.ros2_thread = ROS2Thread(self.node)
            
            self.is_initialized = True
            print("ROS2 Action Manager 초기화 성공")
        except Exception as e:
            print(f"ROS2 Action Manager 초기화 오류: {e}")
            self.is_initialized = False

    def start(self):
        """ROS2 스레드 시작"""
        if self.is_initialized and self.ros2_thread and not self.ros2_thread.isRunning():
            self.ros2_thread.start()
            print("ROS2 스레드 시작됨")
            return True
        return False
    
    def stop(self):
        """ROS2 스레드 중지"""
        if self.ros2_thread and self.ros2_thread.isRunning():
            self.ros2_thread.stop()
            print("ROS2 스레드 중지됨")
    
    def pose_callback(self, msg):
        """위치 정보 수신 처리"""
        try:
            # 위치 정보 추출
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            # 방향 정보를 Yaw 각도로 변환 (쿼터니언 → 오일러)
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            # UI 업데이트 시그널 발생
            self.position_updated.emit(x, y, yaw)

        except Exception as e:
            print(f"위치 정보 처리 중 오류: {e}")
    
    def goal_pose_callback(self, msg):
        """테스트 프로그램에서 보낸 goal_pose 토픽 수신 처리"""
        try:
            # 위치 정보 추출
            x = msg.pose.position.x
            y = msg.pose.position.y
            
            # 방향 정보를 Yaw 각도로 변환 (쿼터니언 → 오일러)
            q = msg.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            print(f"목표 위치 수신: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
            
            # 동일한 위치 업데이트 시그널 사용 (UI는 출처에 관계없이 위치만 중요)
            self.position_updated.emit(x, y, yaw)
            
            # 확인용 응답 메시지 발행 (옵션)
            self._publish_response(msg)
            
        except Exception as e:
            print(f"목표 위치 처리 중 오류: {e}")
    
    def _publish_response(self, original_msg):
        """수신 확인용 응답 메시지 발행 (옵션)"""
        try:
            # 발행자가 아직 생성되지 않았으면 생성
            if not hasattr(self, 'response_publisher'):
                self.response_publisher = self.node.create_publisher(
                    PoseStamped,
                    '/received_goal_pose',
                    10
                )
            
            # 원본 메시지를 그대로 전달
            self.response_publisher.publish(original_msg)
        except Exception as e:
            print(f"응답 발행 중 오류: {e}")
    
# Action 관련 함수

    def wait_for_action_server(self, timeout_sec=1.0):
        """액션 서버 연결 확인"""
        return self.action_client.wait_for_server(timeout_sec=timeout_sec)
    
    def send_goal(self, x, y, yaw, done_callback, feedback_callback=None):
        """목표 위치로 이동 명령 전송"""
        if not self.is_initialized or not self.action_client:
            print("ROS2 Action Manager가 초기화되지 않았습니다.")
            return False
            
        # 목표 위치 설정
        goal_msg = NavigateToPose.Goal()
        
        # 목표 포즈 설정
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        
        # 위치 설정
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # 방향 설정 (yaw를 쿼터니언으로 변환)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = sy
        goal_msg.pose.pose.orientation.w = cy
        
        # 현재 목표 정보 저장
        self.current_goal = {
            'x': float(x),
            'y': float(y),
            'yaw': float(yaw),
            'timestamp': self.node.get_clock().now().to_msg()
        }
        
        # 목표 전송
        send_goal_future = self.action_client.send_goal_async(
            goal_msg, 
            feedback_callback=feedback_callback if feedback_callback else self._default_feedback_callback
        )
        send_goal_future.add_done_callback(done_callback)
        self.goal_handle_future = send_goal_future  # 목표 핸들 저장 (후에 취소 가능하도록)
        
        return True
    
    def cancel_goal(self):
        """현재 실행 중인 내비게이션 목표 취소"""
        try:
            if not hasattr(self, 'goal_handle_future') or self.goal_handle_future is None:
                print("현재 실행 중인 내비게이션 목표가 없습니다.")
                return False
            
            # 목표 핸들 가져오기
            if not self.goal_handle_future.done():
                print("목표 핸들이 아직 준비되지 않았습니다. 취소 대기 중...")
                # 대기하지 않고 취소 요청 (시간 초과 설정)
                wait_count = 0
                while not self.goal_handle_future.done() and wait_count < 10:
                    time.sleep(0.1)
                    wait_count += 1
                    self.node.get_logger().info(f"목표 핸들 준비 대기 중... {wait_count}/10")
            
            # 취소 요청을 보낼 수 있는 상태인지 확인
            if self.goal_handle_future.done():
                goal_handle = self.goal_handle_future.result()
                if goal_handle is not None and goal_handle.accepted:
                    # 취소 요청 보내기
                    cancel_future = goal_handle.cancel_goal_async()
                    print("내비게이션 목표 취소 요청이 전송되었습니다.")
                    return True
            
            # 직접 정지 명령 시도 (추가 안전장치)
            self._send_stop_command()
            
            print("목표 핸들 취소 시도 완료")
            return True
            
        except Exception as e:
            print(f"목표 취소 중 오류 발생: {str(e)}")
            # 오류 발생 시에도 정지 명령 시도
            self._send_stop_command()
            return False
            
    def _send_stop_command(self):
        """로봇 정지 명령 발행 (추가 안전장치)"""
        try:
            # 직접 정지 명령을 보내는 로직
            # cmd_vel 토픽에 0 속도를 발행하여 로봇을 정지시킴
            if not hasattr(self, 'stop_publisher'):
                from geometry_msgs.msg import Twist
                self.stop_publisher = self.node.create_publisher(
                    Twist,
                    '/cmd_vel',
                    10
                )
            
            # 정지 명령 (모든 속도 성분이 0인 Twist 메시지)
            from geometry_msgs.msg import Twist
            stop_msg = Twist()
            # 모든 속도 성분을 0으로 설정
            stop_msg.linear.x = 0.0
            stop_msg.linear.y = 0.0
            stop_msg.linear.z = 0.0
            stop_msg.angular.x = 0.0
            stop_msg.angular.y = 0.0
            stop_msg.angular.z = 0.0
            
            # 여러 번 발행하여 확실히 정지하도록 함
            for i in range(3):
                self.stop_publisher.publish(stop_msg)
                time.sleep(0.1)
                
            print("로봇 정지 명령이 전송되었습니다.")
            
        except Exception as e:
            print(f"로봇 정지 명령 전송 중 오류: {str(e)}")
    
    def _default_feedback_callback(self, feedback_msg):
        """기본 피드백 처리 콜백"""
        feedback = feedback_msg.feedback
        print(f"현재 로봇 위치: {feedback.current_pose.pose.position.x:.2f}, {feedback.current_pose.pose.position.y:.2f}")
    
    def cleanup(self):
        """리소스 정리"""
        print("[ROS2ActionManager] 리소스 정리 시작...")
        
        try:
            # 먼저 스레드 중지
            self.stop()
            
            # 구독자 정리
            try:
                if self.pose_subscriber:
                    self.node.destroy_subscription(self.pose_subscriber)
                    self.pose_subscriber = None
                    
                if self.goal_pose_subscriber:
                    self.node.destroy_subscription(self.goal_pose_subscriber)
                    self.goal_pose_subscriber = None
                    
                if hasattr(self, 'response_publisher') and self.response_publisher:
                    self.node.destroy_publisher(self.response_publisher)
                    self.response_publisher = None
            except Exception as e:
                print(f"[ROS2ActionManager] 구독자/발행자 정리 중 오류: {e}")
            
            # 액션 클라이언트 정리 (수정된 부분)
            try:
                if self.action_client:
                    # 직접 _destroy 메서드 호출하지 않고 참조만 제거
                    # self.action_client._destroy() - 이 부분이 문제를 일으키므로 제거
                    print("[ROS2ActionManager] 액션 클라이언트 참조 제거")
                    self.action_client = None
            except Exception as e:
                print(f"[ROS2ActionManager] 액션 클라이언트 정리 중 오류: {e}")
            
            # 노드 정리
            try:
                if self.node:
                    self.node.destroy_node()
                    self.node = None
            except Exception as e:
                print(f"[ROS2ActionManager] 노드 정리 중 오류: {e}")
            
            # 초기화 상태 재설정
            self.is_initialized = False
            
            print("[ROS2ActionManager] 리소스 정리 완료")
        except Exception as e:
            print(f"[ROS2ActionManager] 리소스 정리 중 오류: {e}")