import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from PySide6.QtCore import QObject, Signal, Slot
import time
from std_msgs.msg import Bool  # Bool 메시지 타입 추가
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int32  # Int32 메시지 타입 추가
import signal
import sys
from geometry_msgs.msg import PolygonStamped, Point32  # PolygonStamped와 Point32 메시지 타입 추가
from std_msgs.msg import Header  # Header 메시지 타입 추가

# 전역 변수로 ROS2 컨텍스트 상태 관리
_ros_context = None

def init_ros_context():
    """ROS2 컨텍스트 초기화"""
    global _ros_context
    if not rclpy.ok():
        rclpy.init()
        _ros_context = True

def shutdown_ros_context():
    """ROS2 컨텍스트 종료"""
    global _ros_context
    if rclpy.ok():
        rclpy.shutdown()
        _ros_context = False

def signal_handler(signum, frame):
    """시그널 핸들러"""
    print("\n=== 프로그램 종료 중... ===")
    shutdown_ros_context()
    sys.exit(0)

# SIGINT (Ctrl+C) 시그널 핸들러 등록
signal.signal(signal.SIGINT, signal_handler)

class MapSubscriber(QObject):
    """맵 데이터 구독자 클래스"""
    map_data_received = Signal(dict)  # 맵 데이터 수신 시그널
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.subscription = None
        self._is_view_active = False
        self._has_received_map = False  # 맵 데이터 수신 여부를 추적하는 플래그
        self._reinitialize_count = 0
        self._max_reinitialize_attempts = 3
        
        # ROS2 초기화
        init_ros_context()
        
        # 노드 초기화
        self.initialize_node()
    
    def initialize_node(self):
        """노드 초기화"""
        try:
            # 노드 생성
            if self.node is None:
                self.node = Node("map_subscriber")
            
            # 토픽 발행자 수 확인
            publishers = self.node.count_publishers('/map')
            if publishers == 0:
                print("경고: /map 토픽에 발행자가 없습니다. 사용 가능한 토픽을 확인하세요.")
                return
            
            # QoS 프로파일 설정
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
            
            # 구독 생성
            self.subscription = self.node.create_subscription(
                OccupancyGrid,
                '/map',
                self.map_callback,
                qos_profile
            )
            print("맵 데이터 구독이 초기화되었습니다.")
            
        except Exception as e:
            print(f"맵 데이터 구독 초기화 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def map_callback(self, msg):
        """맵 데이터 콜백"""
        try:
            if not self._is_view_active or self._has_received_map:
                return
                
            print("\n=== 맵 데이터 수신 ===")
            print(f"크기: {msg.info.width} x {msg.info.height}")
            print(f"해상도: {msg.info.resolution} meters/cell")
            print(f"원점: ({msg.info.origin.position.x}, {msg.info.origin.position.y})")
            print(f"데이터 길이: {len(msg.data)}")
            print("=====================\n")
            
            # 맵 데이터를 딕셔너리로 변환
            map_data = {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin': {
                    'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y,
                    'z': msg.info.origin.position.z
                },
                'data': msg.data
            }
            
            # 시그널 발생
            self.map_data_received.emit(map_data)
            
            # 맵 데이터 수신 완료 표시
            self._has_received_map = True
            print("맵 데이터를 성공적으로 수신했습니다. 더 이상 수신하지 않습니다.")
            
            # 구독 비활성화
            self.deactivate_subscription()
            
        except Exception as e:
            print(f"맵 데이터 처리 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def set_view_active(self, active: bool):
        """뷰 활성화 상태 설정"""
        self._is_view_active = active
        if active and not self._has_received_map:
            self.activate_subscription()
        elif not active:
            self.deactivate_subscription()
    
    def activate_subscription(self):
        """구독 활성화"""
        if not self._has_received_map and self.subscription is None:
            self.initialize_node()
    
    def deactivate_subscription(self):
        """구독 비활성화"""
        if self.subscription is not None:
            try:
                self.node.destroy_subscription(self.subscription)
                self.subscription = None
                print("맵 데이터 구독이 비활성화되었습니다.")
            except Exception as e:
                print(f"맵 데이터 구독 비활성화 중 오류 발생: {str(e)}")
    
    def cleanup(self):
        """리소스 정리"""
        try:
            if self.subscription is not None:
                self.node.destroy_subscription(self.subscription)
                self.subscription = None
            
            if self.node is not None:
                self.node.destroy_node()
                self.node = None
                
            print("맵 데이터 구독 리소스가 정리되었습니다.")
        except Exception as e:
            print(f"맵 데이터 구독 정리 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def spin_once(self):
        """노드 스핀"""
        if not self._is_view_active or self._has_received_map:
            return
            
        try:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            print(f"맵 데이터 구독 스핀 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
            
            # 재초기화 시도
            self._reinitialize_count += 1
            if self._reinitialize_count <= self._max_reinitialize_attempts:
                print(f"맵 데이터 구독 재초기화 시도 {self._reinitialize_count}/{self._max_reinitialize_attempts}")
                self.cleanup()
                self.initialize_node()
            else:
                print("맵 데이터 구독 재초기화 최대 시도 횟수 초과")

class BatterySubscriber(QObject):
    battery_updated = Signal(int)  # 배터리 데이터 업데이트 시그널
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.subscription = None
        self.is_active = False
        self.last_debug_time = time.time()
        self.debug_counter = 0
        self._reinitialize_count = 0  # 재초기화 횟수 추적
        
        # ROS2 초기화
        init_ros_context()
        
        # 노드 초기화
        self.initialize_node()
        
    def initialize_node(self):
        """ROS2 노드 초기화"""
        try:
            print("\n=== BatterySubscriber 노드 초기화 시작 ===")
            self.node = Node("battery_subscriber")
            self.is_active = True
            
            # QoS 프로파일 설정
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
            print("[BatterySubscriber] QoS 프로파일 설정 완료")
            
            # 구독 생성
            self.subscription = self.node.create_subscription(
                Int32,
                '/mobile_robot/battery_percent',
                self.battery_callback,
                qos_profile
            )
            print("[BatterySubscriber] 구독 생성 완료")
            
            # 토픽 발행자 수 확인
            publisher_count = self.node.count_publishers('/mobile_robot/battery_percent')
            print(f"[BatterySubscriber] /mobile_robot/battery_percent 토픽 발행자 수: {publisher_count}")
            
            if publisher_count == 0:
                print("[BatterySubscriber] 경고: 토픽 발행자가 없습니다!")
                print("[BatterySubscriber] 사용 가능한 토픽 목록:")
                topic_list = self.node.get_topic_names_and_types()
                for topic_name, topic_types in topic_list:
                    print(f"  - {topic_name} ({topic_types})")
            
            print("=== BatterySubscriber 노드 초기화 완료 ===\n")
            
        except Exception as e:
            print(f"[BatterySubscriber] 초기화 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
            
    def _reinitialize_node(self):
        """노드 재초기화"""
        try:
            print("[BatterySubscriber] 노드 재초기화 시작...")
            self._reinitialize_count += 1
            
            # 기존 구독자 정리
            if self.subscription:
                self.node.destroy_subscription(self.subscription)
                self.subscription = None
            
            # 노드 재생성
            if self.node:
                self.node.destroy_node()
            
            # 새 노드 생성
            self.node = Node("battery_subscriber")
            self.is_active = True
            
            # QoS 프로파일 설정
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
            
            # 구독자 재생성
            self.subscription = self.node.create_subscription(
                Int32,
                '/mobile_robot/battery_percent',
                self.battery_callback,
                qos_profile
            )
            
            print("[BatterySubscriber] 노드 재초기화 완료")
            print(f"/mobile_robot/battery_percent 토픽 발행자 수: {self.node.count_publishers('/mobile_robot/battery_percent')}")
            
        except Exception as e:
            print(f"[BatterySubscriber] 노드 재초기화 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
            
    def battery_callback(self, msg):
        """배터리 데이터 수신 콜백"""
        try:
            print(f"[BatterySubscriber] 배터리 데이터 수신: {msg.data}%")
            # 디버그 메시지 (5초마다)
            current_time = time.time()
            if current_time - self.last_debug_time >= 5.0:
                self.debug_counter += 1
                print(f"\n=== 배터리 상태 ({self.debug_counter}) ===")
                print(f"배터리 잔량: {msg.data}%")
                self.last_debug_time = current_time
            
            # 시그널 발생
            self.battery_updated.emit(msg.data)
            print(f"[BatterySubscriber] 배터리 시그널 발생: {msg.data}%")
            
        except Exception as e:
            print(f"[BatterySubscriber] 배터리 콜백 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
            
    def spin_once(self):
        """ROS2 노드 스핀"""
        if not rclpy.ok():
            init_ros_context()
            return
            
        if self.node and self.is_active:
            try:
                # 스핀
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except IndexError as e:
                print(f"[BatterySubscriber] wait set 인덱스 오류: {str(e)}")
                # 노드 재초기화 시도
                self._reinitialize_node()
            except Exception as e:
                print(f"노드 스핀 중 오류 발생: {str(e)}")
                import traceback
                traceback.print_exc()
    
    def cleanup(self):
        """리소스 정리"""
        try:
            if self.node:
                print("\n=== 배터리 리소스 정리 시작 ===")
                
                # 구독자 정리
                if self.subscription:
                    self.node.destroy_subscription(self.subscription)
                    self.subscription = None
                    print("배터리 구독자 정리 완료")
                
                # 노드 정리
                self.node.destroy_node()
                self.node = None
                print("노드 정리 완료")
                
                print("=== 배터리 리소스 정리 완료 ===\n")
                
        except Exception as e:
            print(f"리소스 정리 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()

class EmergencyButtonSubscriber(QObject):
    emergency_state_changed = Signal(bool)  # 비상 버튼 상태 변경 시그널
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.subscription = None
        self.is_active = False
        self._reinitialize_count = 0
        
        # ROS2 초기화
        init_ros_context()
        
        # 노드 초기화
        self.initialize_node()
        
    def initialize_node(self):
        """ROS2 노드 초기화"""
        try:
            print("\n=== EmergencyButtonSubscriber 노드 초기화 시작 ===")
            self.node = Node("emergency_button_subscriber")
            self.is_active = True
            
            # QoS 프로파일 설정
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
            
            # 구독 생성
            self.subscription = self.node.create_subscription(
                Bool,
                '/safety/emergency_button',
                self.emergency_callback,
                qos_profile
            )
            print("[EmergencyButtonSubscriber] 구독 생성 완료")
            
            # 토픽 발행자 수 확인
            publisher_count = self.node.count_publishers('/safety/emergency_button')
            print(f"[EmergencyButtonSubscriber] /safety/emergency_button 토픽 발행자 수: {publisher_count}")
            
            if publisher_count == 0:
                print("[EmergencyButtonSubscriber] 경고: 토픽 발행자가 없습니다!")
            
            print("=== EmergencyButtonSubscriber 노드 초기화 완료 ===\n")
            
        except Exception as e:
            print(f"[EmergencyButtonSubscriber] 노드 초기화 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def emergency_callback(self, msg):
        """비상 버튼 상태 콜백"""
        try:
            self.emergency_state_changed.emit(msg.data)
        except Exception as e:
            print(f"[EmergencyButtonSubscriber] 콜백 처리 중 오류 발생: {str(e)}")
    
    def spin_once(self):
        """노드 스핀"""
        if not self.is_active:
            return
            
        try:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            print(f"[EmergencyButtonSubscriber] 스핀 중 오류 발생: {str(e)}")
            self._reinitialize_node()
    
    def _reinitialize_node(self):
        """노드 재초기화"""
        self._reinitialize_count += 1
        if self._reinitialize_count <= 3:  # 최대 3번 재시도
            print(f"[EmergencyButtonSubscriber] 노드 재초기화 시도 {self._reinitialize_count}/3")
            self.cleanup()
            self.initialize_node()
        else:
            print("[EmergencyButtonSubscriber] 노드 재초기화 최대 시도 횟수 초과")
    
    def cleanup(self):
        """리소스 정리"""
        try:
            if self.subscription is not None:
                self.node.destroy_subscription(self.subscription)
                self.subscription = None
            
            if self.node is not None:
                self.node.destroy_node()
                self.node = None
                
            self.is_active = False
            print("[EmergencyButtonSubscriber] 리소스 정리 완료")
        except Exception as e:
            print(f"[EmergencyButtonSubscriber] 리소스 정리 중 오류 발생: {str(e)}")

class KeepoutZonePublisher(QObject):
    """금지 구역 발행자 클래스"""
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.publisher = None
        
        # ROS2 초기화
        init_ros_context()
        
        # 노드 초기화
        self.initialize_node()
    
    def initialize_node(self):
        """노드 초기화"""
        try:
            # 노드 생성
            if self.node is None:
                self.node = Node("keepout_zone_publisher")
            
            # QoS 프로파일 설정
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
            
            # 발행자 생성
            self.publisher = self.node.create_publisher(
                PolygonStamped,
                '/keepout_zone',
                qos_profile
            )
            print("금지 구역 발행자가 초기화되었습니다.")
            
        except Exception as e:
            print(f"금지 구역 발행자 초기화 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def publish_keepout_zone(self, zone_index, points):
        """
        금지 구역 발행
        
        Args:
            zone_index (int): 금지 구역 인덱스 (frame_id로 사용)
            points (list): 금지 구역 꼭짓점 좌표 리스트 [(x1, y1), (x2, y2), ...]
        """
        try:
            if self.publisher is None:
                print("발행자가 초기화되지 않았습니다.")
                return
            
            # PolygonStamped 메시지 생성
            msg = PolygonStamped()
            
            # Header 설정 (금지 구역 인덱스를 frame_id로 사용)
            msg.header = Header()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = f"keepout_zone_{zone_index}"
            
            # 폴리곤 꼭짓점 설정
            for point in points:
                p = Point32()
                p.x = float(point[0])
                p.y = float(point[1])
                p.z = 0.0  # 2D 맵이므로 z는 0으로 설정
                msg.polygon.points.append(p)
            
            # 메시지 발행
            self.publisher.publish(msg)
            print(f"금지 구역 {zone_index} 발행 완료: {len(points)}개 꼭짓점")
            
        except Exception as e:
            print(f"금지 구역 발행 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def spin_once(self):
        """노드 스핀"""
        try:
            if self.node:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            print(f"금지 구역 발행자 스핀 중 오류 발생: {str(e)}")
    
    def cleanup(self):
        """리소스 정리"""
        try:
            if self.publisher is not None:
                self.node.destroy_publisher(self.publisher)
                self.publisher = None
            
            if self.node is not None:
                self.node.destroy_node()
                self.node = None
                
            print("금지 구역 발행자 리소스가 정리되었습니다.")
        except Exception as e:
            print(f"금지 구역 발행자 정리 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()

class DeleteKeepoutZonePublisher(QObject):
    """금지 구역 삭제 발행자 클래스"""
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.publisher = None
        
        # ROS2 초기화
        init_ros_context()
        
        # 노드 초기화
        self.initialize_node()
    
    def initialize_node(self):
        """노드 초기화"""
        try:
            # 노드 생성
            if self.node is None:
                self.node = Node("delete_keepout_zone_publisher")
            
            # QoS 프로파일 설정
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
            
            # 발행자 생성
            self.publisher = self.node.create_publisher(
                Int32,
                '/delete_keepout_zone',
                qos_profile
            )
            print("금지 구역 삭제 발행자가 초기화되었습니다.")
            
        except Exception as e:
            print(f"금지 구역 삭제 발행자 초기화 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def publish_delete_keepout_zone(self, zone_index):
        """
        금지 구역 삭제 발행
        
        Args:
            zone_index (int): 삭제할 금지 구역 인덱스
        """
        try:
            if self.publisher is None:
                print("발행자가 초기화되지 않았습니다.")
                return
            
            # Int32 메시지 생성
            msg = Int32()
            msg.data = zone_index
            
            # 메시지 발행
            self.publisher.publish(msg)
            print(f"금지 구역 삭제 메시지 발행 완료: 인덱스 {zone_index}")
            
        except Exception as e:
            print(f"금지 구역 삭제 발행 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def spin_once(self):
        """노드 스핀"""
        try:
            if self.node:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            print(f"금지 구역 삭제 발행자 스핀 중 오류 발생: {str(e)}")
    
    def cleanup(self):
        """리소스 정리"""
        try:
            if self.publisher is not None:
                self.node.destroy_publisher(self.publisher)
                self.publisher = None
            
            if self.node is not None:
                self.node.destroy_node()
                self.node = None
                
            print("금지 구역 삭제 발행자 리소스가 정리되었습니다.")
        except Exception as e:
            print(f"금지 구역 삭제 발행자 정리 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()

class PreferredAreaPublisher(QObject):
    """우선 경로 영역 발행자 클래스"""
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.publisher = None
        
        # ROS2 초기화
        init_ros_context()
        
        # 노드 초기화
        self.initialize_node()
    
    def initialize_node(self):
        """노드 초기화"""
        try:
            # 노드 생성
            if self.node is None:
                self.node = Node("preferred_path_manager")
            
            # QoS 프로파일 설정
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
            
            # 발행자 생성
            self.publisher = self.node.create_publisher(
                PolygonStamped,
                '/preferred_area',
                qos_profile
            )
            print("우선 경로 영역 발행자가 초기화되었습니다.")
            
            # 초기 데이터 발행
            self.publish_from_json()
            
        except Exception as e:
            print(f"우선 경로 영역 발행자 초기화 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def publish_preferred_area(self, area_index, points):
        """
        우선 경로 영역 발행
        
        Args:
            area_index (int): 우선 경로 영역 인덱스 (frame_id로 사용)
            points (list): 우선 경로 영역 꼭짓점 좌표 리스트 [(x1, y1), (x2, y2), ...]
        """
        try:
            if self.publisher is None:
                print("발행자가 초기화되지 않았습니다.")
                return
            
            # PolygonStamped 메시지 생성
            msg = PolygonStamped()
            
            # Header 설정 (우선 경로 영역 인덱스를 frame_id로 사용)
            msg.header = Header()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = str(area_index)  # 인덱스를 문자열로 변환하여 frame_id로 사용
            
            # 폴리곤 꼭짓점 설정
            for point in points:
                p = Point32()
                p.x = float(point[0])
                p.y = float(point[1])
                p.z = 0.0  # 2D 맵이므로 z는 0으로 설정
                msg.polygon.points.append(p)
            
            # 메시지 발행
            self.publisher.publish(msg)
            print(f"우선 경로 영역 {area_index} 발행 완료: {len(points)}개 꼭짓점")
            
        except Exception as e:
            print(f"우선 경로 영역 발행 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def publish_from_json(self):
        """
        forbidden_areas.json 파일에서 우선 경로 영역 데이터를 읽어 발행
        """
        try:
            import os
            import json
            
            # 현재 스크립트의 디렉토리 경로
            script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            # 데이터 파일 경로
            data_file = os.path.join(script_dir, "data", "forbidden_areas.json")
            
            print(f"우선 경로 영역 데이터 파일 경로: {data_file}")
            
            # JSON 파일 읽기
            with open(data_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # 우선 경로 영역 데이터만 필터링
            preferred_areas = []
            for area in data.get('areas', []):
                if area.get('area_type') == '우선 경로':
                    preferred_areas.append({
                        'index': area.get('index'),
                        'points': area.get('points', [])
                    })
            
            print(f"우선 경로 영역 개수: {len(preferred_areas)}")
            
            # 각 우선 경로 영역 데이터 발행
            for area in preferred_areas:
                index = area.get('index')
                points = area.get('points')
                
                if index and points:
                    self.publish_preferred_area(index, points)
                
        except Exception as e:
            print(f"우선 경로 영역 데이터 로드 및 발행 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def spin_once(self):
        """노드 스핀"""
        try:
            if self.node:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            print(f"우선 경로 영역 발행자 스핀 중 오류 발생: {str(e)}")
    
    def cleanup(self):
        """리소스 정리"""
        try:
            if self.publisher is not None:
                self.node.destroy_publisher(self.publisher)
                self.publisher = None
            
            if self.node is not None:
                self.node.destroy_node()
                self.node = None
                
            print("우선 경로 영역 발행자 리소스가 정리되었습니다.")
        except Exception as e:
            print(f"우선 경로 영역 발행자 정리 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
