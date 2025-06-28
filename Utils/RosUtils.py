from PySide6.QtCore import QTimer, QThread, QObject, Signal
import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
import time

class ROS2Thread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.running = True
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def run(self):
        try:
            while self.running and rclpy.ok():
                try:
                    # 단일 executor를 사용해 spin_once 호출로 인한 충돌 방지
                    self.executor.spin_once(timeout_sec=0.1)
                    time.sleep(0.001)  # CPU 부하 감소를 위한 짧은 대기
                except ExternalShutdownException:
                    print("ROS2 노드가 외부에서 종료되었습니다.")
                    break
                except Exception as e:
                    print(f"ROS2Thread 실행 중 오류: {e}")
                    break
        except Exception as e:
            print(f"ROS2Thread 오류: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.cleanup()

    def cleanup(self):
        """리소스 정리"""
        try:
            # 종료 시 항상 노드를 executor에서 제거
            if self.node and self.executor:
                try:
                    print("[ROS2Thread] 실행기에서 노드 제거 중...")
                    self.executor.remove_node(self.node)
                except Exception as e:
                    print(f"[ROS2Thread] 노드 제거 중 오류: {e}")
                
                try:
                    print("[ROS2Thread] 실행기 종료 중...")
                    self.executor.shutdown()
                    self.executor = None
                except Exception as e:
                    print(f"[ROS2Thread] 실행기 종료 중 오류: {e}")
                
                # 노드 참조만 제거
                self.node = None
        except Exception as e:
            print(f"[ROS2Thread] 리소스 정리 중 오류: {e}")

    def stop(self):
        """스레드 종료"""
        print("[ROS2Thread] 스레드 종료 시작...")
        
        # 실행 중지 플래그 설정
        self.running = False
        
        # 좀더 안전한 종료 시퀀스
        success = self.wait(2000)  # 최대 2초 대기
        if not success or self.isRunning():
            print("[ROS2Thread] 자연 종료 실패, 강제 종료 시도 중...")
            try:
                # 실행기 강제 종료 시도 (스레드가 아직 실행 중일 경우)
                if self.node and self.executor:
                    try:
                        self.executor.remove_node(self.node)
                    except Exception as e:
                        print(f"[ROS2Thread] 노드 제거 중 오류: {e}")
                
                # 강제 종료
                if self.isRunning():
                    print("[ROS2Thread] 스레드 강제 종료 중...")
                    self.terminate()
                    self.wait(1000)  # 추가 1초 대기
                    
                # 최종 확인
                if self.isRunning():
                    print("[ROS2Thread] 스레드가 여전히 실행 중입니다. 자원 누수 가능성 있음.")
                else:
                    print("[ROS2Thread] 스레드가 성공적으로 종료되었습니다.")
            except Exception as e:
                print(f"[ROS2Thread] 스레드 강제 종료 중 오류: {e}")
        else:
            print("[ROS2Thread] 스레드가 성공적으로 종료되었습니다.")

class RosNode(QObject):
    def __init__(self, node_name):
        super().__init__()
        self.node_name = node_name
        self.node = None
        self.executor = None
        self.timer = None
        self.is_running = False
        
    def initialize(self):
        """ROS2 노드 초기화"""
        try:
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node(self.node_name)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            return True
        except Exception as e:
            print(f"ROS2 노드 초기화 실패: {str(e)}")
            self.cleanup()
            return False
            
    def run(self):
        """ROS2 노드 실행"""
        if not self.is_running:
            return
            
        try:
            if self.executor and rclpy.ok():
                self.executor.spin_once(timeout_sec=0.1)
        except ExternalShutdownException:
            print("ROS2 노드가 외부에서 종료되었습니다.")
            self.cleanup()
        except Exception as e:
            print(f"ROS2 노드 실행 중 오류 발생: {str(e)}")
            self.cleanup()
            
    def start(self):
        """타이머로 ROS2 노드 실행 시작"""
        if self.is_running:
            return True

        if not self.node:
            if not self.initialize():
                return False
                
        self.timer = QTimer()
        self.timer.timeout.connect(self.run)
        self.timer.start(100)  # 100ms 간격으로 실행
        self.is_running = True
        return True
        
    def cleanup(self):
        """리소스 정리"""
        print("[RosNode] 리소스 정리 시작...")
        
        try:
            self.is_running = False
            
            # 타이머 중지
            if self.timer:
                print("[RosNode] 타이머 중지 중...")
                self.timer.stop()
                self.timer.deleteLater()  # 명시적으로 삭제
                self.timer = None
                
            # 노드 정리    
            if self.node:
                try:
                    # 노드 정리 전에 실행기에서 제거
                    if self.executor:
                        print("[RosNode] 실행기에서 노드 제거 중...")
                        self.executor.remove_node(self.node)
                        self.executor.shutdown()
                        self.executor = None
                    
                    # 노드 파괴
                    print("[RosNode] 노드 정리 중...")
                    self.node.destroy_node()
                except Exception as e:
                    print(f"[RosNode] 노드 정리 중 오류 발생: {str(e)}")
                finally:
                    self.node = None
                
            print("[RosNode] 리소스 정리 완료")
        except Exception as e:
            print(f"[RosNode] 리소스 정리 중 일반 오류: {str(e)}")