import sys
import os
from PySide6.QtWidgets import QApplication, QStackedWidget, QWidget, QVBoxLayout, QMessageBox
from PySide6.QtCore import Qt, QObject, QTimer
from PySide6.QtGui import QFont
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from controllers.SettingsController import SettingsController
from controllers.MovingController import MovingController
from controllers.InfoController import InfoController
from controllers.LocationSelectionController import LocationSelectionController
from controllers.LocationAddController import LocationAddController
from controllers.WiFiController import WiFiController
from controllers.MoveOptionController import MoveOptionController
from models.LocationSelectionModel import LocationSelectionModel
from controllers.ArriveController import ArriveController
from controllers.ChangeLayoutController import ChangeLayoutController
from controllers.MapImageControlController import MapImageControlController
from controllers.TouchTestController import TouchTestController
from controllers.DeveloperModeController import DeveloperModeController

class MainController(QObject):
    def __init__(self, app=None):
        super().__init__()
        # 외부에서 QApplication 인스턴스 받기
        self.app = app or QApplication.instance() or QApplication(sys.argv)

        # 이전 배터리 레벨 저장용 변수
        self.previous_battery_level = None

        # ROS2 노드 초기화
        try:
            rclpy.init()
            self.ros_node = Node('mobile_robot_display')
            
            # 배터리 상태 구독자 생성
            self.battery_subscription = self.ros_node.create_subscription(
                Float32,
                'mobile_robot/battery_percent',
                self.battery_callback,
                10
            )

            # ROS2 메시지 처리를 위한 타이머 설정
            self.ros_timer = QTimer()
            self.ros_timer.timeout.connect(self.process_ros_messages)
            self.ros_timer.start(500)  # 500ms 간격으로 메시지 처리 (간격 증가)
        except Exception as e:
            print(f"[ROS2] 초기화 중 오류 발생: {str(e)}")
            self.ros_node = None
            self.ros_timer = None

        # 스택 위젯 생성 (화면 전환용)
        self.stack = QStackedWidget()

        # 모델 초기화
        self.selection_model = LocationSelectionModel()

        # 이전 화면 기록
        self.previous_view = None

        # 화면 이동 히스토리 스택
        self.view_history = []

        # 현재 이동 중인 목적지 정보
        self.current_destination = None

        # 컨트롤러 초기화
        self.selection_controller = LocationSelectionController(self.stack, self.selection_model, self)
        self.edit_controller = LocationAddController(self.stack, self.selection_model, self)
        self.settings_controller = SettingsController(self.stack, self)
        self.moving_controller = MovingController(self.stack, self)
        self.info_controller = InfoController(self.stack, self)
        self.wifi_controller = WiFiController(self.stack, self)
        self.arrive_controller = ArriveController(self.stack, self)
        self.move_option_controller = MoveOptionController(self.stack, self)
        self.change_layout_controller = ChangeLayoutController(self.stack, self.selection_model)
        self.map_image_control_controller = MapImageControlController(self.stack)
        self.touch_test_controller = TouchTestController(self.stack, self)
        self.developer_mode_controller = DeveloperModeController(self.stack, self)

        # 기타 설정 컨트롤러 추가
        from controllers.OtherSettingController import OtherSettingController
        self.other_setting_controller = OtherSettingController(self.stack, self)

        # 컨트롤러 간의 시그널 연결 - 모든 컨트롤러가 초기화된 후에 연결합니다
        self._connect_signals()

    def _connect_signals(self):
        # LocationController 시그널 연결
        self.selection_controller.connect_to_moving_controller(self.moving_controller)
        self.selection_controller.connect_to_settings_controller(self.settings_controller)

        # ChangeLayoutController 시그널 연결
        self.change_layout_controller.layout_changed.connect(self.show_selection_view)
        
        # 상태 바 와이파이 아이콘 클릭 시그널 연결
        self.connect_status_bar_wifi()

    def battery_callback(self, msg):
        """배터리 상태 업데이트 콜백
        Args:
            msg (Float32): 배터리 잔량 메시지
        """
        # 배터리 잔량을 정수로 변환
        battery_level = int(msg.data)
        
        # 이전 값과 다를 때만 처리
        if self.previous_battery_level != battery_level:
            # 배터리 잔량은 항상 출력
            print(f"[배터리 상태] 현재 잔량: {battery_level}%")
            
            # 20%, 40%, 60%, 80%, 100%일 때만 상태바 업데이트 메시지 출력
            if battery_level in [20, 40, 60, 80, 100]:
                # 모든 뷰의 상태바 업데이트
                for i in range(self.stack.count()):
                    view = self.stack.widget(i)
                    if hasattr(view, 'status_bar'):
                        view.status_bar.update_battery_status(msg.data)
                        #print(f"[배터리 상태] {view.__class__.__name__} 상태바 업데이트 완료")
            else:
                # 상태바만 업데이트하고 메시지는 출력하지 않음
                for i in range(self.stack.count()):
                    view = self.stack.widget(i)
                    if hasattr(view, 'status_bar'):
                        view.status_bar.update_battery_status(msg.data)
            
            self.previous_battery_level = battery_level
        else:
            # 값이 같을 때는 상태바만 업데이트
            for i in range(self.stack.count()):
                view = self.stack.widget(i)
                if hasattr(view, 'status_bar'):
                    view.status_bar.update_battery_status(msg.data)

    def process_ros_messages(self):
        """ROS2 메시지 처리"""
        if not self.ros_node:
            return

        try:
            # wait set 크기 제한을 위한 짧은 타임아웃 설정
            rclpy.spin_once(self.ros_node, timeout_sec=0.01)
        except IndexError as e:
            print(f"[ROS2] wait set 인덱스 오류: {str(e)}")
            # 노드 재초기화 시도
            self._reinitialize_ros_node()
        except Exception as e:
            print(f"[ROS2] 메시지 처리 중 오류 발생: {str(e)}")

    def _reinitialize_ros_node(self):
        """ROS2 노드 재초기화"""
        try:
            if self.ros_node:
                self.ros_node.destroy_node()
            
            rclpy.shutdown()
            rclpy.init()
            
            self.ros_node = Node('mobile_robot_display')
            self.battery_subscription = self.ros_node.create_subscription(
                Float32,
                'mobile_robot/battery_percent',
                self.battery_callback,
                10
            )
            print("[ROS2] 노드 재초기화 완료")
        except Exception as e:
            print(f"[ROS2] 노드 재초기화 실패: {str(e)}")
            self.ros_node = None

    def start(self):
        # 초기 화면 설정
        locations = self.selection_model.get_all_locations()
        if not locations:
            self.selection_controller.show()
        else:
            self.selection_controller.show()

        self.stack.showFullScreen()
        sys.exit(self.app.exec())

    # 앱에서 이전 화면으로 돌아가는 핵심 내비게이션
    # 화면 이동 히스토리를 추적하여 사용자가 이전에 보던 화면
    def show_previous_view(self):
        # 현재 화면 가져오기
        current_view = self.stack.currentWidget()

        # 이동 히스토리가 있는 경우
        if len(self.view_history) > 1:

            # 히스토리에서 현재 화면을 찾기
            try:
                current_index = self.view_history.index(current_view)

                # 현재 화면이 히스토리의 첫 번째가 아니면 이전 화면으로 이동
                if current_index > 0:
                    prev_view = self.view_history[current_index - 1]
                    self.stack.setCurrentWidget(prev_view)
                    return
                else:
                    print("[MainController] 히스토리의 첫 번째 화면입니다.")
            except ValueError:
                print(f"[MainController] 현재 화면이 히스토리에 없습니다: {current_view.__class__.__name__}")

        # 기존 로직 사용 (히스토리가 없거나 현재 화면이 히스토리에 없을 경우)
        if self.previous_view:
            # 이전 화면이 스택에 있는지 확인
            found = False
            for i in range(self.stack.count()):
                if self.stack.widget(i) == self.previous_view:
                    found = True
                    break
            
            if found:
                # 이전 화면이 스택에 있으면 전환
                self.stack.setCurrentWidget(self.previous_view)
            else:
                # 이전 화면이 없는 경우 기본 화면으로
                self.selection_controller.show()
        else:
            # 이전 화면이 없는 경우 기본 화면으로
            self.selection_controller.show()
    
    def set_previous_view(self, view):
        self.previous_view = view
        
        # 뷰 히스토리에 추가 (중복 방지)
        if self.view_history and self.view_history[-1] == view:
            # 이미 히스토리 마지막에 같은 뷰가 있으면 추가하지 않음
            pass
        else:
            # 히스토리에 추가
            self.view_history.append(view)

    def quit_application(self):
        """애플리케이션 종료"""
        try:
            if self.ros_node:
                self.ros_node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f"[ROS2] 종료 중 오류 발생: {str(e)}")
        self.app.quit()

    def show_selection_view(self):
        """목적지 선택 화면으로 이동"""
        # 현재 화면이 도착 화면인 경우 위치 고정 상태 확인
        if self.stack.currentWidget() == self.arrive_controller.arrive_view:
            if self.arrive_controller.is_fixed():
                return
            print("[MainController] 도착 화면에서 목적지 선택 화면으로 이동. 도착 컨트롤러 정리.")
            self.arrive_controller.cleanup()

        # 목적지 선택 화면 표시 및 갱신
        self.selection_controller.show()

    def show_add_location(self):
        self.edit_controller.show_add()

    def show_edit_location(self, location_data):
        self.edit_controller.show_edit(location_data)

    def connect_status_bar_wifi(self):
        # 모든 뷰 리스트 가져오기
        views = []
        for i in range(self.stack.count()):
            view = self.stack.widget(i)
            views.append(view)

        # 각 뷰의 상태 바 와이파이 아이콘 클릭 시그널 연결
        for view in views:
            if hasattr(view, 'status_bar') and hasattr(view.status_bar, 'wifi_clicked'):
                # 뷰의 상태바에 wifi_clicked 시그널이 있으면 연결
                view.status_bar.wifi_clicked.connect(self.wifi_controller.show)

    def show_map_image_control(self):
        """맵 이미지 컨트롤 화면 표시"""
        try:
            print("[MainController] 맵 이미지 컨트롤 화면으로 전환합니다.")
            if hasattr(self, 'map_image_control_controller'):
                success = self.map_image_control_controller.show()
                if not success:
                    print("[MainController] 맵 이미지 컨트롤 화면 표시 실패")
            else:
                print("[MainController] 맵 이미지 컨트롤 컨트롤러가 초기화되지 않았습니다.")
        except Exception as e:
            print(f"[MainController] 맵 이미지 컨트롤 화면 표시 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()

    def show_touch_test_view(self):
        """터치 테스트 화면 표시"""
        self.touch_test_controller.show()

def main():
    """메인 함수"""
    app = QApplication(sys.argv)
    
    # 전역 폰트 설정
    app.setFont(QFont('Arial', 12))
    
    # 창 생성
    widget = QWidget()
    stack = QStackedWidget()
    
    layout = QVBoxLayout(widget)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.addWidget(stack)
    
    # 메인 컨트롤러 초기화
    controller = MainController()
    
    # 창 설정 및 표시
    widget.setWindowTitle("TF로봇 앱")
    widget.resize(800, 600)
    widget.showMaximized()
    
    # 컨트롤러 시작
    controller.start()
    
    sys.exit(app.exec())