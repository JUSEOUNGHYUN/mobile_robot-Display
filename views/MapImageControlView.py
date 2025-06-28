from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QGraphicsView, QGraphicsScene
)
from PySide6.QtCore import Qt, Signal, QPointF, QTimer, QObject, QThread
from PySide6.QtGui import QPainter, QColor, QPixmap, QTransform, QImage, qRgba
from PySide6.QtUiTools import QUiLoader
import os
import math
import numpy as np
from threading import Thread
import time
from Utils.RosTopic import MapSubscriber
from Utils.ButtonStyle import RoundButtonStyle

class LongPressButton(QPushButton):
    """롱프레스 기능이 있는 버튼"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.timer = QTimer()
        self.timer.timeout.connect(self.on_timeout)
        self.pressed.connect(self.start_timer)
        self.released.connect(self.stop_timer)
        self.repeat_interval = 50  # 반복 간격 (ms)
        self.callback = None
        
    def set_callback(self, callback):
        """콜백 함수 설정"""
        self.callback = callback
        self.clicked.connect(callback)  # 일반 클릭에도 콜백 연결
        
    def start_timer(self):
        """타이머 시작"""
        if self.callback:
            self.timer.start(self.repeat_interval)
            
    def stop_timer(self):
        """타이머 정지"""
        self.timer.stop()
        
    def on_timeout(self):
        """타이머 timeout 시 호출"""
        if self.callback:
            self.callback()

class MapImageControlView(QWidget):
    """맵 이미지 컨트롤 뷰
    
    맵 이미지를 조작하여 실제 맵 데이터와 정확하게 매핑할 수 있게 하는 화면
    """
    # 시그널 정의
    save_clicked = Signal(dict)  # 저장 버튼 클릭 시그널 (변환 정보 전달)
    cancel_clicked = Signal()    # 취소 버튼 클릭 시그널
    refresh_clicked = Signal()   # 새로고침 버튼 클릭 시그널
    
    # 이미지 변환 시그널
    move_image_signal = Signal(float, float)  # 이미지 이동 시그널 (dx, dy)
    scale_changed_signal = Signal(float)     # 크기 변경 시그널 (비율)
    rotation_changed_signal = Signal(float)  # 회전 변경 시그널 (각도)

    # 맵 업데이트 시그널 추가
    map_updated = Signal(QPixmap)

    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 맵 데이터 정보
        self.map_data = None       # 맵 데이터 픽스맵
        self.map_data_item = None  # 맵 데이터 아이템
        self.map_scene = None      # 맵 씬
        
        # 오버레이 이미지 정보
        self.map_image = None      # 오버레이 이미지
        self.map_image_item = None # 오버레이 이미지 아이템
        
        # 변환 정보
        self.translation = QPointF(0, 0)
        self.scale_factor = 1.0
        self.rotation_angle = 0
        
        # 확대/축소 및 회전 스텝 설정
        self.scale_step = 0.05  # 5%씩 확대/축소
        self.rotation_step = 1.0  # 1도씩 회전
        
        # UI 설정
        self.setup_ui()
        
        # map_scene이 None이면 직접 초기화
        if self.map_scene is None:
            self.map_scene = QGraphicsScene()
            if hasattr(self, 'map_view') and self.map_view is not None:
                self.map_view.setScene(self.map_scene)
        
        self.setup_connections()
        
        # 맵 업데이트 시그널 연결
        self.map_updated.connect(self.update_map_data)
        
        # ROS2 맵 구독자 설정
        self.setup_ros()

    def show(self):
        """화면 표시 시 위젯 상태 설정"""
        super().show()
        print("MapImageControlView show() 호출됨")

    def setup_ros(self):
        """ROS2 노드 설정"""
        try:
            print("ROS2 맵 구독자 초기화 시작...")
            self.map_subscriber = MapSubscriber()
            self.map_subscriber.map_data_received.connect(self.process_map_message)
            
            # ROS2 스핀 스레드 시작
            self.ros_thread = Thread(target=self.ros_spin)
            self.ros_thread.daemon = True
            self.ros_thread.start()
            print("ROS2 맵 구독자 초기화 완료")
            
        except Exception as e:
            print(f"ROS2 설정 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()

    def ros_spin(self):
        """ROS2 스핀 함수"""
        print("ROS2 스핀 시작")
        try:
            while True:
                if hasattr(self, 'map_subscriber'):
                    self.map_subscriber.spin_once()
                time.sleep(0.1)
                
        except Exception as e:
            print(f"ROS2 스핀 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
        finally:
            print("ROS2 스핀 종료")

    def process_map_message(self, map_data):
        """맵 메시지 처리"""
        try:
            width = map_data['width']
            height = map_data['height']
            
            if width == 0 or height == 0:
                print("맵 크기가 0입니다.")
                return
            
            # numpy 배열로 변환
            map_data_array = np.array(map_data['data']).reshape((height, width))
            
            # 좌우 반전
            map_data_array = np.fliplr(map_data_array)
            
            # QImage 생성
            image = QImage(width, height, QImage.Format_RGB32)
            
            # 픽셀 값 설정
            for y in range(height):
                for x in range(width):
                    value = map_data_array[y][x]
                    # 맵 데이터 값에 따른 색상 설정
                    if value == -1:  # 알 수 없는 영역
                        color = qRgba(128, 128, 128, 255)  # 회색
                    elif value == 0:  # 빈 공간
                        color = qRgba(255, 255, 255, 255)  # 흰색
                    else:  # 장애물
                        # 장애물 확률에 따른 그라데이션 (0~100%)
                        intensity = int((1.0 - (value / 100.0)) * 255)
                        color = qRgba(intensity, intensity, intensity, 255)
                    image.setPixel(x, y, color)
            
            # QPixmap으로 변환
            pixmap = QPixmap.fromImage(image)
            
            if not pixmap.isNull():
                # UI 스레드에서 맵 업데이트를 위해 시그널 발생
                self.map_updated.emit(pixmap)
            else:
                print("생성된 Pixmap이 null입니다!")
                
        except Exception as e:
            print(f"맵 데이터 처리 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()

    def update_map_data(self, pixmap):
        """맵 데이터 업데이트 (UI 스레드에서 호출)"""
        try:
            print("=== 맵 데이터 업데이트 시작 ===")
            if pixmap and not pixmap.isNull():
                print(f"업데이트할 Pixmap 크기: {pixmap.width()}x{pixmap.height()}")
                self.map_data = pixmap
                
                # 이전 맵 데이터 아이템 제거
                if self.map_data_item and self.map_scene:
                    print("이전 맵 데이터 아이템 제거")
                    self.map_scene.removeItem(self.map_data_item)
                
                # 새 맵 데이터 추가
                self.map_data_item = self.map_scene.addPixmap(self.map_data)
                self.map_data_item.setZValue(0)  # 맵 데이터를 아래 레이어에 배치
                print("새 맵 데이터 아이템 추가됨")
                
                # 씬 크기 설정
                scene_rect = self.map_data.rect()
                self.map_scene.setSceneRect(scene_rect)
                print(f"씬 크기 설정됨: {scene_rect.width()}x{scene_rect.height()}")
                
                # 뷰에 맞추기
                if hasattr(self, 'map_view') and self.map_view is not None:
                    self.map_view.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio)
                    print("맵 뷰 크기 조정됨")
                    
                print("맵 데이터가 성공적으로 업데이트되었습니다.")
                
                # 변환 정보 적용
                self.update_transform()
            else:
                print("유효하지 않은 Pixmap입니다!")
                    
        except Exception as e:
            print(f"맵 데이터 업데이트 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
            
        print("=== 맵 데이터 업데이트 종료 ===")

    def closeEvent(self, event):
        """위젯 종료 시 호출"""
        # ROS2 종료
        if hasattr(self, 'map_subscriber'):
            self.map_subscriber.cleanup()
        super().closeEvent(event)

    def setup_ui(self):
        """UI 요소 초기화"""
        # UI 파일 로드
        loader = QUiLoader()
        ui_file_path = os.path.join(os.path.dirname(__file__), "..", "UI", "MapImageControlView.ui")
        
        if os.path.exists(ui_file_path):
            self.ui = loader.load(ui_file_path)
            
            # 메인 레이아웃 설정
            layout = self.layout() or QVBoxLayout(self)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(0)
            layout.addWidget(self.ui)
            
            # UI 요소 연결
            self.map_view = self.ui.findChild(QGraphicsView, "map_view")
            
            # 기존 버튼을 LongPressButton으로 교체
            self.replace_button("move_up_btn")
            self.replace_button("move_down_btn")
            self.replace_button("move_left_btn")
            self.replace_button("map_move_right_2")  # 버튼 이름이 map_move_right_2로 변경됨
            self.replace_button("zoom_in_btn")
            self.replace_button("zoom_out_btn")
            self.replace_button("rotate_left_btn")
            self.replace_button("rotate_right_btn")
            
            # 저장/취소/새로고침 버튼
            self.save_btn = self.ui.findChild(QPushButton, "save_btn")
            self.cancel_btn = self.ui.findChild(QPushButton, "cancel_btn")
            self.refresh_image_control = self.ui.findChild(QPushButton, "refresh_image_control")
            
            # 맵 씬 초기화
            self.map_scene = QGraphicsScene()
            self.map_view.setScene(self.map_scene)
            
            # 맵 뷰 설정
            self.map_view.setRenderHint(QPainter.Antialiasing)
            self.map_view.setRenderHint(QPainter.SmoothPixmapTransform)
            self.map_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            self.map_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            
        else:
            print(f"UI 파일을 찾을 수 없음: {ui_file_path}")
            raise FileNotFoundError(f"UI 파일을 찾을 수 없음: {ui_file_path}")
        
    def replace_button(self, button_name):
        """기존 버튼을 LongPressButton으로 교체"""
        old_button = self.ui.findChild(QPushButton, button_name)
        if old_button:
            # 새 버튼 생성
            new_button = LongPressButton(old_button.parent())
            new_button.setObjectName(button_name)
            new_button.setText(old_button.text())
            new_button.setGeometry(old_button.geometry())
            new_button.setStyleSheet(old_button.styleSheet())
            
            # 이전 버튼의 레이아웃 정보 복사
            if old_button.parent().layout():
                for i in range(old_button.parent().layout().count()):
                    if old_button.parent().layout().itemAt(i).widget() == old_button:
                        old_button.parent().layout().removeWidget(old_button)
                        old_button.parent().layout().insertWidget(i, new_button)
                        break
            
            # 이전 버튼 제거
            old_button.deleteLater()
            
            # 새 버튼을 클래스 속성으로 저장
            setattr(self, button_name, new_button)

    def setup_connections(self):
        """시그널/슬롯 연결"""
        # 이동 버튼 연결
        if hasattr(self, 'move_up_btn') and self.move_up_btn is not None:
            self.move_up_btn.set_callback(lambda: self.move_image_signal.emit(0, -5))
            RoundButtonStyle.apply_icon_transfer_button_style(self.move_up_btn, width=60, height=60, icon_path=":/file/Registration/map_up_30.png",icon_size=70,border_radius=15, bg_color=(255, 255, 255), bg_opacity=180)
        
        if hasattr(self, 'move_down_btn') and self.move_down_btn is not None:
            self.move_down_btn.set_callback(lambda: self.move_image_signal.emit(0, 5))
            RoundButtonStyle.apply_icon_transfer_button_style(self.move_down_btn, width=60, height=60, icon_path=":/file/Registration/map_down_30.png",icon_size=70,border_radius=15, bg_color=(255, 255, 255), bg_opacity=180)
        
        if hasattr(self, 'move_left_btn') and self.move_left_btn is not None:
            self.move_left_btn.set_callback(lambda: self.move_image_signal.emit(-5, 0))
            RoundButtonStyle.apply_icon_transfer_button_style(self.move_left_btn, width=60, height=60, icon_path=":/file/Registration/map_left_30.png",icon_size=70,border_radius=15, bg_color=(255, 255, 255), bg_opacity=180)
        
        if hasattr(self, 'map_move_right_2') and self.map_move_right_2 is not None:
            self.map_move_right_2.set_callback(lambda: self.move_image_signal.emit(5, 0))
            RoundButtonStyle.apply_icon_transfer_button_style(self.map_move_right_2, width=60, height=60, icon_path=":/file/Registration/map_right_30.png",icon_size=70,border_radius=15, bg_color=(255, 255, 255), bg_opacity=180)
        
        # 크기 조정
        if hasattr(self, 'zoom_in_btn') and self.zoom_in_btn is not None:
            self.zoom_in_btn.set_callback(self.on_zoom_in)
            RoundButtonStyle.apply_icon_transfer_button_style(self.zoom_in_btn, width=60, height=60, icon_path=":/file/MapControl/zoom_in.png",icon_size=70,border_radius=15, bg_color=(255, 255, 255), bg_opacity=180)
        
        if hasattr(self, 'zoom_out_btn') and self.zoom_out_btn is not None:
            self.zoom_out_btn.set_callback(self.on_zoom_out)
            RoundButtonStyle.apply_icon_transfer_button_style(self.zoom_out_btn, width=60, height=60, icon_path=":/file/MapControl/zoom_out.png",icon_size=70,border_radius=15, bg_color=(255, 255, 255), bg_opacity=180)

        # 회전
        if hasattr(self, 'rotate_left_btn') and self.rotate_left_btn is not None:
            self.rotate_left_btn.set_callback(lambda: self.on_rotate(-self.rotation_step))
            RoundButtonStyle.apply_icon_transfer_button_style(self.rotate_left_btn, width=60, height=60, icon_path=":/file/MarkerMove/robot_rotate_left.png",icon_size=70,border_radius=15, bg_color=(255, 255, 255), bg_opacity=180)
        
        if hasattr(self, 'rotate_right_btn') and self.rotate_right_btn is not None:
            self.rotate_right_btn.set_callback(lambda: self.on_rotate(self.rotation_step))
            RoundButtonStyle.apply_icon_transfer_button_style(self.rotate_right_btn, width=60, height=60, icon_path=":/file/MarkerMove/robot_rotate_right.png",icon_size=70,border_radius=15, bg_color=(255, 255, 255), bg_opacity=180)
        
        # 이미지 이동 시그널 연결
        self.move_image_signal.connect(self.on_move_image)
        self.scale_changed_signal.connect(self.on_scale_changed)
        self.rotation_changed_signal.connect(self.on_rotation_changed)
        
        # 저장/취소/새로고침 버튼
        if hasattr(self, 'save_btn') and self.save_btn is not None:
            self.save_btn.clicked.connect(self.on_save_clicked)
        
        if hasattr(self, 'cancel_btn') and self.cancel_btn is not None:
            self.cancel_btn.clicked.connect(self.on_cancel_clicked)
            
        if hasattr(self, 'refresh_image_control') and self.refresh_image_control is not None:
            self.refresh_image_control.clicked.connect(self.on_refresh_clicked)
            RoundButtonStyle.apply_icon_transfer_button_style(self.refresh_image_control, width=60, height=60, icon_path=":/file/Registration/map_view_refresh.png",icon_size=70,border_radius=15, bg_color=(255, 255, 255), bg_opacity=180)

    def set_transform_data(self, transform_data):
        """변환 데이터 설정"""
        if not transform_data:
            transform_data = {
                'translate_x': 0,
                'translate_y': 0,
                'scale': 1.0,
                'rotation': 0
            }
            
        translation = QPointF(
            transform_data.get('translate_x', 0),
            transform_data.get('translate_y', 0)
        )
        scale = transform_data.get('scale', 1.0)
        rotation = transform_data.get('rotation', 0)
        
        # 변환 적용
        self.update_transform(translation, scale, rotation)

    def get_transform_data(self):
        """현재 변환 정보를 딕셔너리로 반환"""
        return {
            'translate_x': self.translation.x(),
            'translate_y': self.translation.y(),
            'scale': self.scale_factor,
            'rotation': self.rotation_angle
        }

    def update_transform(self, translation=None, scale=None, rotation=None):
        """변환 정보 업데이트 및 적용"""
        # 전달된 값으로 변환 정보 업데이트
        if translation is not None:
            self.translation = translation
        
        if scale is not None:
            self.scale_factor = scale
        
        if rotation is not None:
            self.rotation_angle = rotation
        
        # 맵 데이터 아이템에만 변환 적용
        if self.map_data_item and self.map_data:
            # 변환 객체 생성
            transform = QTransform()
            
            # 이미지 중심을 기준으로 변환
            center_x = self.map_data.width() / 2
            center_y = self.map_data.height() / 2
            
            # 순서: 원점으로 이동 -> 크기 조정 -> 회전 -> 다시 원래 위치로 이동 -> 추가 이동
            transform.translate(center_x + self.translation.x(), center_y + self.translation.y())
            transform.rotate(self.rotation_angle)
            transform.scale(self.scale_factor, self.scale_factor)
            transform.translate(-center_x, -center_y)
            
            # 맵 데이터 아이템에만 적용
            self.map_data_item.setTransform(transform)

    # 이벤트 핸들러
    def on_zoom_in(self):
        """확대"""
        new_scale = self.scale_factor + self.scale_step
        self.scale_changed_signal.emit(new_scale)
    
    def on_zoom_out(self):
        """축소"""
        new_scale = max(0.1, self.scale_factor - self.scale_step)  # 최소 10%
        self.scale_changed_signal.emit(new_scale)
    
    def on_scale_changed(self, scale):
        """크기 변경 시그널 처리"""
        self.scale_factor = scale
        self.update_transform()
    
    def on_rotate(self, angle):
        """회전 버튼 클릭 시 호출"""
        new_angle = self.rotation_angle + angle
        # 각도 범위 -180~180 유지
        if new_angle > 180:
            new_angle = new_angle - 360
        elif new_angle < -180:
            new_angle = new_angle + 360
        self.rotation_changed_signal.emit(new_angle)
    
    def on_rotation_changed(self, angle):
        """회전 변경 시그널 처리"""
        self.rotation_angle = angle
        self.update_transform()
    
    def on_move_image(self, dx, dy):
        """이미지 이동 시그널 처리"""
        self.translation += QPointF(dx, dy)
        self.update_transform()
    
    def on_save_clicked(self):
        """저장 버튼 클릭 시 호출"""
        transform_data = self.get_transform_data()
        self.save_clicked.emit(transform_data)
    
    def on_cancel_clicked(self):
        """취소 버튼 클릭 시 호출"""
        self.cancel_clicked.emit()
    
    def on_refresh_clicked(self):
        """새로고침 버튼 클릭 처리"""
        # 변환 정보 초기화
        self.translation = QPointF(0, 0)
        self.scale_factor = 1.0
        self.rotation_angle = 0
            
        # 변환 정보 업데이트
        self.update_transform(
            translation=self.translation,
            scale=self.scale_factor,
            rotation=self.rotation_angle
        )
        
        # 새로고침 시그널 발생
        self.refresh_clicked.emit()
    
    def resizeEvent(self, event):
        """창 크기 변경 시 호출"""
        super().resizeEvent(event)
        if self.map_scene and not self.map_scene.sceneRect().isEmpty():
            self.map_view.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio) 

    def set_map_image(self, pixmap):
        """맵 이미지(오버레이) 설정"""
        if pixmap:
            # 이미지 투명도 조정
            temp_image = pixmap.toImage()
            width = temp_image.width()
            height = temp_image.height()
            transparent_image = QImage(width, height, QImage.Format_ARGB32)
            
            # 투명도 설정 (70%)
            for y in range(height):
                for x in range(width):
                    pixel = temp_image.pixel(x, y)
                    r, g, b, a = (pixel >> 16) & 0xff, (pixel >> 8) & 0xff, pixel & 0xff, (pixel >> 24) & 0xff
                    transparent_image.setPixel(x, y, qRgba(r, g, b, int(255 * 0.7)))  # 알파값 178 (70% 불투명)
            
            self.map_image = QPixmap.fromImage(transparent_image)
            
            # map_scene이 None이면 초기화
            if self.map_scene is None:
                self.map_scene = QGraphicsScene()
                if hasattr(self, 'map_view') and self.map_view is not None:
                    self.map_view.setScene(self.map_scene)
            
            # 이전 오버레이 이미지 아이템 제거
            if self.map_image_item and self.map_scene:
                self.map_scene.removeItem(self.map_image_item)
                
            # 새 오버레이 이미지 추가
            self.map_image_item = self.map_scene.addPixmap(self.map_image)
            self.map_image_item.setZValue(1)  # 맵 이미지를 맵 데이터 위에 배치
            
            # 맵 데이터가 없는 경우 씬 크기 설정
            if not self.map_data:
                self.map_scene.setSceneRect(self.map_image.rect())
                if hasattr(self, 'map_view') and self.map_view is not None:
                    self.map_view.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio) 

    # 키보드 이벤트 처리 추가
    def keyPressEvent(self, event):
        """키보드 이벤트 처리"""
        if event.key() == Qt.Key_Left:
            self.move_image_signal.emit(-5, 0)  # 왼쪽으로 5픽셀
        elif event.key() == Qt.Key_Right:
            self.move_image_signal.emit(5, 0)   # 오른쪽으로 5픽셀
        elif event.key() == Qt.Key_Up:
            self.move_image_signal.emit(0, -5)  # 위로 5픽셀
        elif event.key() == Qt.Key_Down:
            self.move_image_signal.emit(0, 5)   # 아래로 5픽셀
        elif event.key() == Qt.Key_Plus or event.key() == Qt.Key_Equal:
            self.on_zoom_in()                   # 확대
        elif event.key() == Qt.Key_Minus:
            self.on_zoom_out()                  # 축소
        elif event.key() == Qt.Key_Comma:
            self.on_rotate(-self.rotation_step)  # 반시계 방향 회전
        elif event.key() == Qt.Key_Period:
            self.on_rotate(self.rotation_step)   # 시계 방향 회전
        else:
            super().keyPressEvent(event) 