from PySide6.QtCore import QObject, Signal, QTimer, Qt, QEvent, QMetaObject, Slot, QPointF, QThread
from PySide6.QtGui import QImage, QPixmap, QColor, QPen, QBrush, QPolygonF, QTransform
from PySide6.QtWidgets import (
    QGraphicsScene, QGraphicsPixmapItem, QGraphicsRectItem,
    QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsPolygonItem, QGraphicsView,
    QGraphicsItem, QApplication
)
import numpy as np
from Utils.RosTopic import MapSubscriber
import os
import math
from PySide6.QtGui import QPainter
from PySide6.QtCore import QRectF

class MapMarker(QGraphicsItem):
    """맵에 표시할 마커 클래스"""
    
    def __init__(self, pos, yaw=0.0, parent=None):
        super().__init__(parent)
        self.pos = pos
        self.yaw = yaw
        self.setData(0, "marker")
        self.setZValue(12)  # 맵 위에 렌더링되도록 z값 설정
        self.setPos(pos)
        
    def boundingRect(self):
        """아이템의 경계 사각형 반환"""
        # 원과 화살표를 모두 포함하는 크기
        return QRectF(-20, -20, 40, 40)
        
    def paint(self, painter, option, widget):
        """아이템 그리기"""
        # 안티앨리어싱 활성화
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 원 (로봇 위치) 그리기
        circle_radius = 3.0
        painter.setPen(QPen(QColor(0, 0, 255), 2))
        painter.setBrush(QBrush(QColor(0, 0, 255, 150)))
        painter.drawEllipse(-circle_radius, -circle_radius, circle_radius * 2, circle_radius * 2)
        
        # 화살표 (로봇 방향) 그리기
        arrow_length = 15.0
        arrow_width = 6.0
        
        # 화살표 시작점과 끝점
        start_x = 0  # 원점에서 시작
        start_y = 0
        end_x = arrow_length * math.cos(self.yaw)
        end_y = arrow_length * math.sin(self.yaw)
        
        # 화살표 몸통
        painter.setPen(QPen(QColor(255, 0, 0), 2))
        painter.drawLine(start_x, start_y, end_x, end_y)
        
        # 화살촉
        arrow_tip_length = 8.0
        tip_base_x = end_x - arrow_tip_length * math.cos(self.yaw)
        tip_base_y = end_y - arrow_tip_length * math.sin(self.yaw)
        
        tip_left_x = tip_base_x + arrow_width/2 * math.cos(self.yaw + math.pi/2)
        tip_left_y = tip_base_y + arrow_width/2 * math.sin(self.yaw + math.pi/2)
        
        tip_right_x = tip_base_x + arrow_width/2 * math.cos(self.yaw - math.pi/2)
        tip_right_y = tip_base_y + arrow_width/2 * math.sin(self.yaw - math.pi/2)
        
        # 화살촉 폴리곤 그리기
        painter.setBrush(QBrush(QColor(255, 0, 0, 200)))
        arrow_tip = QPolygonF([
            QPointF(end_x, end_y),
            QPointF(tip_left_x, tip_left_y),
            QPointF(tip_right_x, tip_right_y)
        ])
        painter.drawPolygon(arrow_tip)

class MapManager(QObject):
    # 시그널 정의
    position_clicked = Signal(float, float)  # x, y 좌표를 전달하는 시그널
    map_updated = Signal(object)  # 맵 업데이트 시그널
    
    # 맵 제어 결과 시그널 추가
    map_zoomed_in = Signal()
    map_zoomed_out = Signal()
    map_moved = Signal(str)  # 방향을 전달하는 시그널 ('up', 'down', 'left', 'right')
    robot_moved = Signal(str)  # 방향을 전달하는 시그널 ('up', 'down', 'left', 'right', 'rotate_right')
    
    def __init__(self, map_view, map_scene, load_image=True, apply_transform=True, parent=None):
        super().__init__(parent)
        self.map_view = map_view
        self.map_scene = map_scene
        self.map_subscriber = None
        self.map_timer = None
        self.map_info = None
        self.map_timestamp = None
        self.marker_item = None  # 마커 아이템 추가
        self.load_image = load_image  # 이미지 로드 여부
        self.apply_transform = apply_transform  # 변환 정보 적용 여부
        self.is_subscribing = False  # 구독 상태 초기화
        
        # 맵 기본 정보 설정 (실제 맵과 동일한 값 사용)
        self.default_map_width = 552  # 픽셀 단위
        self.default_map_height = 491  # 픽셀 단위
        self.default_resolution = 0.05000000074505806  # 미터/픽셀 (실제 맵 해상도와 정확히 일치)
        self.default_origin_x = -13.0  # 미터
        self.default_origin_y = -6.26   # 미터
        
        # 맵 정보가 없을 때 사용할 기본 맵 정보 설정
        self.setup_default_map_info()
        
        # 링고벨 관련 변수
        self.ringo_positions = []  # 링고벨 위치 목록
        self.clicked_ringo_uuid = None  # 클릭된 링고벨의 UUID
        
        # 마지막 클릭 위치 및 방향 정보 저장 변수
        self.last_clicked_pos = None
        self.last_clicked_real_pos = None
        self.original_scene_pos = None     # 변환 전 원본 씬 좌표
        self.last_yaw = 0.0
        
        # 금지구역 관련 변수
        self.area_buttons = []  # 금지구역 버튼 목록
        
        # 맵 이미지 오버레이
        self.map_overlay_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "resources", "map_original_image.png")
        self.map_overlay_pixmap = None

        # 영역 설정 모드 상태 변수 추가
        self.is_area_setting_screen = False  
        
        # 이미지 로드 처리
        if load_image and os.path.exists(self.map_overlay_path):
            self.map_overlay_pixmap = QPixmap(self.map_overlay_path)
            print(f"맵 오버레이 이미지 로드됨: {self.map_overlay_path}")
        elif not load_image:
            print("이미지 로드가 비활성화되었습니다.")
        else:
            print(f"맵 오버레이 이미지를 찾을 수 없음: {self.map_overlay_path}")
        
        # 맵 오버레이 변환 정보
        self.overlay_transform = {
            'translate_x': 0,
            'translate_y': 0,
            'scale': 1.0,
            'rotation': 0
        }
        
        # 변환 정보 불러오기
        self.load_overlay_transform()
        
        # 마우스 이벤트 설정
        self.map_view.viewport().installEventFilter(self)
        
        # 맵 업데이트 시그널 연결
        self.map_updated.connect(self._update_map_ui, Qt.QueuedConnection)
        
    def setup_default_map_info(self):
        """기본 맵 정보 설정"""
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import MapMetaData
        
        try:
            # MapMetaData 객체 생성
            self.map_info = MapMetaData()
            self.map_info.width = self.default_map_width
            self.map_info.height = self.default_map_height
            self.map_info.resolution = self.default_resolution
            
            # Pose 객체 생성
            origin_pose = Pose()
            origin_pose.position.x = self.default_origin_x
            origin_pose.position.y = self.default_origin_y
            origin_pose.position.z = 0.0
            
            self.map_info.origin = origin_pose
            print(f"기본 맵 정보 설정됨: 크기={self.default_map_width}x{self.default_map_height}, " + 
                  f"해상도={self.default_resolution}, 원점=({self.default_origin_x}, {self.default_origin_y})")
        except Exception as e:
            print(f"기본 맵 정보 설정 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()

    def set_map_info(self, width, height, resolution, origin_x, origin_y):
        """맵 정보 직접 설정"""
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import MapMetaData
        
        try:
            # MapMetaData 객체 생성
            self.map_info = MapMetaData()
            self.map_info.width = width
            self.map_info.height = height
            self.map_info.resolution = resolution
            
            # Pose 객체 생성
            origin_pose = Pose()
            origin_pose.position.x = origin_x
            origin_pose.position.y = origin_y
            origin_pose.position.z = 0.0
            
            self.map_info.origin = origin_pose
            print(f"맵 정보 설정됨: 크기={width}x{height}, 해상도={resolution}, 원점=({origin_x}, {origin_y})")
            return True
        except Exception as e:
            print(f"맵 정보 설정 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
            return False

    def start_subscription(self):
        """맵 구독 시작"""
        try:
            if self.is_subscribing:
                print("이미 구독 중입니다.")
                return
                
            if self.map_subscriber is None:
                self.map_subscriber = MapSubscriber()
                self.map_subscriber.map_updated.connect(self.update_map)
                
                # ROS2 타이머 설정
                self.map_timer = QTimer()
                self.map_timer.timeout.connect(self.map_subscriber.spin_once)
                self.map_timer.start(100)  # 100ms 간격으로 ROS2 스핀
                
                self.is_subscribing = True
                print("맵 구독 시작됨")
                
                # 초기 맵 요청
                if self.map_subscriber:
                    self.map_subscriber.request_map()
        except Exception as e:
            print(f"맵 구독 시작 중 오류 발생: {str(e)}")
            self.cleanup()
            
    def stop_subscription(self):
        """맵 구독 중지"""
        try:
            if not self.is_subscribing:
                return
                
            if self.map_timer is not None:
                self.map_timer.stop()
                self.map_timer = None
                
            if self.map_subscriber is not None:
                self.map_subscriber.cleanup()
                self.map_subscriber = None
                
            # 맵 씬 초기화
            self.map_scene.clear()
            self.map_info = None
            self.is_subscribing = False
            print("맵 구독 중지됨")
        except Exception as e:
            print(f"맵 구독 중지 중 오류 발생: {str(e)}")
            self.cleanup()
            
    def cleanup(self):
        """리소스 정리"""
        try:
            # 타이머 정리
            if self.map_timer is not None:
                self.map_timer.stop()
                self.map_timer = None
            
            # 구독 중지
            if self.map_subscriber is not None:
                self.map_subscriber.cleanup()
                self.map_subscriber = None
            
            # 맵 씬 정리
            if self.map_scene is not None:
                self.map_scene.clear()
            
            # 마커 아이템 정리
            self.marker_item = None
            
            # 상태 초기화
            self.map_info = None
            self.is_subscribing = False
            self.last_clicked_pos = None
            self.last_clicked_real_pos = None
            self.original_scene_pos = None
            
            print("맵 매니저 리소스 정리 완료")
        except Exception as e:
            print(f"맵 매니저 리소스 정리 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()

    @Slot(object)
    def update_map(self, msg):
        print("[MapManager] update_map 호출")
        try:
            print("[MapManager] 맵 데이터 수신됨")
            self.map_info = msg.info
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            print(f"[MapManager] 맵 크기: {width}x{height}, 해상도: {resolution}")
            map_data = np.array(msg.data).reshape(height, width)
            image = QImage(width, height, QImage.Format_ARGB32)
            image.fill(Qt.transparent)
            for y in range(height):
                for x in range(width):
                    # 좌우 반전을 위해 x 좌표를 반전 (width - 1 - x)
                    value = map_data[y, width - 1 - x]
                    if value == -1:
                        color = QColor(128, 128, 128, 255)
                    elif value == 0:
                        color = QColor(255, 255, 255, 255)
                    elif value == 100:
                        color = QColor(0, 0, 0, 255)
                    else:
                        gray = 255 - int((value / 100) * 255)
                        color = QColor(gray, gray, gray, 255)
                    image.setPixelColor(x, y, color)
            pixmap = QPixmap.fromImage(image)
            print(f"[MapManager] pixmap 생성 완료, size: {pixmap.size()}")
            self._update_map_ui(pixmap)
            
            # 맵 이미지 자동 저장 활성화
            self.save_map_as_png()
            
            print("[MapManager] update_map 끝")
        except Exception as e:
            print(f"[MapManager] 맵 업데이트 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()

    @Slot(object)
    def _update_map_ui(self, pixmap):
        print("[MapManager] _update_map_ui 호출")
        try:
            self.map_scene.clear()
            
            # 고정 크기 설정
            fixed_width = 552
            fixed_height = 491
            self.map_scene.setSceneRect(0, 0, fixed_width, fixed_height)
            
            # 맵 데이터가 있는 경우에만 기본 맵 추가
            if isinstance(pixmap, QPixmap) and not self.load_image:
                map_item = self.map_scene.addPixmap(pixmap)
                map_item.setZValue(-2)  # z-index를 낮게 설정하여 맨 아래에 위치시킴
                print(f"[MapManager] 맵 크기: {pixmap.width()} x {pixmap.height()} 픽셀")
            
            # 하얀색 배경 패널 추가 (고정된 크기 사용)
            white_panel = QGraphicsRectItem(0, 0, fixed_width, fixed_height)
            white_panel.setBrush(QBrush(QColor(255, 255, 255)))  # 하얀색으로 설정
            white_panel.setPen(QPen(Qt.NoPen))  # 테두리 없음
            white_panel.setZValue(-1)  # 맵보다 위, 오버레이와 같은 레벨
            self.map_scene.addItem(white_panel)
            
            # 맵 오버레이 이미지 추가
            if self.load_image and self.map_overlay_pixmap:
                print(f"[MapManager] 오버레이 원본 크기: {self.map_overlay_pixmap.width()} x {self.map_overlay_pixmap.height()} 픽셀")
                
                # 항상 고정된 크기로 이미지 조정
                scaled_overlay = self.map_overlay_pixmap.scaled(
                    fixed_width, 
                    fixed_height,
                    Qt.KeepAspectRatio,  # 가로세로 비율 유지
                    Qt.SmoothTransformation
                )
                
                # 오버레이 이미지를 맵 위에 추가
                overlay_item = self.map_scene.addPixmap(scaled_overlay)
                overlay_item.setZValue(-1)  # 맵보다 위에, 하얀색 패널과 같은 레벨
                overlay_item.setOpacity(1.0)  # 투명도 설정 (0.0 ~ 1.0)
                
                # 변환 정보 적용 (apply_transform이 True인 경우에만)
                if self.apply_transform and self.overlay_transform:
                    transform = QTransform()
                    center_x = scaled_overlay.width() / 2
                    center_y = scaled_overlay.height() / 2
                    
                    # 순서: 원점으로 이동 -> 크기 조정 -> 회전 -> 다시 원래 위치로 이동 -> 추가 이동
                    transform.translate(center_x + self.overlay_transform['translate_x'], 
                                    center_y + self.overlay_transform['translate_y'])
                    transform.rotate(self.overlay_transform['rotation'])
                    transform.scale(self.overlay_transform['scale'], 
                                self.overlay_transform['scale'])
                    transform.translate(-center_x, -center_y)
                    
                    # 변환 적용
                    overlay_item.setTransform(transform)
                    print(f"[MapManager] 맵 변환 정보 적용됨: {self.overlay_transform}")
                else:
                    print("[MapManager] 맵 변환 정보 적용하지 않음")
                
                print(f"[MapManager] 조정된 오버레이 크기: {scaled_overlay.width()} x {scaled_overlay.height()} 픽셀")
            
            # 맵 업데이트 후 마지막 클릭 위치에 마커 다시 표시
            if self.last_clicked_pos is not None:
                # 저장된 방향 정보를 사용하여 마커 표시
                self.show_click_marker(self.last_clicked_pos, self.last_yaw)
            
            # 뷰에 맞게 씬 조정 (화면에 꽉 차도록)
            self.map_view.resetTransform()  # 변환 행렬 초기화
            self.map_view.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio)
            QTimer.singleShot(100, lambda: self.map_view.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio))
            self.map_view.viewport().update()
            
            print(f"[MapManager] _update_map_ui 완료 - 씬 크기: {self.map_scene.sceneRect().size()}")
        except Exception as e:
            print(f"[MapManager] 맵 UI 업데이트 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()

    def eventFilter(self, obj, event):
        """
        이벤트 필터 - 맵 클릭 이벤트 처리 및 좌표 변환
        
        화면 좌표계(씬 좌표계)에서 실제 좌표계(ROS 좌표계)로의 변환:
        - 화면 좌표: 픽셀 단위, Qt QGraphicsScene 내 좌표
        - 실제 좌표: 미터 단위, ROS에서 사용하는 맵 좌표
        
        변환 공식:
        x 좌표 (좌우 반전 고려):
        1. adjusted_scene_x = scene_width - scene_pos.x()  # 좌우 반전
        2. x = adjusted_scene_x * resolution + origin.x    # 미터 단위 변환
        
        y 좌표:
        y = scene_pos.y() * resolution + origin.y
        
        여기서:
        - resolution: 맵 해상도 (미터/픽셀)
        - origin: 맵 원점 좌표 (미터 단위)
        - scene_width: 맵 씬의 전체 너비 (픽셀 단위)
        """
        if obj == self.map_view.viewport():
            if event.type() == QEvent.MouseButtonPress:
                if event.button() == Qt.LeftButton:
                    # 클릭 위치를 씬 좌표로 변환
                    scene_pos = self.map_view.mapToScene(event.pos())
                    self.last_clicked_pos = scene_pos  # 클릭 위치 저장 (화면 좌표)
                    self.last_yaw = 0.0  # 초기 방향은 0으로 설정
                    
                    # 맵 정보가 없으면 기본값 사용
                    if not self.map_info:
                        self.setup_default_map_info()
                    
                    # 씬의 크기 확인
                    scene_rect = self.map_scene.sceneRect()
                    if scene_rect.isEmpty() and self.map_overlay_pixmap:
                        scene_rect = self.map_overlay_pixmap.rect()
                    
                    # 좌우 반전된 씬에서의 좌표 조정
                    if self.overlay_transform:
                        # 변환 행렬 생성 (맵에 적용된 것과 동일)
                        transform = QTransform()
                        center_x = scene_rect.width() / 2
                        center_y = scene_rect.height() / 2
                        
                        # 맵에 적용된 변환과 동일한 순서로 변환 행렬 구성
                        transform.translate(center_x + self.overlay_transform['translate_x'], 
                                          center_y + self.overlay_transform['translate_y'])
                        transform.rotate(self.overlay_transform['rotation'])
                        transform.scale(self.overlay_transform['scale'], 
                                       self.overlay_transform['scale'])
                        transform.translate(-center_x, -center_y)
                        
                        # 변환 행렬의 역행렬 계산
                        inverse_transform = transform.inverted()[0]
                        
                        # 클릭 좌표에 역변환 적용
                        original_pos = inverse_transform.map(scene_pos)
                        
                        # 좌우 반전 적용
                        adjusted_scene_x = scene_rect.width() - original_pos.x()
                        
                        # 실제 좌표계로 변환
                        x = adjusted_scene_x * self.map_info.resolution + self.map_info.origin.position.x
                        y = original_pos.y() * self.map_info.resolution + self.map_info.origin.position.y
                    else:
                        # 변환 정보가 없는 경우 기존 방식 사용
                        adjusted_scene_x = scene_rect.width() - scene_pos.x()
                        x = adjusted_scene_x * self.map_info.resolution + self.map_info.origin.position.x
                        y = scene_pos.y() * self.map_info.resolution + self.map_info.origin.position.y

                    self.last_clicked_real_pos = (x, y)  # 실제 좌표 저장

                    if self.is_area_setting_screen == True:
                        print(self.is_area_setting_screen)
                        print("영역 설정 모드에서는 마커를 표시하지 않습니다.")
                        return

                    # 디버그 정보 출력
                    print(f"맵 클릭 - 좌표 변환 정보:")
                    print(f"  - 원본 클릭 좌표: ({scene_pos.x():.1f}, {scene_pos.y():.1f}) 픽셀")
                    if self.overlay_transform:
                        print(f"  - 역변환 후 좌표: ({original_pos.x():.1f}, {original_pos.y():.1f}) 픽셀")
                    print(f"  - 조정된 좌표: ({adjusted_scene_x:.1f}, {original_pos.y() if self.overlay_transform else scene_pos.y():.1f}) 픽셀")
                    print(f"  - 실제 좌표: ({x:.3f}, {y:.3f}) 미터")
                    print(f"  - 맵 해상도: {self.map_info.resolution} 미터/픽셀")
                    print(f"  - 맵 원점: ({self.map_info.origin.position.x}, {self.map_info.origin.position.y}) 미터")
                    
                    # 클릭 위치를 시그널로 전달 (실제 좌표 전달)
                    self.position_clicked.emit(x, y)
                    

                    # 클릭 위치에 마커 표시 (기본 방향 0으로 설정)
                    self.show_click_marker(scene_pos, self.last_yaw)
                    return True
        return super().eventFilter(obj, event)
        
    def show_click_marker(self, pos, yaw=0.0):
        """클릭 위치에 마커 표시"""
        self.last_clicked_pos = pos
        self.last_yaw = yaw

        try:
            # 마커 아이템이 이미 있으면 제거
            if self.marker_item:
                # 삭제된 객체에 대한 접근 오류 방지
                try:
                    if self.marker_item.scene() == self.map_scene:
                        self.map_scene.removeItem(self.marker_item)
                except RuntimeError:
                    # 이미 삭제된 객체일 경우 오류 무시
                    print("마커 제거 오류 무시: 이미 삭제됨")
                self.marker_item = None

            # 클릭 위치에 새 마커 추가
            self.marker_item = MapMarker(pos, yaw)
            self.map_scene.addItem(self.marker_item)
        except Exception as e:
            print(f"마커 표시 중 오류 발생: {str(e)}")

    def update_marker(self, real_x, real_y, yaw=0.0):
        """
        실제 좌표(미터 단위)를 씬 좌표로 변환하여 마커 표시
        
        Args:
            real_x (float): 실제 x 좌표 (미터)
            real_y (float): 실제 y 좌표 (미터)
            yaw (float): 로봇 방향 (라디안)
        """
        try:
            # 씬의 크기 확인
            scene_rect = self.map_scene.sceneRect()
            if scene_rect.isEmpty() and self.map_overlay_pixmap:
                scene_rect = self.map_overlay_pixmap.rect()
            
            if scene_rect.isEmpty():
                print("씬이 비어있습니다.")
                return False

            if self.map_info:
                # 맵 정보가 있는 경우 맵 기반 좌표 변환
                scene_x = (real_x - self.map_info.origin.position.x) / self.map_info.resolution
                # 좌우 반전된 맵에서 x좌표 반전 (맵 너비 사용)
                scene_x = self.map_info.width - scene_x
                scene_y = (real_y - self.map_info.origin.position.y) / self.map_info.resolution
                
                # overlay_transform이 있으면 변환 적용
                if self.overlay_transform and hasattr(self, 'original_scene_pos') and self.original_scene_pos:
                    # 기존 original_scene_pos를 사용하여 마커 위치 설정
                    scene_x = self.original_scene_pos.x()
                    scene_y = self.original_scene_pos.y()
                    print(f"마커 위치 업데이트: original_scene_pos 사용 ({scene_x:.1f}, {scene_y:.1f})")
                else:
                    # overlay_transform 적용 (맵 좌표 → 씬 좌표 변환)
                    if self.overlay_transform:
                        transform = QTransform()
                        center_x = scene_rect.width() / 2
                        center_y = scene_rect.height() / 2
                        
                        # 기본 씬 좌표 생성
                        base_scene_pos = QPointF(scene_x, scene_y)
                        
                        # 변환 행렬 생성
                        transform.translate(center_x + self.overlay_transform['translate_x'], 
                                           center_y + self.overlay_transform['translate_y'])
                        transform.rotate(self.overlay_transform['rotation'])
                        transform.scale(self.overlay_transform['scale'], 
                                       self.overlay_transform['scale'])
                        transform.translate(-center_x, -center_y)
                        
                        # 변환 적용
                        transformed_pos = transform.map(base_scene_pos)
                        scene_x = transformed_pos.x()
                        scene_y = transformed_pos.y()
                
                print(f"맵 정보 기반 마커 위치 계산:")
                print(f"  - 실제 좌표: ({real_x:.3f}, {real_y:.3f}) 미터")
                print(f"  - 씬 좌표: ({scene_x:.1f}, {scene_y:.1f}) 픽셀")
            else:
                # 맵 정보가 없는 경우 씬 크기 기반 좌표 변환
                # 씬의 중앙을 원점(0,0)으로 간주
                center_x = scene_rect.width() / 2
                center_y = scene_rect.height() / 2
                
                # 실제 좌표를 픽셀 좌표로 변환 (임의의 스케일 사용)
                scale = 100  # 1미터당 100픽셀
                scene_x = center_x + (real_x * scale)
                scene_y = center_y + (real_y * scale)
                
                print(f"맵 정보 없이 마커 위치 계산:")
                print(f"  - 실제 좌표: ({real_x:.3f}, {real_y:.3f}) 미터")
                print(f"  - 씬 좌표: ({scene_x:.1f}, {scene_y:.1f}) 픽셀")
            
            # 씬 범위 내로 좌표 제한
            scene_x = max(0, min(scene_x, scene_rect.width()))
            scene_y = max(0, min(scene_y, scene_rect.height()))
            
            # 씬 좌표에 마커 표시
            scene_pos = QPointF(scene_x, scene_y)
            self.show_click_marker(scene_pos, yaw)
            self.last_clicked_real_pos = (real_x, real_y)
            
            print(f"[MapManager] 마커 업데이트 성공: 실제좌표({real_x:.3f}, {real_y:.3f}) -> 씬좌표({scene_x:.1f}, {scene_y:.1f})")
            return True
            
        except Exception as e:
            print(f"마커 업데이트 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
            return False

    def is_ringo_click(self, scene_pos):
        """링고벨 클릭 여부 확인"""
        # TODO: 실제 링고벨 위치와 UUID 정보를 가져오는 로직 구현 필요
        # 현재는 임시로 False 반환
        return False

    def get_clicked_ringo_uuid(self):
        """클릭된 링고벨의 UUID 반환"""
        return self.clicked_ringo_uuid

    def update_ringo_positions(self, positions):
        """링고벨 위치 정보 업데이트"""
        self.ringo_positions = positions 

    # =========================================================================
    # 맵 컨트롤 메서드 (모델 로직)
    # =========================================================================
    def zoom_in(self):
        """맵 확대 (Model 로직)"""
        if self.map_view:
            # scale 메서드로 확대 (1.25배 = 25% 확대)
            self.map_view.scale(1.25, 1.25)
            self.map_zoomed_in.emit()
        
    def zoom_out(self):
        """맵 축소 (Model 로직)"""
        if self.map_view:
            # scale 메서드로 축소 (0.8배 = 20% 축소)
            self.map_view.scale(0.8, 0.8)
            self.map_zoomed_out.emit()
        
    def move_map(self, direction, pixels=50):
        """맵 이동 (Model 로직)"""
        if not self.map_view:
            return
            
        if direction == 'up':
            self.map_view.verticalScrollBar().setValue(
                self.map_view.verticalScrollBar().value() - pixels
            )
        elif direction == 'down':
            self.map_view.verticalScrollBar().setValue(
                self.map_view.verticalScrollBar().value() + pixels
            )
        elif direction == 'left':
            self.map_view.horizontalScrollBar().setValue(
                self.map_view.horizontalScrollBar().value() - pixels
            )
        elif direction == 'right':
            self.map_view.horizontalScrollBar().setValue(
                self.map_view.horizontalScrollBar().value() + pixels
            )
            
        self.map_moved.emit(direction)
    
    def move_robot(self, direction, step_size=0.1):
        """로봇 이동 (Model 로직)"""
        # 변환 전 원본 씬 좌표(original_scene_pos)가 있는지 확인
        if not hasattr(self, 'original_scene_pos') or self.original_scene_pos is None:
            print("마커 이동 실패: original_scene_pos가 없음")
            return False
        
        # 원본 씬 좌표 가져오기
        scene_x, scene_y = self.original_scene_pos.x(), self.original_scene_pos.y()
        print(f"마커 이동 - 변환 전 씬 좌표: ({scene_x}, {scene_y}), 방향: {direction}")
        
        # 방향에 따라 씬 좌표 직접 수정 (픽셀 단위로 이동)
        pixel_step = step_size / self.map_info.resolution if self.map_info else 10
        
        # 이동할 새 좌표 계산
        new_scene_x, new_scene_y = scene_x, scene_y
        if direction == 'up':
            new_scene_y -= pixel_step
        elif direction == 'down':
            new_scene_y += pixel_step
        elif direction == 'left':
            new_scene_x -= pixel_step
        elif direction == 'right':
            new_scene_x += pixel_step
            
        # 맵 이미지 경계 확인
        if self.map_overlay_pixmap:
            # 맵 이미지의 경계 계산
            map_width = self.map_overlay_pixmap.width()
            map_height = self.map_overlay_pixmap.height()
            
            # 경계를 벗어나는지 확인
            if new_scene_x < 0 or new_scene_x > map_width or new_scene_y < 0 or new_scene_y > map_height:
                print(f"마커 이동 실패: 맵 이미지 경계 벗어남 ({new_scene_x}, {new_scene_y}), 맵 크기: {map_width}x{map_height}")
                return False
            
        # 금지구역 체크 - 자체 구현한 함수 사용
        is_forbidden_area, touched_area_index, touched_button = self.check_forbidden_area(QPointF(new_scene_x, new_scene_y))
        if is_forbidden_area:
            print(f"마커 이동 시 금지 구역 {touched_area_index+1} 감지됨 ({new_scene_x}, {new_scene_y})")
            
            # 금지구역 우회 기능 구현
            if touched_button:
                # 금지구역의 위치와 크기 정보 가져오기
                area_rect = touched_button.geometry()
                area_x = area_rect.x()
                area_y = area_rect.y()
                area_width = area_rect.width()
                area_height = area_rect.height()
                
                # 우회 경로 계산
                bypass_steps = []
                
                if direction in ['left', 'right']:
                    # 좌우 이동 시 우회 방향 결정 (위로 갈지 아래로 갈지)
                    dist_to_top = abs(scene_y - area_y)
                    dist_to_bottom = abs(scene_y - (area_y + area_height))
                    
                    bypass_direction = 'up' if dist_to_top <= dist_to_bottom else 'down'
                    print(f"금지구역 우회: {direction} 이동 시도 중 금지구역 감지, {bypass_direction}으로 우회")
                    
                    if bypass_direction == 'up':
                        # 위로 우회: 위로 이동 -> 옆으로 이동 -> 아래로 이동
                        up_dist = pixel_step + (scene_y - area_y)
                        side_dist = area_width + 2 * pixel_step
                        down_dist = up_dist
                        
                        # 위로 이동
                        bypass_scene_y = scene_y - up_dist
                        bypass_steps.append(('up', QPointF(scene_x, bypass_scene_y)))
                        
                        # 옆으로 이동
                        if direction == 'right':
                            bypass_scene_x = scene_x + side_dist
                        else:  # 'left'
                            bypass_scene_x = scene_x - side_dist
                        bypass_steps.append((direction, QPointF(bypass_scene_x, bypass_scene_y)))
                        
                        # 아래로 이동
                        bypass_scene_y = bypass_scene_y + down_dist
                        bypass_steps.append(('down', QPointF(bypass_scene_x, bypass_scene_y)))
                        
                    else:  # 'down'
                        # 아래로 우회: 아래로 이동 -> 옆으로 이동 -> 위로 이동
                        down_dist = pixel_step + ((area_y + area_height) - scene_y)
                        side_dist = area_width + 2 * pixel_step
                        up_dist = down_dist
                        
                        # 아래로 이동
                        bypass_scene_y = scene_y + down_dist
                        bypass_steps.append(('down', QPointF(scene_x, bypass_scene_y)))
                        
                        # 옆으로 이동
                        if direction == 'right':
                            bypass_scene_x = scene_x + side_dist
                        else:  # 'left'
                            bypass_scene_x = scene_x - side_dist
                        bypass_steps.append((direction, QPointF(bypass_scene_x, bypass_scene_y)))
                        
                        # 위로 이동
                        bypass_scene_y = bypass_scene_y - up_dist
                        bypass_steps.append(('up', QPointF(bypass_scene_x, bypass_scene_y)))
                
                elif direction in ['up', 'down']:
                    # 상하 이동 시 우회 방향 결정 (왼쪽으로 갈지 오른쪽으로 갈지)
                    dist_to_left = abs(scene_x - area_x)
                    dist_to_right = abs(scene_x - (area_x + area_width))
                    
                    bypass_direction = 'left' if dist_to_left <= dist_to_right else 'right'
                    print(f"금지구역 우회: {direction} 이동 시도 중 금지구역 감지, {bypass_direction}으로 우회")
                    
                    if bypass_direction == 'left':
                        # 왼쪽으로 우회: 왼쪽 이동 -> 상하 이동 -> 오른쪽 이동
                        left_dist = pixel_step + (scene_x - area_x)
                        vert_dist = area_height + 2 * pixel_step
                        right_dist = left_dist
                        
                        # 왼쪽으로 이동
                        bypass_scene_x = scene_x - left_dist
                        bypass_steps.append(('left', QPointF(bypass_scene_x, scene_y)))
                        
                        # 상하 이동
                        if direction == 'up':
                            bypass_scene_y = scene_y - vert_dist
                        else:  # 'down'
                            bypass_scene_y = scene_y + vert_dist
                        bypass_steps.append((direction, QPointF(bypass_scene_x, bypass_scene_y)))
                        
                        # 오른쪽으로 이동
                        bypass_scene_x = bypass_scene_x + right_dist
                        bypass_steps.append(('right', QPointF(bypass_scene_x, bypass_scene_y)))
                        
                    else:  # 'right'
                        # 오른쪽으로 우회: 오른쪽 이동 -> 상하 이동 -> 왼쪽 이동
                        right_dist = pixel_step + ((area_x + area_width) - scene_x)
                        vert_dist = area_height + 2 * pixel_step
                        left_dist = right_dist
                        
                        # 오른쪽으로 이동
                        bypass_scene_x = scene_x + right_dist
                        bypass_steps.append(('right', QPointF(bypass_scene_x, scene_y)))
                        
                        # 상하 이동
                        if direction == 'up':
                            bypass_scene_y = scene_y - vert_dist
                        else:  # 'down'
                            bypass_scene_y = scene_y + vert_dist
                        bypass_steps.append((direction, QPointF(bypass_scene_x, bypass_scene_y)))
                        
                        # 왼쪽으로 이동
                        bypass_scene_x = bypass_scene_x - left_dist
                        bypass_steps.append(('left', QPointF(bypass_scene_x, bypass_scene_y)))
                
                # 우회 경로 실행
                for step_dir, step_pos in bypass_steps:
                    # 각 단계마다 금지구역 체크
                    step_forbidden, _, _ = self.check_forbidden_area(step_pos)
                    if step_forbidden:
                        print(f"우회 경로에서 다른 금지구역 감지됨: {step_dir} ({step_pos.x()}, {step_pos.y()})")
                        continue
                    
                    # 마커 위치 업데이트
                    self.original_scene_pos = step_pos
                    print(f"우회 이동: {step_dir} - 마커 위치: ({step_pos.x()}, {step_pos.y()})")
                    
                    # 마커 업데이트 (변환 전 원본 씬 좌표 사용)
                    self.show_click_marker(self.original_scene_pos, self.last_yaw)
                    
                    # 시그널 발생
                    self.robot_moved.emit(step_dir)
                    
                    # 약간의 지연 효과 (UI 업데이트를 위해)
                    QApplication.processEvents()
                    QThread.msleep(100)
                
                # 우회 완료 후 실제 좌표 계산 및 저장
                self._update_real_coordinates()
                return True
            else:
                print(f"마커 이동 실패: 금지 구역 {touched_area_index+1}에 의해 차단됨")
                return False
        
        # 금지구역이 아니면 새 씬 좌표 저장
        self.original_scene_pos = QPointF(new_scene_x, new_scene_y)
        print(f"마커 이동 후 씬 좌표: ({new_scene_x}, {new_scene_y})")
        
        # 마커 업데이트 (변환 전 원본 씬 좌표 사용)
        self.show_click_marker(self.original_scene_pos, self.last_yaw)
        
        # 새 씬 좌표로 실제 좌표 계산
        self._update_real_coordinates()
        
        
        # 시그널 발생
        self.robot_moved.emit(direction)
        
        return True
        
    def rotate_robot(self, direction, step_size=0.1):
        """로봇 회전 (Model 로직)"""
        # original_scene_pos가 없으면 last_clicked_pos를 사용
        if not hasattr(self, 'original_scene_pos') or self.original_scene_pos is None:
            if hasattr(self, 'last_clicked_pos') and self.last_clicked_pos:
                # last_clicked_pos를 original_scene_pos로 설정
                self.original_scene_pos = QPointF(self.last_clicked_pos.x(), self.last_clicked_pos.y())
                print(f"회전 시 original_scene_pos 초기화: ({self.original_scene_pos.x()}, {self.original_scene_pos.y()})")
            else:
                print("마커 회전 실패: 위치 정보 없음")
                return False
            
        if direction == 'right':
            # 시계 방향 회전 - 양수 방향으로 회전
            self.last_yaw += step_size
            # 2π 범위 내로 유지
            self.last_yaw = self.last_yaw % (2 * math.pi)
        elif direction == 'left':
            # 반시계 방향 회전 - 음수 방향으로 회전
            self.last_yaw -= step_size
            # 2π 범위 내로 유지 (음수 값 처리)
            self.last_yaw = self.last_yaw % (2 * math.pi)
            
        # 마커 업데이트 (변환 전 원본 씬 좌표 사용)
        self.show_click_marker(self.original_scene_pos, self.last_yaw)
        
        # 실제 좌표 업데이트 (회전 시에도 좌표 정보 유지)
        self._update_real_coordinates()
        
        # 시그널 발생
        self.robot_moved.emit(f"rotate_{direction}")
        
        return True

    def load_overlay_transform(self):
        """맵 오버레이 변환 정보 불러오기"""
        try:
            transform_file = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                        "data", "map_transform.json")
            if os.path.exists(transform_file):
                import json
                with open(transform_file, 'r', encoding='utf-8') as f:
                    transform_data = json.load(f)
                self.overlay_transform.update(transform_data)
                print(f"맵 오버레이 변환 정보 로드됨: {self.overlay_transform}")
        except Exception as e:
            print(f"맵 오버레이 변환 정보 로드 중 오류 발생: {str(e)}")
    
    def set_overlay_transform(self, transform_data):
        """맵 오버레이 변환 정보 설정"""
        if transform_data:
            self.overlay_transform.update(transform_data)
            print(f"맵 오버레이 변환 정보 업데이트됨: {self.overlay_transform}")
            
            # 변환 정보가 업데이트되면 맵 즉시 업데이트
            if self.map_subscriber and self.map_subscriber.last_map_msg:
                self.update_map(self.map_subscriber.last_map_msg)
                print("맵 변환 정보 업데이트로 인한 맵 리로드")

    def save_map_as_png(self, file_path=None):
        """현재 맵 데이터를 PNG 이미지로 저장
        
        Args:
            file_path (str, optional): 저장할 경로. 기본값은 resources/map_cap.png
            
        Returns:
            bool: 저장 성공 여부
        """
        try:
            # 저장 경로가 지정되지 않은 경우 기본 경로 사용
            if file_path is None:
                file_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                        "resources", "map_cap.png")
            
            # 현재 맵 씬에서 이미지 생성
            if self.map_scene is None or self.map_scene.sceneRect().isEmpty():
                print("맵 씬이 비어있어 저장할 수 없습니다.")
                return False
            
            # 맵 씬의 크기 가져오기
            rect = self.map_scene.sceneRect()
            width = int(rect.width())
            height = int(rect.height())
            
            # 맵만 렌더링하기 위해 오버레이 항목 임시 숨김
            overlay_items = []
            for item in self.map_scene.items():
                if item.zValue() > -2:  # 맵(-2)보다 위에 있는 항목들
                    overlay_items.append((item, item.isVisible()))
                    item.setVisible(False)
            
            # 맵만 포함된 이미지 생성
            image = QImage(width, height, QImage.Format_ARGB32)
            image.fill(Qt.transparent)
            
            painter = QPainter(image)
            painter.setRenderHint(QPainter.Antialiasing)
            self.map_scene.render(painter, QRectF(0, 0, width, height), rect)
            painter.end()
            
            # 원래대로 가시성 복원
            for item, was_visible in overlay_items:
                item.setVisible(was_visible)
            
            # 이미지 저장
            image.save(file_path)
            
            print(f"맵 이미지가 저장되었습니다: {file_path}")
            print(f"맵 크기: {width}x{height}, 해상도: {self.map_info.resolution if self.map_info else 'N/A'}")
            return True
        
        except Exception as e:
            print(f"맵 저장 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
            return False 

    @Slot(bool)
    def on_area_setting_changed(self, is_area_setting):
        """영역 설정 모드 상태 변경 시 호출되는 슬롯"""
        self.is_area_setting_screen = is_area_setting
        print(f"[MapManager] 영역 설정 모드 상태 변경: {self.is_area_setting_screen}")
        # 여기에 영역 설정 모드에 따른 추가 처리 로직을 구현할 수 있습니다.
        
    def update_forbidden_areas(self, area_buttons):
        """
        금지구역 정보 업데이트
        
        Parameters:
            area_buttons (list): 금지구역 버튼 목록
        """
        self.area_buttons = area_buttons
        print(f"[MapManager] 금지구역 정보 업데이트: {len(self.area_buttons)}개")
    
    def _update_real_coordinates(self):
        """
        현재 씬 좌표(original_scene_pos)를 실제 맵 좌표로 변환하여 저장
        """
        if not hasattr(self, 'original_scene_pos') or self.original_scene_pos is None:
            print("실제 좌표 업데이트 실패: original_scene_pos가 없음")
            return
            
        scene_x, scene_y = self.original_scene_pos.x(), self.original_scene_pos.y()
        
        # 맵 정보가 있는 경우에만 변환 수행
        if self.map_info:
            # 좌표 변환 로직
            # 씬 좌표를 실제 좌표로 변환 (overlay_transform 고려)
            if self.overlay_transform:
                # 변환 적용 (map_transform.json에 저장된 변환 정보 사용)
                transform = QTransform()
                scene_rect = self.map_scene.sceneRect()
                center_x = scene_rect.width() / 2
                center_y = scene_rect.height() / 2
                
                transform.translate(center_x + self.overlay_transform['translate_x'], 
                                   center_y + self.overlay_transform['translate_y'])
                transform.rotate(self.overlay_transform['rotation'])
                transform.scale(self.overlay_transform['scale'], 
                               self.overlay_transform['scale'])
                transform.translate(-center_x, -center_y)
                
                inverse_transform = transform.inverted()[0]
                original_pos = inverse_transform.map(QPointF(scene_x, scene_y))
                
                # 맵 좌표 계산
                adjusted_scene_x = scene_rect.width() - original_pos.x()
                real_x = adjusted_scene_x * self.map_info.resolution + self.map_info.origin.position.x
                real_y = original_pos.y() * self.map_info.resolution + self.map_info.origin.position.y
            else:
                # overlay_transform 없는 경우 직접 변환
                adjusted_scene_x = self.map_scene.width() - scene_x
                real_x = adjusted_scene_x * self.map_info.resolution + self.map_info.origin.position.x
                real_y = scene_y * self.map_info.resolution + self.map_info.origin.position.y
            
            # 실제 좌표 저장 (튜플 형태로 통일)
            self.last_clicked_real_pos = (real_x, real_y)
            print(f"변환된 실제 좌표: ({real_x}, {real_y})")
    
    def check_forbidden_area(self, scene_pos):
        """
        클릭 위치가 금지구역 내부인지 확인하는 함수
        
        Parameters:
            scene_pos (QPointF): 확인할 씬 좌표
            
        Returns:
            tuple: (is_forbidden_area, touched_area_index, touched_button)
        """
        is_forbidden_area = False
        touched_area_index = -1
        touched_button = None
        
        scene_x, scene_y = scene_pos.x(), scene_pos.y()
        
        # 내부 저장된 금지구역 정보 사용
        if hasattr(self, 'area_buttons') and self.area_buttons:
            # 버튼 확인
            for i, button in enumerate(self.area_buttons):
                if button is None:
                    continue
                    
                try:
                    # 버튼의 현재 geometry를 사용하여 클릭 위치가 버튼 내부인지 확인
                    if hasattr(button, 'geometry'):
                        # 우선 경로 버튼은 금지구역으로 간주하지 않음
                        area_type = button.property("area_type")
                        if area_type == "우선 경로":
                            continue
                            
                        btn_rect = button.geometry()
                        # 클릭 위치가 버튼 영역 내에 있는지 확인
                        if btn_rect.contains(int(scene_x), int(scene_y)):
                            is_forbidden_area = True
                            touched_area_index = i
                            touched_button = button
                            print(f"[MapManager] 금지 구역 {i+1} 내부 감지! 좌표: ({scene_x}, {scene_y})")
                            break
                except Exception as e:
                    print(f"[MapManager] 버튼 {i} 확인 중 오류: {e}")
        
        # 부모 뷰에서도 금지구역 정보 확인 (area_buttons가 비어있을 경우)
        elif not is_forbidden_area:
            parent_view = self.parent()
            if parent_view and hasattr(parent_view, 'area_buttons') and parent_view.area_buttons:
                # 부모 뷰의 금지구역 정보 저장
                self.area_buttons = parent_view.area_buttons
                
                # 버튼 확인
                for i, button in enumerate(parent_view.area_buttons):
                    if button is None:
                        continue
                        
                    try:
                        # 버튼의 현재 geometry를 사용하여 클릭 위치가 버튼 내부인지 확인
                        if hasattr(button, 'geometry'):
                            # 우선 경로 버튼은 금지구역으로 간주하지 않음
                            area_type = button.property("area_type")
                            if area_type == "우선 경로":
                                continue
                                
                            btn_rect = button.geometry()
                            # 클릭 위치가 버튼 영역 내에 있는지 확인
                            if btn_rect.contains(int(scene_x), int(scene_y)):
                                is_forbidden_area = True
                                touched_area_index = i
                                touched_button = button
                                print(f"[MapManager] 금지 구역 {i+1} 내부 감지! 좌표: ({scene_x}, {scene_y})")
                                break
                    except Exception as e:
                        print(f"[MapManager] 버튼 {i} 확인 중 오류: {e}")
        
        return is_forbidden_area, touched_area_index, touched_button