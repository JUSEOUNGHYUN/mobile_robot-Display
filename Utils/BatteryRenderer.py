#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from PySide6.QtCore import Qt, QRect
from PySide6.QtGui import QPixmap, QPainter, QColor, QFont, QPen
import os

class BatteryRenderer:
    """배터리 잔량을 시각적으로 표시하는 클래스"""
    
    def __init__(self):
        """초기화"""
        self.base_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'resources', 'Battery')
        self.battery_image = None
        self.load_base_image()
    
    def load_base_image(self):
        """기본 배터리 이미지 로드"""
        try:
            # 11~19% 이미지를 기본 템플릿으로 사용
            image_path = os.path.join(self.base_path, 'battery_11to19.png')
            self.battery_image = QPixmap(image_path)
            if self.battery_image.isNull():
                print(f"배터리 이미지 로드 실패: {image_path}")
                # 이미지 로드 실패 시 빈 픽스맵 생성
                self.battery_image = QPixmap(100, 50)
                self.battery_image.fill(Qt.transparent)
        except Exception as e:
            print(f"배터리 이미지 로드 중 오류 발생: {str(e)}")
            self.battery_image = QPixmap(100, 50)
            self.battery_image.fill(Qt.transparent)
    
    def render_battery(self, percentage):
        """
        배터리 잔량에 따라 이미지 렌더링
        
        Args:
            percentage (int): 배터리 잔량 (0-100)
            
        Returns:
            QPixmap: 렌더링된 배터리 이미지
        """
        if not self.battery_image:
            self.load_base_image()
        
        if self.battery_image.isNull():
            return QPixmap()
        
        # 퍼센트 범위 제한
        percentage = max(0, min(100, percentage))
        
        # 이미지 복사
        result = QPixmap(self.battery_image)
        
        # 페인터 생성
        painter = QPainter(result)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 배터리 내부 영역 계산 (이미지에 맞게 조정 필요)
        # 배터리 이미지의 내부 영역 좌표 (x, y, width, height)
        # 이 값들은 실제 이미지에 맞게 조정해야 함
        inner_x = 4  # 배터리 내부 시작 x 좌표
        inner_y = 4  # 배터리 내부 시작 y 좌표
        inner_width = result.width() - 8  # 배터리 내부 너비
        inner_height = result.height() - 8  # 배터리 내부 높이
        
        # 배터리 잔량에 따른 채우기 너비 계산
        fill_width = int(inner_width * percentage / 100)
        
        # 배터리 잔량 영역 채우기
        if percentage > 20:
            # 20% 이상이면 초록색
            fill_color = QColor(0, 200, 0)  # 초록색
        elif percentage > 10:
            # 10~20%면 노란색
            fill_color = QColor(255, 200, 0)  # 노란색
        else:
            # 10% 이하면 빨간색
            fill_color = QColor(255, 0, 0)  # 빨간색
        
        painter.fillRect(inner_x, inner_y, fill_width, inner_height, fill_color)
        
        # 배터리 잔량 텍스트 추가
        font = QFont("Arial", 12, QFont.Bold)
        painter.setFont(font)
        
        # 텍스트 그림자 효과 (선택사항)
        painter.setPen(QPen(QColor(0, 0, 0, 100)))
        painter.drawText(QRect(1, 1, result.width(), result.height()), 
                        Qt.AlignCenter, f"{percentage}%")
        
        # 텍스트 그리기
        painter.setPen(QPen(Qt.white))
        painter.drawText(QRect(0, 0, result.width(), result.height()), 
                        Qt.AlignCenter, f"{percentage}%")
        
        painter.end()
        return result
    
    def get_battery_pixmap(self, percentage):
        """
        배터리 잔량에 따른 이미지 반환
        
        Args:
            percentage (int): 배터리 잔량 (0-100)
            
        Returns:
            QPixmap: 배터리 이미지
        """
        return self.render_battery(percentage)


# 사용 예시:
if __name__ == "__main__":
    from PySide6.QtWidgets import QApplication, QLabel
    import sys
    
    app = QApplication(sys.argv)
    
    battery_renderer = BatteryRenderer()
    
    # 다양한 배터리 잔량으로 테스트
    percentages = [5, 15, 30, 50, 75, 100]
    
    for i, percentage in enumerate(percentages):
        label = QLabel()
        label.setPixmap(battery_renderer.get_battery_pixmap(percentage))
        label.setWindowTitle(f"배터리 {percentage}%")
        label.setGeometry(100 + i * 120, 100, 100, 50)
        label.show()
    
    sys.exit(app.exec()) 