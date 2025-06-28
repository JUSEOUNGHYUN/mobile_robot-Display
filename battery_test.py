#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import traceback
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider, QPushButton
from PySide6.QtCore import Qt, QTimer, QRect
from PySide6.QtGui import QPixmap, QPainter, QColor, QFont, QPen

# 직접 BatteryRenderer 클래스 구현 (모듈 임포트 문제 해결)
class BatteryRenderer:
    """배터리 잔량을 시각적으로 표시하는 클래스"""
    
    def __init__(self):
        """초기화"""
        print("BatteryRenderer 초기화 중...")
        self.base_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', 'Battery')
        print(f"배터리 이미지 경로: {self.base_path}")
        self.battery_image = None
        self.load_base_image()
    
    def load_base_image(self):
        """기본 배터리 이미지 로드"""
        try:
            # 11~19% 이미지를 기본 템플릿으로 사용
            image_path = os.path.join(self.base_path, 'battery_11to19.png')
            print(f"이미지 로드 시도: {image_path}")
            
            # 파일 존재 여부 확인
            if not os.path.exists(image_path):
                print(f"오류: 이미지 파일이 존재하지 않음: {image_path}")
                self.battery_image = QPixmap(100, 50)
                self.battery_image.fill(Qt.transparent)
                return
                
            self.battery_image = QPixmap(image_path)
            if self.battery_image.isNull():
                print(f"배터리 이미지 로드 실패: {image_path}")
                # 이미지 로드 실패 시 빈 픽스맵 생성
                self.battery_image = QPixmap(100, 50)
                self.battery_image.fill(Qt.transparent)
            else:
                print(f"이미지 로드 성공: {self.battery_image.width()}x{self.battery_image.height()}")
        except Exception as e:
            print(f"배터리 이미지 로드 중 오류 발생: {str(e)}")
            traceback.print_exc()
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
        try:
            if not self.battery_image:
                print("배터리 이미지가 로드되지 않음, 다시 로드 시도")
                self.load_base_image()
            
            if self.battery_image.isNull():
                print("배터리 이미지가 여전히 null, 기본 이미지 생성")
                result = QPixmap(100, 50)
                result.fill(Qt.transparent)
                return result
            
            # 퍼센트 범위 제한
            percentage = max(0, min(100, percentage))
            
            # 이미지 복사
            result = QPixmap(self.battery_image)
            print(f"렌더링 시작: {result.width()}x{result.height()}, 배터리 잔량: {percentage}%")
            
            # 페인터 생성
            painter = QPainter(result)
            painter.setRenderHint(QPainter.Antialiasing)
            
            # 배터리 내부 영역 계산 (이미지에 맞게 조정 필요)
            inner_x = 4  # 배터리 내부 시작 x 좌표
            inner_y = 4  # 배터리 내부 시작 y 좌표
            inner_width = result.width() - 8  # 배터리 내부 너비
            inner_height = result.height() - 8  # 배터리 내부 높이
            
            print(f"배터리 내부 영역: x={inner_x}, y={inner_y}, width={inner_width}, height={inner_height}")
            
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
            
            print(f"채우기 영역: width={fill_width}, color={fill_color.name()}")
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
            print("렌더링 완료")
            return result
        except Exception as e:
            print(f"배터리 렌더링 중 오류 발생: {str(e)}")
            traceback.print_exc()
            # 오류 발생 시 기본 이미지 반환
            result = QPixmap(100, 50)
            result.fill(Qt.transparent)
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


class BatteryTestApp(QWidget):
    """배터리 렌더러 테스트 애플리케이션"""
    
    def __init__(self):
        super().__init__()
        
        print("BatteryTestApp 초기화 중...")
        self.battery_renderer = BatteryRenderer()
        self.current_percentage = 50
        self.auto_mode = False
        self.auto_direction = 1  # 1: 증가, -1: 감소
        
        self.setup_ui()
        
    def setup_ui(self):
        """UI 초기화"""
        print("UI 초기화 중...")
        self.setWindowTitle("배터리 렌더러 테스트")
        self.setGeometry(100, 100, 400, 300)
        
        # 레이아웃 설정
        layout = QVBoxLayout(self)
        
        # 배터리 이미지 레이블
        self.battery_label = QLabel()
        self.battery_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.battery_label)
        
        # 퍼센트 표시 레이블
        self.percentage_label = QLabel(f"배터리 잔량: {self.current_percentage}%")
        self.percentage_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.percentage_label)
        
        # 슬라이더
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 100)
        self.slider.setValue(self.current_percentage)
        self.slider.valueChanged.connect(self.on_slider_changed)
        layout.addWidget(self.slider)
        
        # 자동 모드 버튼
        self.auto_button = QPushButton("자동 모드 시작")
        self.auto_button.clicked.connect(self.toggle_auto_mode)
        layout.addWidget(self.auto_button)
        
        # 타이머 설정
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_auto_percentage)
        
        # 초기 배터리 이미지 업데이트
        self.update_battery_image()
        print("UI 초기화 완료")
        
    def on_slider_changed(self, value):
        """슬라이더 값 변경 이벤트 핸들러"""
        self.current_percentage = value
        self.update_battery_image()
        
    def update_battery_image(self):
        """배터리 이미지 업데이트"""
        try:
            # 배터리 이미지 생성
            pixmap = self.battery_renderer.get_battery_pixmap(self.current_percentage)
            
            # 이미지 설정
            if not pixmap.isNull():
                self.battery_label.setPixmap(pixmap.scaled(200, 100, Qt.KeepAspectRatio))
            else:
                print("경고: 배터리 이미지가 null입니다.")
                
            # 퍼센트 텍스트 업데이트
            self.percentage_label.setText(f"배터리 잔량: {self.current_percentage}%")
        except Exception as e:
            print(f"배터리 이미지 업데이트 중 오류 발생: {str(e)}")
            traceback.print_exc()
        
    def toggle_auto_mode(self):
        """자동 모드 토글"""
        try:
            self.auto_mode = not self.auto_mode
            
            if self.auto_mode:
                self.auto_button.setText("자동 모드 중지")
                self.timer.start(100)  # 100ms 간격으로 업데이트
                self.slider.setEnabled(False)
            else:
                self.auto_button.setText("자동 모드 시작")
                self.timer.stop()
                self.slider.setEnabled(True)
        except Exception as e:
            print(f"자동 모드 토글 중 오류 발생: {str(e)}")
            traceback.print_exc()
            
    def update_auto_percentage(self):
        """자동 모드에서 배터리 퍼센트 업데이트"""
        try:
            # 방향 전환 체크
            if self.current_percentage >= 100:
                self.auto_direction = -1
            elif self.current_percentage <= 0:
                self.auto_direction = 1
                
            # 퍼센트 업데이트
            self.current_percentage += self.auto_direction
            
            # 슬라이더 및 이미지 업데이트
            self.slider.setValue(self.current_percentage)
            self.update_battery_image()
        except Exception as e:
            print(f"자동 퍼센트 업데이트 중 오류 발생: {str(e)}")
            traceback.print_exc()

if __name__ == "__main__":
    try:
        print("배터리 테스트 애플리케이션 시작")
        app = QApplication(sys.argv)
        window = BatteryTestApp()
        window.show()
        print("애플리케이션 실행 중...")
        sys.exit(app.exec())
    except Exception as e:
        print(f"애플리케이션 실행 중 오류 발생: {str(e)}")
        traceback.print_exc() 