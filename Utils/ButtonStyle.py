from PySide6.QtWidgets import QPushButton, QGraphicsDropShadowEffect, QWidget, QLabel
from PySide6.QtGui import QColor, QFont, QIcon
from PySide6.QtCore import QSize

class RoundButtonStyle:
    """둥근 모서리와 그림자 효과가 있는 버튼 스타일 클래스
    
    다양한 화면에서 재사용할 수 있는 버튼 스타일을 제공합니다.
    """
    
    @staticmethod
    def apply_round_style(button, width=230, height=130, font_size=40, border_radius=30):
        """버튼에 둥근 모서리와 그림자 효과를 적용합니다.
        
        Args:
            button (QPushButton): 스타일을 적용할 버튼
            width (int): 버튼 너비
            height (int): 버튼 높이
            font_size (int): 폰트 크기
            border_radius (int): 모서리 둥글기 정도
        """
        # 버튼 크기 설정
        button.setFixedSize(width, height)
        
        # 스타일시트 적용
        button.setStyleSheet(f"""
            QPushButton {{
                background-color: #FFFFFF;
                border: none;
                border-radius: {border_radius}px;
                color: black;
                font-family: "Noto Sans KR Black";
                font-size: {font_size}px;
                font-weight: 500;
            }}
            QPushButton:hover {{
                background-color: #E0E0E0;
            }}
        """)
        
        # 그림자 효과 추가
        shadow = QGraphicsDropShadowEffect(button)
        shadow.setBlurRadius(15)
        shadow.setColor(QColor(0, 0, 0, 80))
        shadow.setOffset(3, 3)
        button.setGraphicsEffect(shadow)
    
    @staticmethod
    def apply_icon_transfer_button_style(button, width=80, height=80, icon_path=None, icon_size=60, border_radius=None, bg_color=(255, 255, 255), bg_opacity=180):
        """아이콘만 있는 둥근 버튼 스타일을 적용합니다.
        
        Args:
            button (QPushButton): 스타일을 적용할 버튼
            width (int): 버튼 너비
            height (int): 버튼 높이
            icon_path (str): 아이콘 경로
            icon_size (int): 아이콘 크기
            border_radius (int): 모서리 둥글기 정도 (None이면 자동으로 원형 계산)
            bg_color (tuple): 배경색 RGB 값 (r, g, b)
            bg_opacity (int): 배경색 투명도 (0-255)
        """
        # 버튼 크기 설정
        button.setFixedSize(width, height)
        
        # 기본값으로 원형 버튼 (반지름은 너비의 절반)
        if border_radius is None:
            border_radius = min(width, height) // 2
        
        # RGB 값 추출
        r, g, b = bg_color
            
        # 스타일시트 적용
        button.setStyleSheet(f"""
            QPushButton {{
                background-color: rgba({r}, {g}, {b}, {bg_opacity}); /* RGB와 투명도 파라미터 적용 */
                border: 1px solid #444444;
                border_radius: {border_radius}px;
                padding: 5px;
            }}
            QPushButton:hover {{
                background-color: #FFFFFF;
            }}
            QPushButton:pressed {{
                background-color: rgba(70, 130, 180, 180); /* 클릭 시 배경색 변경 */
            }}
        """)
        
        # 아이콘 설정
        if icon_path:
            button.setIcon(QIcon(icon_path))
            button.setIconSize(QSize(icon_size, icon_size))
        
        # 그림자 효과 추가
        shadow = QGraphicsDropShadowEffect(button)
        shadow.setBlurRadius(15)
        shadow.setColor(QColor(0, 0, 0, 80))
        shadow.setOffset(3, 3)
        button.setGraphicsEffect(shadow)
    
    @staticmethod
    def apply_icon_button_style(button, width=80, height=80, icon_path=None, icon_size=60, border_radius=None):
        """아이콘만 있는 둥근 버튼 스타일을 적용합니다.
        
        Args:
            button (QPushButton): 스타일을 적용할 버튼
            width (int): 버튼 너비
            height (int): 버튼 높이
            icon_path (str): 아이콘 경로
            icon_size (int): 아이콘 크기
            border_radius (int): 모서리 둥글기 정도 (None이면 자동으로 원형 계산)
        """
        # 버튼 크기 설정
        button.setFixedSize(width, height)
        
        # 기본값으로 원형 버튼 (반지름은 너비의 절반)
        if border_radius is None:
            border_radius = min(width, height) // 2
            
        # 스타일시트 적용
        button.setStyleSheet(f"""
            QPushButton {{
                background-color: #FFFFFF;
                border: none;
                border-radius: {border_radius}px;
                padding: 5px;
            }}
            QPushButton:hover {{
                background-color: #E0E0E0;
            }}
        """)
        
        # 아이콘 설정
        if icon_path:
            button.setIcon(QIcon(icon_path))
            button.setIconSize(QSize(icon_size, icon_size))
        
        # 그림자 효과 추가
        shadow = QGraphicsDropShadowEffect(button)
        shadow.setBlurRadius(15)
        shadow.setColor(QColor(0, 0, 0, 80))
        shadow.setOffset(3, 3)
        button.setGraphicsEffect(shadow)
    
    @staticmethod
    def apply_shadow_to_widget(widget, blur_radius=15, color_opacity=80, offset_x=3, offset_y=3):
        """위젯에 그림자 효과를 적용합니다.
        
        Args:
            widget (QWidget): 그림자를 적용할 위젯
            blur_radius (int): 그림자 블러 반경
            color_opacity (int): 그림자 투명도 (0-255)
            offset_x (int): 그림자 X축 오프셋
            offset_y (int): 그림자 Y축 오프셋
        """
        shadow = QGraphicsDropShadowEffect(widget)
        shadow.setBlurRadius(blur_radius)
        
        # 투명도가 튜플이나 리스트인 경우 (r, g, b, a) 형식으로 전달된 경우
        if isinstance(color_opacity, (tuple, list)):
            if len(color_opacity) == 4:
                r, g, b, a = color_opacity
                shadow.setColor(QColor(r, g, b, a))
            elif len(color_opacity) == 3:
                r, g, b = color_opacity
                shadow.setColor(QColor(r, g, b, 80))  # 기본 투명도 80
            else:
                # 잘못된 형식이면 기본값 사용
                shadow.setColor(QColor(0, 0, 0, 80))
        else:
            # 단일 정수값인 경우 (투명도만 지정)
            shadow.setColor(QColor(0, 0, 0, color_opacity))
            
        shadow.setOffset(offset_x, offset_y)
        widget.setGraphicsEffect(shadow)
    
    @staticmethod
    def apply_header_style(widget, background_color="#F0F0F0", text_color="#000000", border_radius=10):
        """헤더 스타일을 적용합니다 (이미지에 보이는 것과 같은 헤더).
        
        Args:
            widget (QWidget): 스타일을 적용할 위젯 (QWidget, QFrame 등)
            background_color (str): 배경색 (HEX 코드)
            text_color (str): 텍스트 색상 (HEX 코드)
            border_radius (int): 모서리 둥글기 정도
        """
        # 스타일시트 적용
        widget.setStyleSheet(f"""
            QWidget {{
                background-color: {background_color};
                border: none;
                border-radius: {border_radius}px;
                color: {text_color};
            }}
        """)
        
        # 그림자 효과 추가
        RoundButtonStyle.apply_shadow_to_widget(widget)
    
    @staticmethod
    def get_selected_style(border_radius=30):
        """선택된 버튼의 스타일을 반환합니다.
        
        Args:
            border_radius (int): 모서리 둥글기 정도
            
        Returns:
            str: 선택된 버튼의 스타일시트
        """
        return f"""
        QPushButton {{
            background-color: #015714;
            color: white;
            border-radius: {border_radius}px;
        }}
        """
    
    @staticmethod
    def apply_font_by_size(button, width):
        """버튼 크기에 따라 적절한 폰트 크기를 설정합니다.
        
        Args:
            button (QPushButton): 폰트를 설정할 버튼
            width (int): 버튼 너비
        
        Returns:
            int: 설정된 폰트 크기
        """
        if width >= 350:  # 큰 버튼
            font_size = 120
            button.setFont(QFont('Noto Sans KR Black', 32))
        elif width >= 300:  # 중간 큰 버튼
            font_size = 100
            button.setFont(QFont('Noto Sans KR Black', 28))
        elif width >= 250:  # 중간 버튼
            font_size = 80
            button.setFont(QFont('Noto Sans KR Black', 24))
        else:  # 작은 버튼
            font_size = 40
            button.setFont(QFont('Noto Sans KR Black', 20))
            
        return font_size 