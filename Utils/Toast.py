from PySide6.QtWidgets import QLabel, QApplication, QHBoxLayout
from PySide6.QtCore import Qt, QTimer, QPropertyAnimation, QEasingCurve, QRect, QPoint
from PySide6.QtGui import QColor, QPainter, QBrush, QPen, QPainterPath

class Toast(QLabel):
    """초록색 둥근 타원 배경의 토스트 메시지"""
    
    def __init__(self, message, parent=None, duration=2000):
        if parent is None:
            parent = QApplication.activeWindow()
            
        super().__init__(parent)
        
        # 메시지 설정
        self.message = message
        
        # 스타일 설정 (투명 배경 + 흰색 텍스트)
        self.setStyleSheet("""
            color: white;
            font-size: 14px;
            font-weight: normal;
            padding: 15px 20px;
            border: none;
        """)
        
        self.setText(message)
        self.setAlignment(Qt.AlignCenter)
        
        # 윈도우 설정
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Tool | Qt.WindowStaysOnTopHint)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setAttribute(Qt.WA_ShowWithoutActivating)
        
        # 크기 조정
        self.adjustSize()
        
        # 위치 설정
        self._setup_position()
        self.duration = duration
        
    def paintEvent(self, event):
        """초록색 둥근 타원 배경 그리기"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 둥근 타원 경로 생성
        path = QPainterPath()
        path.addRoundedRect(self.rect(), 15, 15)
        
        # 초록색 배경 채우기
        painter.fillPath(path, QBrush(QColor("#00A551")))
        
        # 기본 페인트 이벤트 호출 (텍스트 그리기)
        super().paintEvent(event)
    
    def _setup_position(self):
        """화면 중앙 상단에 위치하도록 설정"""
        if not self.parent():
            return
            
        parent_width = self.parent().width()
        toast_width = self.width()
        
        self.x = int((parent_width - toast_width) / 2)
        self.start_y = -self.height()
        self.end_y = 50  # 상단에서 50px 아래에 위치
    
    def show_toast(self):
        """토스트 표시 및 애니메이션"""
        self.move(self.x, self.start_y)
        self.show()
        
        # 페이드 인 애니메이션
        self.fade_in = QPropertyAnimation(self, b"pos")
        self.fade_in.setDuration(300)
        self.fade_in.setStartValue(self.pos())
        self.fade_in.setEndValue(QPoint(self.x, self.end_y))
        self.fade_in.setEasingCurve(QEasingCurve.OutQuad)
        self.fade_in.start()
        
        # 일정 시간 후 사라짐
        QTimer.singleShot(self.duration, self.start_fade_out)
    
    def start_fade_out(self):
        """페이드 아웃 애니메이션"""
        self.fade_out = QPropertyAnimation(self, b"pos")
        self.fade_out.setDuration(300)
        self.fade_out.setStartValue(self.pos())
        self.fade_out.setEndValue(QPoint(self.x, self.start_y))
        self.fade_out.setEasingCurve(QEasingCurve.InQuad)
        self.fade_out.finished.connect(self.close)
        self.fade_out.start()


class ToastManager:
    """토스트 메시지 관리자"""
    
    _instance = None
    
    @classmethod
    def instance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance
    
    def show_toast(self, message, parent=None, duration=2000):
        try:
            if parent is None:
                parent = QApplication.activeWindow()
                if not parent:
                    return False
                    
            toast = Toast(message, parent, duration)
            toast.show_toast()
            return True
            
        except Exception as e:
            print(f"Toast error: {str(e)}")
            return False