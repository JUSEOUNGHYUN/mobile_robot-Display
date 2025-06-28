from PySide6.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton
from PySide6.QtGui import QFont
from PySide6.QtCore import Qt, Signal

class MessageBox(QDialog):
    """사용자 정의 메시지 박스
    
    다양한 버튼 옵션과 타이틀을 설정할 수 있는 커스텀 메시지 박스
    """
    # 버튼 클릭 결과를 위한 상수
    YES = 1
    NO = 2
    CANCEL = 3
    OK = 4
    
    # 결과 시그널 (어떤 버튼이 클릭되었는지 알려줌)
    result_signal = Signal(int)
    
    def __init__(self, parent=None, title="알림", message="", button_labels=["확인"]):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setFixedSize(400, 200)
        self.setStyleSheet("background-color: white; color: black;")
        
        # 타이틀 바 제거
        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint)
        
        # 레이아웃 설정
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # 타이틀 라벨
        title_label = QLabel(title)
        title_label.setFont(QFont('Noto Sans KR Black', 18, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("color: black; border-bottom: 1px solid #555;")
        layout.addWidget(title_label)
        
        # 메시지 라벨
        message_label = QLabel(message)
        message_label.setFont(QFont('Noto Sans KR Black', 20))
        message_label.setAlignment(Qt.AlignCenter)
        message_label.setStyleSheet("color: black;")
        message_label.setWordWrap(True)
        layout.addWidget(message_label)
        layout.addStretch()
        
        # 버튼 레이아웃
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        
        # 버튼 스타일
        button_style = """
            QPushButton {
                background-color: #2A2A2A;
                background-color: rgba(255, 255, 255, 180); /* 투명도 추가 */
                border: 1px solid #444444;
		        min-width: 80px;
                border-radius: 6px;
            }
            QPushButton:hover {
                background-color: #3A3A3A;
                background-color: rgba(58, 58, 58, 200); /* hover 시 투명도 */
                border: 1px solid #666666;
            }
            QPushButton:pressed {
                background-color: rgba(51, 51, 51, 220); /* 클릭 시 투명도 */
            }
            QPushButton:checked {
                background-color: rgba(70, 130, 180, 220); /* 클릭 시 투명도 */
            }
        """
        
        # 버튼 생성 및 이벤트 연결을 위한 함수
        def create_button_handler(result_value):
            return lambda: self.on_button_clicked(result_value)
            
        # 버튼 생성
        self.buttons = []
        for label in button_labels:
            button = QPushButton(label)
            button.setFont(QFont('Arial', 14))
            button.setStyleSheet(button_style)
            
            # 버튼 클릭 시 결과 값 설정 (클로저 문제 해결)
            if label == "예" or label.lower() == "yes":
                button.clicked.connect(create_button_handler(self.YES))
            elif label == "아니오" or label.lower() == "no":
                button.clicked.connect(create_button_handler(self.NO))
            elif label == "취소" or label.lower() == "cancel":
                button.clicked.connect(create_button_handler(self.CANCEL))
            else:  # 기본은 OK 반환
                button.clicked.connect(create_button_handler(self.OK))
            
            button_layout.addWidget(button)
            self.buttons.append(button)
        
        button_layout.addStretch()
        layout.addLayout(button_layout)
        
        # 결과 저장 변수
        self.result = self.CANCEL  # 기본값
        
    def on_button_clicked(self, result):
        """버튼 클릭 처리"""
        self.result = result
        self.result_signal.emit(result)
        self.accept()
        
    def get_result(self):
        """다이얼로그 결과 반환"""
        return self.result
    
    @staticmethod
    def show_message(parent=None, title="알림", message="", button_labels=["확인"]):
        """간편하게 메시지 박스를 표시하고 결과를 반환하는 정적 메서드"""
        dialog = MessageBox(parent, title, message, button_labels)
        dialog.exec()
        return dialog.get_result()

    @staticmethod
    def show_confirmation(parent=None, message="변경 내용을 저장하시겠습니까?"):
        """확인 메시지 박스 표시 (예/아니오/취소)"""
        return MessageBox.show_message(
            parent, 
            title="확인", 
            message=message, 
            button_labels=["예", "아니오", "취소"]
        )
        
    @staticmethod
    def show_error(parent=None, message="오류가 발생했습니다."):
        """오류 메시지 박스 표시"""
        return MessageBox.show_message(
            parent,
            title="오류",
            message=message,
            button_labels=["확인"]
        )
        
    @staticmethod
    def show_info(parent=None, message=""):
        """정보 메시지 박스 표시"""
        return MessageBox.show_message(
            parent,
            title="정보",
            message=message,
            button_labels=["확인"]
        )
        
    @staticmethod
    def show_warning(parent=None, message=""):
        """경고 메시지 박스 표시"""
        return MessageBox.show_message(
            parent,
            title="경고",
            message=message,
            button_labels=["확인", "취소"]
        )
