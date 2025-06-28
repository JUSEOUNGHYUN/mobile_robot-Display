from PySide6.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QLineEdit
from PySide6.QtGui import QFont
from PySide6.QtCore import Qt

class PasswordDialog(QDialog):
    """비밀번호 입력 대화상자"""
    
    def __init__(self, parent=None, title="비밀번호 입력", message="비밀번호를 입력하세요"):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setFixedSize(400, 250)
        self.setStyleSheet("background-color: black; color: white;")
        
        # 타이틀 바 제거
        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint)
        
        # 레이아웃 설정
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # 타이틀 라벨
        title_label = QLabel(title)
        title_label.setFont(QFont('Arial', 18, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("color: white; border-bottom: 1px solid #555;")
        layout.addWidget(title_label)
        
        # 메시지 라벨
        message_label = QLabel(message)
        message_label.setFont(QFont('Arial', 16))
        message_label.setAlignment(Qt.AlignCenter)
        message_label.setStyleSheet("color: white;")
        message_label.setWordWrap(True)
        layout.addWidget(message_label)
        
        # 비밀번호 입력창
        self.password_input = QLineEdit()
        self.password_input.setPlaceholderText("비밀번호 입력")
        self.password_input.setEchoMode(QLineEdit.Password)
        self.password_input.setStyleSheet("""
            QLineEdit {
                background-color: #333333;
                color: white;
                border: 1px solid #555555;
                border-radius: 5px;
                padding: 8px;
                font-size: 14px;
                margin-top: 10px;
                margin-bottom: 10px;
            }
            QLineEdit:focus {
                border: 1px solid #777777;
            }
        """)
        self.password_input.setMinimumHeight(40)
        layout.addWidget(self.password_input)
        layout.addStretch()
        
        # 버튼 레이아웃
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        
        # 버튼 스타일
        button_style = """
            QPushButton {
                background-color: #555555;
                color: white;
                border-radius: 5px;
                min-width: 80px;
                padding: 8px 16px;
            }
            QPushButton:hover {
                background-color: #777777;
            }
        """
        
        # 취소 버튼
        cancel_button = QPushButton("취소")
        cancel_button.setFont(QFont('Arial', 14))
        cancel_button.setStyleSheet(button_style)
        cancel_button.clicked.connect(self.reject)
        button_layout.addWidget(cancel_button)
        
        # 확인 버튼
        ok_button = QPushButton("연결")
        ok_button.setFont(QFont('Arial', 14))
        ok_button.setStyleSheet(button_style)
        ok_button.clicked.connect(self.accept)
        button_layout.addWidget(ok_button)
        
        button_layout.addStretch()
        layout.addLayout(button_layout)
        
        # 비밀번호 저장 변수
        self.password = ""
        
    def accept(self):
        """확인 버튼 처리"""
        self.password = self.password_input.text()
        super().accept()
    
    def get_password(self):
        """입력된 비밀번호 반환"""
        return self.password
    
    @staticmethod
    def show_dialog(parent=None, title="비밀번호 입력", message="비밀번호를 입력하세요"):
        """비밀번호 입력 대화상자 표시"""
        dialog = PasswordDialog(parent, title, message)
        result = dialog.exec()
        if result == QDialog.Accepted:
            return dialog.get_password(), True
        return "", False 