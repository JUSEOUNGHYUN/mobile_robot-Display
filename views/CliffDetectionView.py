from PyQt5 import uic
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import pyqtSignal

class CliffDetectionView(QWidget):
    confirm_clicked = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        uic.loadUi('UI/CliffDetection.ui', self)
        
        # 버튼 시그널 연결
        self.confirm_button.clicked.connect(self.confirm_clicked.emit)
        
    def show_warning(self):
        self.warning_label.setVisible(True)
        self.message_label.setVisible(True)
        self.detail_label.setVisible(True)
        
    def hide_warning(self):
        self.warning_label.setVisible(False)
        self.message_label.setVisible(False)
        self.detail_label.setVisible(False) 