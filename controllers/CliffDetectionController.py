from PyQt5.QtCore import QObject
from models.CliffDetectionModel import CliffDetectionModel
from views.CliffDetectionView import CliffDetectionView

class CliffDetectionController(QObject):
    def __init__(self, main_window):
        super().__init__()
        self.model = CliffDetectionModel()
        self.view = CliffDetectionView()
        self.main_window = main_window
        
        # 시그널 연결
        self.model.cliff_detected.connect(self._on_cliff_detected)
        self.view.confirm_clicked.connect(self._on_confirm_clicked)
        
    def _on_cliff_detected(self, detected):
        if detected:
            self.main_window.setCentralWidget(self.view)
            self.view.show_warning()
        else:
            self.main_window.previous_screen()  # 이전 화면으로 전환
            
    def _on_confirm_clicked(self):
        self.model.reset_detection()
        
    def show(self):
        """화면 표시 및 구독 활성화"""
        self.model.activate()
        self.view.show()
        
    def hide(self):
        """화면 숨김 및 구독 비활성화"""
        self.model.deactivate()
        self.view.hide() 
        
    def cleanup(self):
        """리소스 정리"""
        self.model.cleanup() 