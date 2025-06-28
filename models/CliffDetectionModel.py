from PyQt5.QtCore import QObject, pyqtSignal
from Utils.RosTopic import EmergencyButtonSubscriber

class CliffDetectionModel(QObject):
    cliff_detected = pyqtSignal(bool)
    
    def __init__(self):
        super().__init__()
        self._is_cliff_detected = False
        self.emergency_subscriber = None
        
    def activate(self):
        """구독 활성화"""
        if self.emergency_subscriber is None:
            self.emergency_subscriber = EmergencyButtonSubscriber()
            self.emergency_subscriber.emergency_state_changed.connect(self._on_emergency_state_changed)
            print("[CliffDetectionModel] 비상 버튼 구독 활성화")
        
    def deactivate(self):
        """구독 비활성화"""
        if self.emergency_subscriber is not None:
            self.emergency_subscriber.cleanup()
            self.emergency_subscriber = None
            print("[CliffDetectionModel] 비상 버튼 구독 비활성화")
        
    def _on_emergency_state_changed(self, state):
        self.is_cliff_detected = state
        
    @property
    def is_cliff_detected(self):
        return self._is_cliff_detected
        
    @is_cliff_detected.setter
    def is_cliff_detected(self, value):
        self._is_cliff_detected = value
        self.cliff_detected.emit(value)
        
    def reset_detection(self):
        self.is_cliff_detected = False 
        
    def cleanup(self):
        """리소스 정리"""
        self.deactivate() 