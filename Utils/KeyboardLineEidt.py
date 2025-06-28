import subprocess
from PySide6.QtWidgets import QLineEdit
from PySide6.QtCore import Qt, QEvent

class KeyboardLineEdit(QLineEdit):
    """가상 키보드를 지원하는 QLineEdit 커스텀 클래스"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_AcceptTouchEvents)  # 터치 이벤트 수락 설정
        
        # 제스처 설정
        self.grabGesture(Qt.TapGesture)
        self.grabGesture(Qt.TapAndHoldGesture)
        
    def focusInEvent(self, event):
        """포커스를 받으면 가상 키보드 실행"""
        self.launch_keyboard()
        super().focusInEvent(event)
        
    def mousePressEvent(self, event):
        """마우스 프레스 이벤트 처리"""
        print("KeyboardLineEdit: 마우스 프레스 이벤트 감지됨")
        self.launch_keyboard()
        super().mousePressEvent(event)
        
    def launch_keyboard(self):
        """가상 키보드 실행 함수"""
        try:
            # --xid 옵션 추가: 항상 최상위에 표시
            # -s 옵션: 키보드 크기 설정
            # --layout 옵션: 레이아웃 설정
            # --keep-aspect 옵션: 종횡비 유지
            # --always-on-top 옵션: 항상 위에 표시
            subprocess.Popen(["onboard"])
            #subprocess.Popen(["onboard", "--xid", str(self.winId()), "-s", "800x400", "--layout", "default", "--keep-aspect", "--always-on-top"])
            subprocess.Popen(["wmctrl", "-r", "Onboard", "-b", "add,above"])
        except Exception as e:
            print(f"가상 키보드 실행 오류: {e}")
            
    def event(self, event):
        """모든 이벤트 처리"""
        if event.type() == QEvent.TouchBegin:
            print("KeyboardLineEdit: 터치 이벤트 감지됨")
            self.launch_keyboard()
            self.setFocus()
            return True
        return super().event(event)