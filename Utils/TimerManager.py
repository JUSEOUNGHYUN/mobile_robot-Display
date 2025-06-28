from PySide6.QtCore import QTimer
from functools import partial
import weakref

class TimerManager:
    """타이머 관리 유틸리티 클래스"""
    
    @staticmethod
    def create_single_shot_timer(parent, timeout, callback, *args, **kwargs):
        """안전한 단일 타이머 생성
        
        Args:
            parent: 타이머의 부모 객체 (타이머 관리 리스트를 가진 객체)
            timeout (int): 타임아웃 시간(밀리초)
            callback (callable): 타이머 만료 시 호출할 함수
            *args, **kwargs: 콜백 함수에 전달할 인자
            
        Returns:
            QTimer: 생성된 타이머 객체
        """
        # 부모 객체가 active_timers 속성을 갖고 있는지 확인
        if not hasattr(parent, 'active_timers'):
            parent.active_timers = []
            
        # 인자가 있는 경우 부분 함수 생성, 없으면 콜백 그대로 사용
        if args or kwargs:
            callback_func = partial(callback, *args, **kwargs)
        else:
            callback_func = callback
            
        # 타이머 생성 및 설정
        timer = QTimer(parent)
        timer.setSingleShot(True)
        
        # 약한 참조로 부모 객체 참조
        weak_parent = weakref.ref(parent)
        
        # 타이머 완료 후 정리하는 래퍼 함수
        def timer_wrapper():
            try:
                callback_func()
            except Exception as e:
                print(f"타이머 콜백 실행 중 오류: {str(e)}")
            finally:
                # 부모 객체 참조 얻기
                parent_ref = weak_parent()
                if parent_ref and hasattr(parent_ref, 'active_timers'):
                    # 활성 타이머 목록에서 제거
                    if timer in parent_ref.active_timers:
                        parent_ref.active_timers.remove(timer)
                timer.deleteLater()
        
        # 타이머 연결 및 활성 목록에 추가
        timer.timeout.connect(timer_wrapper)
        parent.active_timers.append(timer)
        timer.start(timeout)
        
        return timer
    
    @staticmethod
    def cleanup_timers(parent):
        """활성 타이머 정리
        
        Args:
            parent: 타이머 목록을 가진 객체
        """
        if not hasattr(parent, 'active_timers'):
            return
            
        for timer in parent.active_timers[:]:
            try:
                timer.stop()
                timer.deleteLater()
                parent.active_timers.remove(timer)
            except Exception as e:
                print(f"타이머 정리 중 오류: {str(e)}")
        
        parent.active_timers.clear() 