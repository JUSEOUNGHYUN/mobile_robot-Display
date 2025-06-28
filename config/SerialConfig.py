"""시리얼 포트 설정 모듈

이 모듈은 애플리케이션에서 사용하는 모든 시리얼 포트 관련 설정을 중앙에서 관리합니다.
"""
import os

class SerialConfig:
    """시리얼 포트 설정 클래스"""
    
    # 기본 전송 속도
    DEFAULT_BAUDRATE = 115200
    
    # Linux 환경의 시리얼 포트 목록
    LINUX_PORTS = [
        '/dev/ttyUSB0', '/dev/ttyUSB1', 
        '/dev/ttyACM0', '/dev/ttyACM1',
        '/dev/ttyS0', '/dev/ttyS1', '/dev/ttyS2', '/dev/ttyS3',
        '/dev/ttyTHS0', '/dev/ttyTHS1', '/dev/ttyTHS4', '/dev/ttyTCU0'
    ]
    
    # 호출벨 심볼릭 링크 경로
    CALLBELL_SYMLINK = '/dev/ttyCallBell'
    
    @classmethod
    def get_default_port(cls):
        """기본 시리얼 포트 반환"""
        return '/dev/ttyUSB0'
    
    @classmethod
    def get_ports_to_try(cls):
        """시도할 포트 목록 반환"""
        return cls.LINUX_PORTS
            
    @classmethod
    def get_ports_to_chmod(cls):
        """chmod로 권한을 부여할 포트 목록 반환"""
        # 주요 시리얼 포트만 권한 부여
        return [
            '/dev/ttyTHS0', '/dev/ttyTHS1', 
            '/dev/ttyS0', '/dev/ttyS1', '/dev/ttyS2', '/dev/ttyS3',
            '/dev/ttyTCU0'
        ] 