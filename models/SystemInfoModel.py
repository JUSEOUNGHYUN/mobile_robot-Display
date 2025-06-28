import os
import subprocess
from config.SerialConfig import SerialConfig

class SystemInfoModel:
    """시스템 정보 모델
    
    시스템 정보 및 심볼릭 링크 관련 데이터를 관리합니다.
    """
    
    def __init__(self):
        self.version = "TFRobot v1.0.0"
        self.tech_stack = [
            "Python 3.8",
            "PySide6 6.4.2 (Qt 6.4.2)",
            "Serial Communication Library",
            "MQTT Protocol",
        ]
        self.developer = "Dentium Robotics Team"
    
    def get_version_info(self):
        """버전 정보 반환"""
        return self.version
    
    def get_tech_stack(self):
        """기술 스택 정보 반환"""
        return self.tech_stack
    
    def get_developer_info(self):
        """개발자 정보 반환"""
        return self.developer
    
    def check_symlinks(self):
        """심볼릭 링크 정보 확인 및 반환"""
        symlink_info = []
        
        if os.name != 'posix':
            # Windows 환경에서는 심볼릭 링크 검사를 지원하지 않음
            symlink_info.append("Windows 환경에서는 심볼릭 링크를 지원하지 않습니다.")
            return symlink_info
        
        # 확인할 심볼릭 링크 목록
        symlinks = [
            SerialConfig.CALLBELL_SYMLINK, 
            "/dev/ttyMCU", 
            "/dev/ttyLIDAR"
        ]
        
        for symlink in symlinks:
            # 심볼릭 링크 존재 여부 확인
            if os.path.exists(symlink):
                try:
                    # 실제 링크 대상 확인
                    if os.path.islink(symlink):
                        target = os.readlink(symlink)
                        
                        # 상대 경로인 경우 절대 경로로 변환
                        if not os.path.isabs(target):
                            target = os.path.normpath(os.path.join(os.path.dirname(symlink), target))
                            
                        symlink_info.append(f"{symlink} → {target} (연결됨)")
                        
                        # 추가 장치 정보 확인 (예: USB 시리얼 장치 정보)
                        if 'ttyUSB' in target or 'ttyACM' in target:
                            try:
                                # udevadm info로 상세 정보 가져오기
                                result = subprocess.run(
                                    ['udevadm', 'info', '--name=' + target, '--attribute-walk'],
                                    capture_output=True, text=True, timeout=2
                                )
                                
                                # 제조사 및 제품 정보 추출
                                output = result.stdout
                                vendor = ""
                                product = ""
                                
                                for line in output.splitlines():
                                    if 'ID_VENDOR_FROM_DATABASE=' in line:
                                        vendor = line.split('=')[1].strip()
                                    if 'ID_MODEL_FROM_DATABASE=' in line:
                                        product = line.split('=')[1].strip()
                                
                                if vendor or product:
                                    symlink_info.append(f"    장치: {vendor} {product}")
                                
                            except (subprocess.TimeoutExpired, subprocess.SubprocessError):
                                symlink_info.append(f"    장치: 정보를 가져올 수 없음")
                    else:
                        symlink_info.append(f"{symlink} (일반 파일)")
                        
                except OSError:
                    symlink_info.append(f"{symlink} (읽기 오류)")
            else:
                symlink_info.append(f"{symlink} (존재하지 않음)")
        
        # ls -l /dev/tty* 명령 실행하여 추가 정보 얻기
        try:
            result = subprocess.run(
                ['ls', '-l', '/dev/ttyUSB*', '/dev/ttyACM*', '/dev/ttyS*'],
                capture_output=True, text=True, timeout=2
            )
            
            if result.returncode == 0 and result.stdout.strip():
                symlink_info.append("\n사용 가능한 시리얼 장치:")
                for line in result.stdout.strip().split('\n'):
                    symlink_info.append(f"    {line}")
        except (subprocess.TimeoutExpired, subprocess.SubprocessError):
            symlink_info.append("\n사용 가능한 시리얼 장치 정보를 가져올 수 없음")
            
        return symlink_info