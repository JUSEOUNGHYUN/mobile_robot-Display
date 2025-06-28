import serial
import time
import binascii
import os
import serial.tools.list_ports
import json
from PySide6.QtCore import QObject, Signal, QTimer, QCoreApplication
from PySide6.QtWidgets import QApplication, QStackedWidget, QWidget
from config.SerialConfig import SerialConfig
from views.MovingView import MovingView
from views.PauseView import PauseView
from Utils.Toast import ToastManager
from Utils.MessageBox import MessageBox

# 전역 변수로 MainController 참조 저장
MAIN_CONTROLLER = None

def set_main_controller(controller):
    """메인 컨트롤러 참조 설정 (앱 초기화 후 호출)"""
    global MAIN_CONTROLLER
    MAIN_CONTROLLER = controller
    print(f"[CallBellManager] 메인 컨트롤러 참조 설정됨: {controller is not None}")

class CallBellManager(QObject):
    """
    호출벨(링고벨) 관리 클래스
    
    호출벨 UUID 추출, 현재 감지된 호출벨 관리, 중복 체크 등의 기능을 제공합니다.
    """
    
    # =========================================================================
    # 1. 싱글톤 및 시그널 정의
    # =========================================================================
    # 시그널 정의
    uuid_signal = Signal(str)  # UUID 추출 시 발생하는 시그널 (문자열 파라미터)
    connection_signal = Signal(bool)  # 연결 상태 변경 시 발생하는 시그널
    callbell_detected_signal = Signal(str)  # 감지된 호출벨 UUID
    callbell_status_changed_signal = Signal(str, int)  # UUID, 상태 코드
    
    # 싱글톤 인스턴스
    _instance = None
    
    @classmethod
    def instance(cls):
        """싱글톤 인스턴스 반환"""
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance
    
    # =========================================================================
    # 2. 초기화 및 기본 설정
    # =========================================================================
    def __init__(self):
        """초기화"""
        super().__init__()
        
        # 시리얼 포트 관련 변수
        self.port = None  # 자동 검색으로 변경
        self.baud_rate = SerialConfig.DEFAULT_BAUDRATE
        self.serial_conn = None
        self.is_connected = False
        
        # 이동 방향 제어
        self.direction = 'stop'  # 'forward', 'backward', 'left', 'right', 'stop'
        
        # UUID 관련 변수
        self.uuid_value = None  # 현재 추출된 UUID 값
        self.temp_uuid = None  # 임시 저장 UUID (등록 안된 UUID)
        
        # UUID 디바운싱 (중복 처리 방지)
        self._last_processed_uuid = None  # 마지막으로 처리한 UUID
        self._last_uuid_process_time = 0  # 마지막 처리 시간 (밀리초)
        self._uuid_debounce_time = 3000  # UUID 디바운스 시간 (밀리초)
        
        # 이동 중 다른 호출벨 무시 설정
        self.ignore_other_bell = True  # 기본값: 이동 중 다른 호출벨 무시함(True)
        print(f"[CallBellManager] 초기화: 이동 중 다른 호출벨 무시 설정 = {self.ignore_other_bell}")
        
        # 위치 데이터 캐시
        self.location_cache = []
        
        # UUID 추출 관련 변수
        self.uuid_extracted = False
        self.uuid_already_processed = False
        
        # UUID 메모리 캐시
        self.registered_uuids = []
        self._load_registered_uuids()
        
        # 지속적인 데이터 수신을 위한 타이머
        self.read_timer = QTimer(self)
        self.read_timer.timeout.connect(self.check_data)
        self.read_timer.setInterval(20)  # 20ms마다 확인
        
        # 화면 상태 캐시 및 타이머
        self._current_view_cache = None
        self._view_cache_time = 0
        self._view_cache_valid_time = 500  # 캐시 유효 시간 (밀리초)
        
        # 호출벨 관련 변수
        self.last_detected_callbell_id = None  # 마지막으로 감지된 호출벨 ID
        self.is_callbell_active = False        # 호출벨 활성화 상태
        
        # 자동 연결 시도
        QTimer.singleShot(500, self.auto_connect)
    
    def _load_registered_uuids(self):
        """등록된 UUID 목록과 위치 정보를 메모리에 로드"""
        try:
            with open('data/locations.txt', 'r', encoding='utf-8') as f:
                locations = json.load(f)
            
            # 위치 정보 전체 캐시
            self.location_cache = locations
            
            # 등록된 UUID 추출
            self.registered_uuids = []
            for location in locations:
                if ('unique_id' in location and 
                    location.get('label') == 'Yes' and
                    location['unique_id']):
                    self.registered_uuids.append(location['unique_id'].upper())
            
            print(f"등록된 UUID 목록 로드 완료: {len(self.registered_uuids)}개")
        except Exception as e:
            print(f"UUID 목록 로드 중 오류: {str(e)}")
            self.registered_uuids = []
            self.location_cache = []
            
    def refresh_uuids(self):
        """UUID 목록 및 위치 정보 새로고침"""
        self._load_registered_uuids()
        
    def is_uuid_registered(self, uuid_value):
        """UUID가 등록되어 있는지 확인 (메모리 캐시 활용)"""
        if not uuid_value:
            return False
        return uuid_value.upper() in self.registered_uuids
    
    def get_cached_locations(self):
        """메모리에 캐시된 모든 위치 정보 반환"""
        return self.location_cache.copy() if self.location_cache else []
        
    def get_location_by_uuid(self, uuid_value):
        """UUID로 등록된 위치 데이터 찾기"""
        if not uuid_value or not self.is_uuid_registered(uuid_value):
            return None
            
        # 메모리 캐시에서 위치 정보 찾기 (파일 I/O 없음)
        uuid_upper = uuid_value.upper()
        for location in self.location_cache:
            if ('unique_id' in location and 
                location.get('label') == 'Yes' and
                location['unique_id'].upper() == uuid_upper):
                return location
                
        return None
    
    def _get_location_by_uuid(self, uuid_value):
        """UUID로 등록된 위치 데이터 찾기 (내부 메서드)"""
        return self.get_location_by_uuid(uuid_value)
    
    # =========================================================================
    # 3. 시리얼 연결 관리
    # =========================================================================
    def auto_connect(self):
        """자동으로 사용 가능한 포트 찾아서 연결"""
        print("호출벨 자동 연결 시도 중...")
        
        # 심볼릭 링크를 가장 먼저 시도
        if os.name == 'posix':  # Linux 환경
            symlink_path = "/dev/ttyCallBell"
            
            # 심볼릭 링크 존재 확인 후 연결 시도
            if os.path.exists(symlink_path):
                print(f"호출벨 심볼릭 링크 {symlink_path} 발견, 연결 시도 중...")
                try:
                    if self.connect(symlink_path, self.baud_rate):
                        print(f"호출벨 심볼릭 링크 연결 성공: {symlink_path}")
                        # 초기화 명령 전송
                        self.send_command("C501")
                        time.sleep(0.2)  # 초기화 명령 처리 시간 대기
                        self.request_uuid()  # UUID 요청
                        return True
                    else:
                        print(f"호출벨 심볼릭 링크 연결 실패: {symlink_path}, 다른 포트 시도...")
                except Exception as e:
                    print(f"심볼릭 링크 연결 시도 중 오류: {str(e)}")
            else:
                print(f"호출벨 심볼릭 링크가 존재하지 않습니다: {symlink_path}, 다른 포트 시도...")
        
        # 알려진 포트 목록 시도
        ports_to_try = SerialConfig.get_ports_to_try()
        
        # 시스템에서 감지된 포트도 추가
        detected_ports = [port.device for port in serial.tools.list_ports.comports()]
        for port in detected_ports:
            if port not in ports_to_try:
                ports_to_try.insert(0, port)  # 감지된 포트를 우선순위로 배치
        
        # 각 포트 연결 시도
        for port in ports_to_try:
            # 이미 시도한 심볼릭 링크는 건너뜀
            if os.name == 'posix' and port == symlink_path:
                continue
                
            print(f"포트 {port} 연결 시도 중...")
            if self.connect(port, self.baud_rate):
                print(f"호출벨 자동 연결 성공: {port}")
                # 초기화 명령 전송
                self.send_command("C501")
                time.sleep(0.2)  # 초기화 명령 처리 시간 대기
                self.request_uuid()  # UUID 요청
                
                # 연결 성공한 포트의 심볼릭 링크 생성 안내 메시지
                if os.name == 'posix' and port != symlink_path:
                    print(f"알림: 다음 명령으로 심볼릭 링크를 생성하면 향후 연결이 더 안정적일 수 있습니다:")
                    print(f"sudo ln -sf {port} {symlink_path}")
                return True
        print("호출벨 자동 연결 실패. 모든 포트를 시도했습니다.")
        return False
    
    def connect(self, port=None, baud_rate=None):
        """시리얼 포트 연결"""
        if port:
            self.port = port
        if baud_rate:
            self.baud_rate = baud_rate
        
        try:
            # 기존 연결이 있으면 먼저 닫기
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                self.read_timer.stop()
            
            # 새로운 연결 시도
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            
            # 연결 성공 처리
            self.is_connected = True
            self.connection_signal.emit(True)
            
            # 지속적인 데이터 수신 시작
            self.read_timer.start()
            
            print(f"호출벨 포트 {self.port} 연결 성공")
            return True
            
        except Exception as e:
            print(f"호출벨 포트 {self.port} 연결 오류: {e}")
            self.is_connected = False
            self.connection_signal.emit(False)
            return False
    
    def disconnect(self):
        """시리얼 포트 연결 해제"""
        try:
            # 데이터 수신 타이머 중지
            if self.read_timer.isActive():
                self.read_timer.stop()
                
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                self.is_connected = False
                self.connection_signal.emit(False)
                return True
        except Exception as e:
            pass
        return False
    
    def close(self):
        """시리얼 포트 닫기"""
        try:
            print("[CallBellManager] 리소스 정리 시작...")
            
            # 타이머 중지
            if hasattr(self, 'read_timer') and self.read_timer.isActive():
                print("[CallBellManager] 타이머 중지 중...")
                self.read_timer.stop()
                
            # 시리얼 연결 닫기
            if self.serial_conn and self.serial_conn.is_open:
                print("[CallBellManager] 시리얼 포트 닫는 중...")
                self.serial_conn.close()
                self.is_connected = False
                self.connection_signal.emit(False)
            
            # 기타 리소스 정리
            self.uuid_extracted = False
            self.uuid_value = None
            self.uuid_already_processed = False
            self.temp_uuid = None
            
            print("[CallBellManager] 리소스 정리 완료")
            return True
        except Exception as e:
            print(f"[CallBellManager] 리소스 정리 중 오류: {e}")
            return False
    
    # =========================================================================
    # 4. 시리얼 데이터 송신
    # =========================================================================
    def send_command(self, command):
        """명령어 전송"""
        if not self.is_connected or not self.serial_conn or not self.serial_conn.is_open:
            print("호출벨이 연결되어 있지 않습니다.")
            return False
        
        try:
            if isinstance(command, str):
                # 16진수 문자열을 바이트로 변환
                cmd_bytes = binascii.unhexlify(command)
            else:
                # 이미 바이트 형식인 경우
                cmd_bytes = command
                
            self.serial_conn.write(cmd_bytes)
            hex_data = ' '.join([f'{b:02X}' for b in cmd_bytes])
            print(f"SND>> {hex_data}")
            return True
            
        except Exception as e:
            print(f"호출벨 명령 전송 오류: {e}")
            return False
    
    def request_uuid(self):
        """UUID 요청 명령 전송"""
        # 이미 UUID가 추출되었고 처리되었다면 요청하지 않음
        if self.uuid_extracted and self.uuid_already_processed:
            print(f"이미 UUID({self.uuid_value})가 추출되어 있습니다. 새로운 요청 없음.")
            return True
            
        print("호출벨 UUID 요청 명령 전송")
        # A505 명령 생성 (03 01 00 08 A5 05 00 DC)
        command = bytearray([
            0x03, 0x01, 0x00, 0x08,     # 헤더
            0xA5, 0x05,                 # 명령어 (UUID 요청)
            0x00, 0xDC                  # 체크섬
        ])
        
        # 명령 전송 결과 확인
        result = self.send_command(command)
        print(f"UUID 요청 명령 전송 결과: {result}")
        
        # 연결 상태 확인
        if self.is_connected and self.serial_conn and self.serial_conn.is_open:
            print(f"호출벨 연결 상태: 정상 (포트: {self.port})")
        else:
            print(f"호출벨 연결 상태: 비정상 (포트: {self.port})")
            
        return result
    
    def send_ccc_response(self):
        """A501 응답 메시지 자동 전송 (C501)"""
        try:
            # C501 명령 전송 (03 01 00 08 C5 01 00 2E)
            command = bytearray([
                0x03, 0x01, 0x00, 0x08,     # 헤더
                0xC5, 0x01,                 # 명령어 (응답)
                0x00, 0x2E                  # Result, CheckSum
            ])
            
            # 명령 전송
            self.serial_conn.write(command)
        except Exception as e:
            pass
    
    # =========================================================================
    # 5. 시리얼 데이터 수신 및 처리
    # =========================================================================
    def check_data(self):
        """타이머에 의한 지속적인 데이터 체크"""
        if not self.is_connected or not self.serial_conn or not self.serial_conn.is_open:
            return
            
        try:
            # 수신 데이터가 있는지 확인
            if self.serial_conn.in_waiting:
                data = self.serial_conn.read(self.serial_conn.in_waiting)
                if data:
                    # 데이터 처리
                    self.process_data(data)
        except Exception as e:
            print(f"데이터 체크 중 오류 발생: {str(e)}")
    
    def process_data(self, data):
        """데이터 처리"""
        try:
            # 데이터 패턴 확인 및 UUID 추출
            if len(data) >= 12:
                # 16진수로 변환하여 확인
                hex_data = binascii.hexlify(data).decode().upper()
                
                # 로그 출력
                print(f"RCV<< {' '.join([f'{b:02X}' for b in data])}")
                
                # 패킷 유효성 확인 (첫 3바이트가 03 01 00인지)
                if data[0] == 0x03 and data[1] == 0x01 and data[2] == 0x00:
                    # 옵션 코드 추출 (패킷의 5-6번째 바이트)
                    option_code = (data[4] << 8) | data[5]
                    
                    print(f"옵션 코드: 0x{option_code:04X}")
                    
                    # A501 응답인 경우 C501 자동 응답
                    if option_code == 0xA501:
                        print("A501 응답 수신, C501 자동 응답")
                        self.send_ccc_response()
                    
                    # A505 응답인 경우 UUID 추출 (호출벨 버튼 눌림)
                    elif option_code == 0xA505:
                        print("A505 응답 수신, UUID 추출 시작")
                        return self._extract_and_process_uuid(data)
                    else:
                        print(f"알 수 없는 옵션 코드: 0x{option_code:04X}")
                else:
                    print(f"유효하지 않은 패킷 헤더: {data[0]:02X} {data[1]:02X} {data[2]:02X}")
            else:
                print(f"데이터 길이 부족: {len(data)} 바이트 (최소 12 바이트 필요)")
        except Exception as e:
            print(f"데이터 처리 중 오류 발생: {str(e)}")
            
        return False
    
    # =========================================================================
    # 6. UUID 추출 및 처리
    # =========================================================================
    def _extract_and_process_uuid(self, data):
        """A505 응답에서 UUID를 추출하고 처리"""
        print(f"A505 응답 수신 - UUID 추출 시도")
        # 모든 가능한 패턴 시도
        patterns_to_try = [
            # 패턴 1: 데이터 12~15 바이트 (일반적인 위치)
            lambda d: (d[12] << 24) | (d[13] << 16) | (d[14] << 8) | d[15] if len(d) >= 16 else None,
            
            # 패턴 2: 데이터 11~14 바이트 (대체 위치)
            lambda d: (d[11] << 24) | (d[12] << 16) | (d[13] << 8) | d[14] if len(d) >= 15 else None
            
            # 패턴 3 삭제됨: 더 이상 사용하지 않음
        ]
        
        uuid_extracted_success = False
        # 모든 패턴 시도
        for i, pattern_func in enumerate(patterns_to_try):
            try:
                uuid_value = pattern_func(data)
                if uuid_value:
                    # UUID 저장
                    self.uuid_extracted = True
                    self.uuid_value = f"{uuid_value:08X}"
                    self.uuid_already_processed = True
                    self.temp_uuid = self.uuid_value  # 임시 변수에 저장
                    print(f"호출벨 UUID 추출 완료 (패턴 {i+1}): {self.uuid_value}")
                    uuid_extracted_success = True
                    break  # 성공하면 패턴 시도 중단
            except Exception as e:
                continue  # 오류 발생 시 다음 패턴 시도
        
        # UUID 추출 성공 시 처리
        if uuid_extracted_success:
            return self._handle_extracted_uuid()
        
        return False
    
    def get_temp_uuid(self):
        """임시 저장된 UUID 반환"""
        return self.temp_uuid

    def clear_temp_uuid(self):
        """임시 UUID 초기화"""
        self.temp_uuid = None
    
    def get_current_uuid(self):
        """현재 저장된 UUID 반환"""
        return self.uuid_value if self.uuid_extracted else None
    
    def reset_uuid(self):
        """UUID 관련 상태 리셋 - 새로운 UUID를 받을 수 있도록 합니다"""
        self.uuid_extracted = False
        self.uuid_already_processed = False
        # uuid_value는 유지 (마지막으로 수신한 값이 필요할 수 있음)
        print(f"UUID 상태 리셋 완료 (이전 UUID 값: {self.uuid_value})")
        return True
    
    def reset_callbell(self):
        """호출벨 상태를 초기화"""
        self.last_detected_callbell_id = None
        self.is_callbell_active = False
        return True
        
    def get_current_callbell_id(self):
        """현재 감지된 호출벨 ID 반환"""
        return self.last_detected_callbell_id
        
    def is_callbell_active_status(self):
        """호출벨 활성화 상태 확인"""
        return self.is_callbell_active
        
    def on_callbell_detected(self, callbell_id):
        """호출벨이 감지되었을 때 호출되는 메서드
        
        Args:
            callbell_id (str): 감지된 호출벨의 고유 ID (MAC 주소 등)
        
        Returns:
            int: 호출벨 상태 코드 (0=새로운 감지, 1=동일 감지, 2=등록된 호출벨)
        """
        # 디바운싱 처리 - 짧은 시간 내 동일 UUID 무시
        current_time = int(time.time() * 1000)  # 현재 시간 (밀리초)
        
        # 같은 UUID가 디바운스 시간 내에 다시 들어온 경우
        if (self._last_processed_uuid == callbell_id and 
            (current_time - self._last_uuid_process_time) < self._uuid_debounce_time):
            print(f"UUID {callbell_id} 디바운싱: 짧은 시간 내 중복 호출 무시 (경과: {current_time - self._last_uuid_process_time}ms)")
            return 1  # 동일 감지
        
        # 현재 UUID와 시간 기록 (디바운싱용)
        self._last_processed_uuid = callbell_id
        self._last_uuid_process_time = current_time
        
        # 이전에 감지된 호출벨과 동일한 경우
        if self.is_callbell_active and callbell_id == self.last_detected_callbell_id:
            print(f"동일한 호출벨 감지됨: {callbell_id}")
            self.callbell_status_changed_signal.emit(callbell_id, 1)  # 동일 감지
            return 1  # 동일 감지
        
        # 새로운 호출벨 감지
        print(f"새로운 호출벨 감지됨: {callbell_id}")
        self.last_detected_callbell_id = callbell_id
        self.is_callbell_active = True
        
        # 등록된 호출벨인지 확인
        status_code = 2 if self.is_uuid_registered(callbell_id) else 0
        
        # 시그널 발생
        self.callbell_detected_signal.emit(callbell_id)
        self.callbell_status_changed_signal.emit(callbell_id, status_code)
        
        return status_code
    
    # =========================================================================
    # 7. 화면 상태 확인 유틸리티
    # =========================================================================
    def _get_current_view_name(self):
        """현재 화면 이름 가져오기 (캐시 사용)"""
        # 캐시가 유효한지 확인
        current_time = time.time() * 1000  # 현재 시간 (밀리초)
        if (self._current_view_cache is not None and 
            (current_time - self._view_cache_time) < self._view_cache_valid_time):
            return self._current_view_cache
            
        view_name = None
        
        # 1. MainController를 통해 확인 (가장 빠른 방법)
        if MAIN_CONTROLLER is not None and hasattr(MAIN_CONTROLLER, "stack"):
            current_widget = MAIN_CONTROLLER.stack.currentWidget()
            if current_widget:
                view_name = current_widget.__class__.__name__
        
        # 2. 메인 컨트롤러가 없거나 확인 실패 시 QApplication을 통해 확인
        if not view_name:
            current_window = QApplication.activeWindow()
            if current_window:
                view_name = current_window.__class__.__name__
        
        # 캐시 업데이트
        self._current_view_cache = view_name
        self._view_cache_time = current_time
        
        return view_name
    
    def _is_location_add_view(self):
        """현재 화면이 LocationAddView인지 확인"""
        view_name = self._get_current_view_name()
        return view_name == 'LocationAddView'
    
    def _is_moving_or_pause_view(self):
        """현재 화면이 이동 중 또는 일시정지 화면인지 확인"""
        view_name = self._get_current_view_name()
        return view_name in ['MovingView', 'PauseView']
    
    def _is_arrive_view(self):
        """현재 화면이 ArriveView인지 확인"""
        view_name = self._get_current_view_name()
        return view_name == 'ArriveView'

    # =========================================================================
    # 8. UUID 추출 후 UI 업데이트
    # =========================================================================
    def _emit_uuid_signal(self, uuid_value):
        """UI 스레드에서 UUID 시그널 발생"""
        try:
            print(f"UI 스레드에서 UUID 시그널 발생: {uuid_value}")
            # 시그널 방식으로 전송
            self.uuid_signal.emit(uuid_value)
            
            # 시그널 방식이 실패할 경우를 대비해 직접 접근 방식도 시도
            QTimer.singleShot(100, lambda: self._try_direct_set_uuid(uuid_value))
        except Exception as e:
            print(f"시그널 발생 오류: {str(e)}")
            # 오류 발생 시 직접 접근 방식으로 시도
            self._try_direct_set_uuid(uuid_value)

    def _try_direct_set_uuid(self, uuid_value):
        """시그널이 작동하지 않을 경우, 직접 LocationAddView의 set_unique_id 메서드 호출"""
        # 현재 화면이 LocationAddView인지 확인 (캐시 활용)
        if not self._is_location_add_view():
            return False
            
        try:
            # 지연 임포트 - 순환 참조 방지
            from views.LocationAddView import LocationAddView
            
            # MainController를 통해 현재 화면 가져오기 (가장 효율적인 방법)
            if MAIN_CONTROLLER and hasattr(MAIN_CONTROLLER, "stack"):
                current_widget = MAIN_CONTROLLER.stack.currentWidget()
                if current_widget and hasattr(current_widget, 'set_unique_id'):
                    print(f"LocationAddView.set_unique_id 직접 호출: {uuid_value}")
                    current_widget.set_unique_id(uuid_value)
                    return True
                    
            # 위 방법으로 실패 시 QApplication을 통해 시도
            current_window = QApplication.activeWindow()
            if isinstance(current_window, LocationAddView) and hasattr(current_window, 'set_unique_id'):
                print(f"ActiveWindow에서 LocationAddView.set_unique_id 직접 호출: {uuid_value}")
                current_window.set_unique_id(uuid_value)
                return True
                
            return False
        except Exception as e:
            print(f"직접 UUID 설정 시도 중 오류: {str(e)}")
            return False
    
    # =========================================================================
    # 9. UUID 기반 목적지 관리
    # =========================================================================
    def _handle_extracted_uuid(self):
        """추출된 UUID 처리"""
        # 디바운싱 로직 - 동일 UUID의 짧은 시간 내 중복 처리 방지
        current_time = int(time.time() * 1000)  # 현재 시간 (밀리초)
        
        # 같은 UUID가 디바운스 시간 내에 다시 들어온 경우
        if (self._last_processed_uuid == self.uuid_value and 
            (current_time - self._last_uuid_process_time) < self._uuid_debounce_time):
            print(f"UUID {self.uuid_value} 디바운싱: 짧은 시간 내 중복 호출 무시 (경과: {current_time - self._last_uuid_process_time}ms)")
            return True
        
        # 현재 UUID와 시간 기록 (디바운싱용)
        self._last_processed_uuid = self.uuid_value
        self._last_uuid_process_time = current_time
        
        # 현재 화면이 LocationAddView인지 확인
        is_add_view = self._is_location_add_view()
        
        # 이동 중 또는 일시정지 화면인지 확인
        if self._is_moving_or_pause_view():
            print(f"현재 이동 중 또는 일시정지 화면입니다.")
            
            # 새로운 호출벨 위치가 등록된 목적지인지 확인
            new_location = self._get_location_by_uuid(self.uuid_value)
            
            # 새로운 목적지가 없으면 무시
            if not new_location:
                print(f"UUID {self.uuid_value}는 등록되지 않은 호출벨입니다. 무시합니다.")
                return True
                
            # 현재 이동 중인 목적지 정보 가져오기
            current_destination = self._get_current_destination()
            if current_destination and current_destination.get('unique_id') == new_location.get('unique_id'):
                print(f"현재 이동 중인 목적지와 동일한 호출벨입니다. UUID {self.uuid_value} 이벤트 무시.")
                ToastManager.instance().show_toast(f"이미 {new_location['name']}(으)로 이동 중입니다.")
                return True
            
            # 이동 중 다른 호출벨 무시 설정 확인
            if hasattr(self, 'ignore_other_bell') and self.ignore_other_bell:
                # 무시 모드가 켜져 있는 경우: 다른 목적지 호출벨 무시
                print(f"이동 중 다른 목적지 호출 무시 (무시 모드 ON): {new_location['name']} (UUID: {self.uuid_value})")
                ToastManager.instance().show_toast(f"이동 중입니다. 호출 무시: {new_location['name']}")
                return True
            else:
                # 무시 모드가 꺼져 있는 경우: 경로 변경하여 새 목적지로 이동
                print(f"이동 중 새 목적지로 경로 변경 (무시 모드 OFF): {new_location['name']} (UUID: {self.uuid_value})")
                ToastManager.instance().show_toast(f"새 목적지로 경로 변경: {new_location['name']}")
                # 직접 MovingController의 change_destination 메서드 호출
                if MAIN_CONTROLLER and hasattr(MAIN_CONTROLLER, "moving_controller"):
                    # 비동기적으로 호출 (UI 스레드 관련 문제 방지)
                    QTimer.singleShot(10, lambda: MAIN_CONTROLLER.moving_controller.change_destination(new_location))
                    return True
                return self._move_to_location(new_location)
        
        # 도착 화면인지 확인
        if self._is_arrive_view():
            print(f"현재 도착 화면입니다.")
            
            # 새로운 호출벨 위치가 등록된 목적지인지 확인
            new_location = self._get_location_by_uuid(self.uuid_value)
            
            # 새로운 목적지가 없으면 무시
            if not new_location:
                print(f"UUID {self.uuid_value}는 등록되지 않은 호출벨입니다. 무시합니다.")
                return True
            
            # 현재 도착한 목적지 정보 가져오기
            if MAIN_CONTROLLER and hasattr(MAIN_CONTROLLER, "arrive_controller"):
                current_position = MAIN_CONTROLLER.arrive_controller.get_current_position()
                if current_position and current_position.get('unique_id') == new_location.get('unique_id'):
                    print(f"현재 도착한 목적지와 동일한 호출벨입니다. UUID {self.uuid_value} 이벤트 무시.")
                    ToastManager.instance().show_toast(f"이미 {new_location['name']}에 있습니다.")
                    return True
            
            # 다른 목적지 호출벨인 경우 새로운 목적지로 이동
            print(f"새로운 목적지 호출: {new_location['name']} (UUID: {self.uuid_value})")
            return self._move_to_location(new_location)
        
        # UUID가 등록된 목적지인지 확인
        matching_location = self._get_location_by_uuid(self.uuid_value)
        
        if matching_location:
            # 목적지 추가 화면에서는 처리 변경
            if is_add_view:
                # MessageBox를 표시하기 위해 LocationAddView로 UUID 전달 (직접 처리)
                print(f"이미 등록된 호출벨: {matching_location['name']} (UUID: {self.uuid_value})")
                # 직접 LocationAddView의 메서드 호출
                self._try_direct_set_uuid(self.uuid_value)
                return True
            else:
                # 다른 화면에서는 이동 처리
                return self._move_to_location(matching_location)
        else:
            print(f"UUID {self.uuid_value}는 등록되지 않은 새로운 호출벨입니다.")
            
            # 목적지 추가 화면일 때는 임시 변수에만 저장
            if is_add_view:
                # 등록되지 않은 UUID이므로 임시 변수에만 저장
                self.temp_uuid = self.uuid_value
                # LocationAddView의 버튼 상태 업데이트
                self._try_direct_set_uuid(self.uuid_value)
                return True
            
        return False