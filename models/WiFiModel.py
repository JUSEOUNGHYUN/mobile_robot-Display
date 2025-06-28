import subprocess
import re
import json

class WiFiModel:

    """
    1. WiFi 인터페이스 관리
      시스템에서 WiFi 인터페이스(wlan0 등) 자동 감지
      WiFi 활성화/비활성화 제어

    2. 네트워크 스캔 및 정보 추출
      주변 WiFi 네트워크 스캔
      신호 강도, 채널, 보안 유형 등 상세 정보 파싱
      신호 강도에 따른 시각적 표시(바) 생성

    3. 연결 관리
      선택한 네트워크에 연결/연결 해제
      비밀번호 관리 및 연결 상태 추적
      기존 저장된 프로필 활용
    """

    def __init__(self):
        """초기화"""
        self.wifi_enabled = True
        self.connected_ssid = None
        self.networks = []
        self.interface = self._get_wifi_interface()

    def _get_wifi_interface(self):
        """WiFi 인터페이스(wlan0 등) 가져오기"""
        try:
            result = subprocess.run(
                ['nmcli', 'device', 'status'],
                capture_output=True, text=True, check=True
            )

            # 출력에서 wifi 인터페이스 찾기
            for line in result.stdout.split('\n'):
                if 'wifi' in line:
                    parts = line.split()
                    if len(parts) > 0:
                        return parts[0]  # 첫 번째 칼럼이 인터페이스 이름

            # 기본값
            return "wlan0"
        except Exception as e:
            print(f"[WiFiModel] WiFi 인터페이스를 가져오는 중 오류 발생: {e}")
            return "wlan0"

    def is_wifi_enabled(self):
        """와이파이 사용 가능 여부 확인"""
        try:
            result = subprocess.run(
                ['nmcli', 'radio', 'wifi'],
                capture_output=True, text=True, check=True
            )

            # 'enabled' 또는 'disabled' 반환
            self.wifi_enabled = 'enabled' in result.stdout.strip().lower()
            return self.wifi_enabled
        except Exception as e:
            print(f"[WiFiModel] WiFi 상태 확인 중 오류 발생: {e}")
            return self.wifi_enabled  # 현재 상태 반환

    def set_wifi_enabled(self, enabled):
        try:
            state = 'on' if enabled else 'off'
            subprocess.run(
                ['nmcli', 'radio', 'wifi', state],
                capture_output=True, text=True, check=True
            )

            # 상태 업데이트
            self.wifi_enabled = enabled
            return True
        except Exception as e:
            print(f"[WiFiModel] WiFi {state} 설정 중 오류 발생: {e}")
            return False

    def scan_networks(self):
        if not self.is_wifi_enabled():
            print("[WiFiModel] WiFi가 비활성화되어 있어 스캔할 수 없습니다.")
            return []
        try:
            # 네트워크 스캔 (rescan yes 옵션으로 강제 스캔)
            subprocess.run(
                ['nmcli', 'device', 'wifi', 'rescan'],
                capture_output=True, text=True
            )

            # 스캔 결과 가져오기
            result = subprocess.run(
                ['nmcli', '-f', 'SSID,BSSID,SIGNAL,SECURITY,CHAN,RATE,IN-USE', 
                 '-t', 'device', 'wifi', 'list'],
                capture_output=True, text=True, check=True
            )

            # 현재 연결된 네트워크 확인
            connected_result = subprocess.run(
                ['nmcli', '-t', 'connection', 'show', '--active'],
                capture_output=True, text=True
            )

            # 연결된 SSID 찾기
            self.connected_ssid = None
            for line in connected_result.stdout.split('\n'):
                if line and 'wifi' in line:
                    parts = line.split(':')
                    if len(parts) > 0:
                        self.connected_ssid = parts[0]
                        break

            # 결과 파싱
            networks = []
            for line in result.stdout.split('\n'):
                if not line:
                    continue

                parts = line.split(':')
                if len(parts) >= 7:
                    ssid = parts[0]
                    bssid = parts[1]

                    # 신호 강도 파싱 및 예외 처리
                    try:
                        # 가끔 신호 강도가 숫자가 아닌 형태로 반환될 수 있음
                        signal_str = parts[2].strip()
                        # 숫자만 추출 (예: 'F2\' -> 0, '75' -> 75)
                        signal_digits = ''.join(c for c in signal_str if c.isdigit())
                        signal = int(signal_digits) if signal_digits else 0
                    except Exception as e:
                        print(f"[WiFiModel] 신호 강도 파싱 오류: {e}, 기본값 0 사용")
                        signal = 0

                    # 나머지 정보 파싱
                    security = parts[3]

                    # 채널 파싱 및 예외 처리
                    try:
                        channel_str = parts[4].strip()
                        channel = int(channel_str) if channel_str.isdigit() else 0
                    except Exception:
                        channel = 0

                    rate = parts[5]
                    in_use = parts[6] == '*'

                    # 신호 강도에 따른 바 표시
                    bars = ""
                    if signal >= 80:
                        bars = "▂▄▆█"
                    elif signal >= 60:
                        bars = "▂▄▆_"
                    elif signal >= 40:
                        bars = "▂▄__"
                    elif signal >= 20:
                        bars = "▂___"
                    else:
                        bars = "____"

                    # SSID가 빈 문자열인 경우 (숨겨진 네트워크)
                    if not ssid:
                        ssid = f"숨겨진 네트워크 ({bssid})"

                    network = {
                        "ssid": ssid,
                        "bssid": bssid,
                        "strength": signal,
                        "secured": security != "",
                        "connected": in_use or ssid == self.connected_ssid,
                        "channel": channel,
                        "rate": rate,
                        "bars": bars,
                        "security": security
                    }

                    networks.append(network)

            # 신호 강도순으로 정렬
            networks.sort(key=lambda x: x["strength"], reverse=True)

            # 연결된 네트워크가 있으면 맨 위로
            connected = [n for n in networks if n["connected"]]
            not_connected = [n for n in networks if not n["connected"]]
            networks = connected + not_connected

            self.networks = networks
            return networks

        except Exception as e:
            print(f"[WiFiModel] WiFi 스캔 중 오류 발생: {e}")
            return self.networks

    def connect_to_network(self, ssid, password=None):
        if not self.is_wifi_enabled():
            return False

        try:
            # 먼저 해당 SSID에 대한 연결 프로필이 있는지 확인
            check_result = subprocess.run(
                ['nmcli', '-t', 'connection', 'show'],
                capture_output=True, text=True, check=True
            )

            connection_exists = False
            for line in check_result.stdout.split('\n'):
                if line and ssid in line:
                    connection_exists = True
                    break

            if connection_exists:
                # 기존 연결 사용
                result = subprocess.run(
                    ['nmcli', 'connection', 'up', ssid],
                    capture_output=True, text=True
                )
            else:
                # 새 연결 생성
                if password:
                    result = subprocess.run(
                        ['nmcli', 'device', 'wifi', 'connect', ssid, 'password', password],
                        capture_output=True, text=True
                    )
                else:
                    result = subprocess.run(
                        ['nmcli', 'device', 'wifi', 'connect', ssid],
                        capture_output=True, text=True
                    )

            # 연결 성공 확인
            success = result.returncode == 0

            if success:
                self.connected_ssid = ssid
                print(f"[WiFiModel] '{ssid}'에 연결되었습니다.")
            else:
                print(f"[WiFiModel] '{ssid}'에 연결 실패: {result.stderr}")

            return success

        except Exception as e:
            print(f"[WiFiModel] '{ssid}'에 연결 시도 중 오류 발생: {e}")
            return False

    def disconnect(self):
        if not self.connected_ssid or not self.interface:
            print("[WiFiModel] 연결된 네트워크가 없거나 인터페이스를 찾을 수 없습니다.")
            return False

        try:
            result = subprocess.run(
                ['nmcli', 'device', 'disconnect', self.interface],
                capture_output=True, text=True, check=True
            )

            self.connected_ssid = None
            return True

        except Exception as e:
            return False

    def get_networks(self):
        return self.networks