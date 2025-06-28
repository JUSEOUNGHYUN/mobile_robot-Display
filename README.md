# TFRobot
<<<<<<< HEAD
temp
=======
<<<<<<< HEAD

## 호출벨 심볼릭 링크 설정 안내

TFRobot 애플리케이션은 호출벨 장치와 통신하기 위해 시리얼 포트를 사용합니다. 호출벨 연결의 안정성과 편의성을 높이기 위해 심볼릭 링크를 사용할 수 있습니다.

### 심볼릭 링크란?

심볼릭 링크는 특정 파일이나 장치에 대한 참조(바로가기)를 제공합니다. 호출벨의 경우, USB 포트에 연결될 때마다 다른 장치 이름(`/dev/ttyUSB0` 등)을 가질 수 있지만, 심볼릭 링크를 사용하면 항상 동일한 이름(`/dev/ttyCallBell`)으로 접근할 수 있습니다.

> 참고: 심볼릭 링크 경로는 `config/SerialConfig.py` 파일의 `CALLBELL_SYMLINK` 상수에 정의되어 있습니다.

### 심볼릭 링크 설정 방법

#### Linux 환경

1. 제공된 설정 스크립트 실행:
   ```bash
   sudo ./create_symlink.sh
   ```
   
2. 스크립트 실행이 어려운 경우 수동으로 설정:
   ```bash
   # 먼저 호출벨이 연결된 포트 확인 (예: /dev/ttyUSB0)
   ls -l /dev/ttyUSB*
   
   # 심볼릭 링크 생성
   sudo ln -sf /dev/ttyUSB0 /dev/ttyCallBell
   
   # 권한 설정
   sudo chmod 666 /dev/ttyCallBell
   ```

3. 영구적인 설정을 위한 udev 규칙 생성:
   ```bash
   # 호출벨 장치의 벤더 ID와 제품 ID 확인
   udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep -E 'ATTRS{idVendor}|ATTRS{idProduct}'
   
   # udev 규칙 파일 생성
   echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY", SYMLINK+="ttyCallBell"' | sudo tee /etc/udev/rules.d/99-callbell.rules
   
   # udev 규칙 재로드
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

### 자동 감지 시스템

TFRobot 애플리케이션은 다음과 같은 순서로 호출벨 장치를 자동으로 감지합니다:

1. 심볼릭 링크(`/dev/ttyCallBell`) 확인 및 연결 시도
2. 시스템에서 감지된 모든 시리얼 포트 순차적 연결 시도
3. 연결 성공 시 해당 포트 사용

심볼릭 링크가 설정되어 있지 않아도 프로그램은 동작하지만, 설정해두면 더 안정적인 연결을 유지할 수 있습니다.

### 문제 해결

- **장치를 찾을 수 없는 경우**: 호출벨 USB 장치가 제대로 연결되어 있는지 확인하세요.
- **권한 오류**: `sudo chmod 666 /dev/ttyUSB*` 명령으로 권한을 설정해 보세요.

자세한 내용은 개발자에게 문의하세요.

=======
temp
>>>>>>> 9d309a5017719a736ab73bbe147b0ddad8672e87
>>>>>>> 2d7540b (Initial commit)
