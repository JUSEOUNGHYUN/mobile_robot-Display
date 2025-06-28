#!/bin/bash

# 호출벨 장치 심볼릭 링크 생성 스크립트
# 실행 방법: sudo ./create_symlink.sh

# 색상 설정
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 루트 권한 확인
if [ "$EUID" -ne 0 ]; then
  echo -e "${RED}이 스크립트는 루트 권한으로 실행해야 합니다. 'sudo ./create_symlink.sh' 명령을 사용하세요.${NC}"
  exit 1
fi

echo -e "${BLUE}=========================================${NC}"
echo -e "${BLUE}    TFRobot 호출벨 심볼릭 링크 설정     ${NC}"
echo -e "${BLUE}=========================================${NC}"
echo ""

# 기존 심볼릭 링크 확인
SYMLINK_PATH="/dev/ttyCallBell"
if [ -L "$SYMLINK_PATH" ]; then
  TARGET=$(readlink -f "$SYMLINK_PATH")
  echo -e "${YELLOW}기존 심볼릭 링크가 발견되었습니다: $SYMLINK_PATH -> $TARGET${NC}"
  
  # 기존 링크가 유효한지 확인
  if [ -e "$TARGET" ]; then
    echo -e "${GREEN}기존 링크가 유효합니다.${NC}"
    
    read -p "기존 링크를 유지하시겠습니까? (y/n): " choice
    if [[ $choice =~ ^[Yy]$ ]]; then
      echo -e "${GREEN}기존 링크를 유지합니다. 설정이 완료되었습니다.${NC}"
      exit 0
    else
      echo -e "${YELLOW}기존 링크를 제거하고 새로 설정합니다...${NC}"
      rm "$SYMLINK_PATH"
    fi
  else
    echo -e "${RED}기존 링크가 유효하지 않습니다. 제거 후 새로 설정합니다...${NC}"
    rm "$SYMLINK_PATH"
  fi
fi

# 연결된 시리얼 포트 목록 가져오기
echo -e "${BLUE}사용 가능한 시리얼 포트 검색 중...${NC}"
available_ports=()

# Linux 환경에서 일반적인 시리얼 포트 패턴
for pattern in "/dev/ttyUSB"* "/dev/ttyACM"* "/dev/ttyS"*; do
  if [ -e "$pattern" ]; then
    available_ports+=("$pattern")
  fi
done

# 포트가 없을 경우 종료
if [ ${#available_ports[@]} -eq 0 ]; then
  echo -e "${RED}연결된 시리얼 포트가 없습니다. 장치가 연결되어 있는지 확인하세요.${NC}"
  exit 1
fi

# 포트 목록 출력 및 선택
echo -e "${BLUE}사용 가능한 시리얼 포트:${NC}"
for i in "${!available_ports[@]}"; do
  echo -e "${GREEN}$((i+1)). ${available_ports[$i]}${NC}"
done

echo ""
# 자동 검색 또는 수동 선택 옵션
echo -e "${YELLOW}옵션을 선택하세요:${NC}"
echo "1. 자동으로 호출벨 포트 검색 (권장)"
echo "2. 수동으로 포트 선택"
read -p "선택 (1 또는 2): " auto_choice

if [ "$auto_choice" == "1" ]; then
  # 자동 검색 로직 
  # 여기서는 첫 번째 포트를 선택하는 단순한 로직 사용
  # 실제로는 장치 ID나 통신 패턴을 확인하는 더 복잡한 로직이 필요할 수 있음
  selected_port="${available_ports[0]}"
  echo -e "${YELLOW}자동으로 ${selected_port}를 선택했습니다.${NC}"
else
  # 수동 선택
  read -p "포트 번호를 선택하세요 (1-${#available_ports[@]}): " choice
  
  # 입력 검증
  if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt ${#available_ports[@]} ]; then
    echo -e "${RED}잘못된 선택입니다.${NC}"
    exit 1
  fi
  
  selected_port="${available_ports[$((choice-1))]}"
fi

# 심볼릭 링크 생성
echo -e "${BLUE}심볼릭 링크 생성 중: $SYMLINK_PATH -> $selected_port${NC}"
ln -sf "$selected_port" "$SYMLINK_PATH"

# 권한 설정
echo -e "${BLUE}장치 권한 설정 중...${NC}"
chmod 666 "$selected_port"
chmod 666 "$SYMLINK_PATH"

# 결과 확인
if [ -L "$SYMLINK_PATH" ] && [ -e "$SYMLINK_PATH" ]; then
  echo -e "${GREEN}심볼릭 링크가 성공적으로 생성되었습니다!${NC}"
  echo -e "${GREEN}$SYMLINK_PATH -> $(readlink -f $SYMLINK_PATH)${NC}"
  echo ""
  echo -e "${YELLOW}이제 TFRobot 애플리케이션에서 호출벨 기능을 사용할 수 있습니다.${NC}"
else
  echo -e "${RED}심볼릭 링크 생성에 실패했습니다.${NC}"
  exit 1
fi

# udev 규칙 설정 제안
echo ""
echo -e "${BLUE}영구적인 설정을 위한 udev 규칙 생성 안내:${NC}"
echo -e "다음 명령으로 호출벨이 항상 같은 이름으로 인식되도록 할 수 있습니다:"
echo -e "${YELLOW}1. 호출벨 장치의 ID 확인:${NC}"
echo "   udevadm info --name=$selected_port --attribute-walk | grep -E 'ATTRS{idVendor}|ATTRS{idProduct}'"
echo -e "${YELLOW}2. /etc/udev/rules.d/99-callbell.rules 파일 생성:${NC}"
echo "   echo 'SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"XXXX\", ATTRS{idProduct}==\"YYYY\", SYMLINK+=\"ttyCallBell\"' | sudo tee /etc/udev/rules.d/99-callbell.rules"
echo -e "${YELLOW}3. udev 규칙 재로드:${NC}"
echo "   sudo udevadm control --reload-rules && sudo udevadm trigger"
echo ""
echo -e "${GREEN}설정이 완료되었습니다!${NC}" 