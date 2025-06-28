#!/bin/bash

# 로봇 앱 프로세스 찾기
echo "TF로봇 프로세스 찾는 중..."
PID=$(pgrep -f "python3 main.py" || pgrep -f "python main.py")

if [ -z "$PID" ]; then
    echo "실행 중인 TF로봇 프로세스를 찾을 수 없습니다."
    exit 1
fi

echo "TF로봇 프로세스 발견: $PID"

# 정상 종료 시도 (SIGTERM)
echo "정상 종료 신호(SIGTERM) 보내는 중..."
kill -15 $PID

# 8초 기다리기 (ROS2 종료에 좀 더 시간 부여)
echo "프로세스 종료 대기 중 (8초)..."
sleep 8

# 프로세스가 아직 살아있는지 확인
if ps -p $PID > /dev/null; then
    echo "정상 종료 실패. 강제 종료 시도 중..."
    kill -9 $PID
    echo "강제 종료 완료."
else
    echo "프로세스가 정상적으로 종료되었습니다."
fi

# ROS2 관련 프로세스 확인
ROS_PROCESSES=$(ps -ef | grep -E "ros|rclpy" | grep -v grep)
if [ -n "$ROS_PROCESSES" ]; then
    echo "남아있는 ROS2 관련 프로세스 목록:"
    echo "$ROS_PROCESSES"
    
    read -p "남아있는 ROS2 프로세스를 종료하시겠습니까? (y/N): " choice
    if [ "$choice" = "y" ] || [ "$choice" = "Y" ]; then
        pkill -9 -f "ros"
        pkill -9 -f "rclpy"
        echo "ROS2 관련 프로세스 종료 완료."
    fi
fi

# 남아있는 파이썬 프로세스 확인
REMAINING=$(ps -ef | grep python | grep -v grep | grep -v "clean_exit.sh")
if [ -n "$REMAINING" ]; then
    echo "남아있는 파이썬 프로세스 목록:"
    echo "$REMAINING"
    
    # 선택적으로 남아있는 모든 프로세스 종료 (주의: 다른 Python 앱도 종료될 수 있음)
    read -p "남아있는 모든 파이썬 프로세스를 종료하시겠습니까? (y/N): " choice
    if [ "$choice" = "y" ] || [ "$choice" = "Y" ]; then
        pkill -9 -f python
        echo "모든 파이썬 프로세스 종료 완료."
    fi
else
    echo "남아있는 파이썬 프로세스가 없습니다."
fi 