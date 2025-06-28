#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import traceback
import signal
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication
from controllers.MainController import MainController
from models.RingoBellManager import RingoBellManager, set_main_controller  # 함수 추가 import
from Utils.RosTopic import KeepoutZonePublisher  # KeepoutZonePublisher 추가
import logging

# 로깅 설정
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
os.environ["QT_IM_MODULE"] = "qtvirtualkeyboard"  # 가상 키보드 모듈 설정

# 전역 변수로 컨트롤러 참조 저장
CONTROLLER = None

def cleanup_resources():
    """종료 시 모든 리소스 정리"""
    logging.debug("[시스템] 종료 전 리소스 정리 중...")
    
    try:
        # 링고벨 매니저 정리
        bell_manager = RingoBellManager.instance()
        if bell_manager:
            logging.debug("[시스템] 호출벨 매니저 정리 중...")
            bell_manager.close()
    except Exception as e:
        logging.error(f"[오류] 호출벨 매니저 정리 중 오류 발생: {str(e)}")
    
    try:
        # 메인 컨트롤러 정리
        global CONTROLLER
        if CONTROLLER:
            # 모든 컨트롤러 및 관련 리소스 정리
            if hasattr(CONTROLLER, 'moving_controller') and CONTROLLER.moving_controller:
                logging.debug("[시스템] 이동 컨트롤러 정리 중...")
                CONTROLLER.moving_controller.cleanup()
            
            # 필요한 다른 컨트롤러 정리 추가
    except Exception as e:
        logging.error(f"[오류] 컨트롤러 정리 중 오류 발생: {str(e)}")
        
    # ROS2 정리
    try:
        import rclpy
        if rclpy.ok():
            logging.debug("[시스템] ROS2 컨텍스트 종료 중...")
            try:
                rclpy.shutdown()
                logging.debug("[시스템] ROS2 종료 완료")
            except Exception as e:
                logging.error(f"[오류] ROS2 종료 중 오류 발생: {str(e)}")
    except ImportError:
        pass  # rclpy가 임포트되지 않았으면 무시
    except Exception as e:
        logging.error(f"[오류] ROS2 종료 확인 중 오류 발생: {str(e)}")
    
    logging.debug("[시스템] 리소스 정리 완료")

def signal_handler(signum, frame):
    """시그널 핸들러"""
    print("\n[시스템] 프로그램 종료 신호를 받았습니다.")
    # 리소스 정리
    cleanup_resources()
    # 애플리케이션 종료
    if 'app' in globals():
        app.quit()
    sys.exit(0)

def main():
    try:
        logging.info("[시스템] 애플리케이션 시작 중...")

        # 시그널 핸들러 등록
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)  # SIGTERM 시그널도 처리

        # 환경 설정
        logging.debug("[시스템] 환경 변수 설정 중...")
        # 아래 환경 변수를 주석 처리하여 커스텀 키보드를 사용하도록 설정
        # os.environ["QT_QPA_PLATFORM"] = "xcb"
        os.environ["DISPLAY"] = ":3"  # 필요에 따라 변경
        os.environ["XDG_RUNTIME_DIR"] = "/run/user/1000"
        os.environ["XAUTHORITY"] = os.path.expanduser("~/.Xauthority")

        # OpenGL 컨텍스트 공유 설정
        QApplication.setAttribute(Qt.AA_ShareOpenGLContexts)

        # 터치 스크린 설정
        QApplication.setAttribute(Qt.AA_SynthesizeTouchForUnhandledMouseEvents, True)

        logging.debug("[시스템] 애플리케이션 생성 시작...")
        # 애플리케이션 생성
        global app
        app = QApplication(sys.argv)
        
        # 애플리케이션 종료 시 리소스 정리
        app.aboutToQuit.connect(cleanup_resources)

        # 링고벨 매니저 초기화 및 연결 (싱글톤)
        bell_manager = RingoBellManager.instance()
        logging.debug("[시스템] 호출벨 매니저 초기화 완료")

        # MainController에 app 인스턴스 전달
        global CONTROLLER
        CONTROLLER = MainController(app)
        logging.debug("[시스템] 애플리케이션 생성 완료")

        # 호출벨 매니저 연결
        CONTROLLER.edit_controller.bell_manager = bell_manager
        logging.debug("[시스템] 호출벨 매니저 컨트롤러 연결 완료")

        # 메인 컨트롤러 참조 설정 (중요: 이 지점에서 전역 참조 설정)
        set_main_controller(CONTROLLER)
        logging.debug("[시스템] 호출벨 매니저에 메인 컨트롤러 참조 설정 완료")

        # 설정 아이콘 연결
        CONTROLLER.settings_controller.connect_setting_icons([
            CONTROLLER.selection_controller.selection_view,
            CONTROLLER.moving_controller.moving_view
        ])

        # 호출벨 포트가 아직 연결되지 않았으면 추가 연결 시도
        if not bell_manager.is_connected:
            logging.debug("[시스템] 호출벨 포트 연결 재시도...")
            bell_manager.auto_connect()

        logging.info("[시스템] 애플리케이션 실행 준비 완료")
        CONTROLLER.start()

        # 애플리케이션 실행
        sys.exit(app.exec())

    except Exception as e:
        logging.error(f"[오류] 애플리케이션 오류 발생: {str(e)}")
        logging.error(traceback.format_exc())

if __name__ == "__main__":
    main()