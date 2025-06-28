from PySide6.QtCore import QObject, QTimer, QSize
from views.WiFiView import WiFiView, WiFiNetworkItem
from models.WiFiModel import WiFiModel
from PySide6.QtWidgets import QWidget, QLabel, QListWidgetItem

class WiFiController(QObject):
    """와이파이 설정 컨트롤러

    와이파이 설정 화면과 관련된 로직을 처리합니다.
    """

    def __init__(self, stack, main_controller):
        super().__init__()
        self.stack = stack
        self.main_controller = main_controller

        # 모델 초기화
        self.model = WiFiModel()

        # 뷰 초기화
        self.view = WiFiView()

        # 스택에 추가
        self.stack.addWidget(self.view)

        # 시그널 연결
        self._connect_signals()

        # 스캔 타이머
        self.scan_timer = QTimer()
        self.scan_timer.setInterval(10000)  # 10초마다 스캔
        self.scan_timer.timeout.connect(self.scan_networks)

    def _connect_signals(self):
        """시그널 연결"""
        # 뒤로가기 시그널 연결
        self.view.back_signal.connect(self.on_back)

        # 와이파이 관련 시그널 연결
        self.view.scan_signal.connect(self.scan_networks)
        self.view.connect_signal.connect(self.connect_to_network)
        self.view.disconnect_signal.connect(self.disconnect_from_network)
        self.view.toggle_wifi_signal.connect(self.toggle_wifi)
    
    def show(self):
        """와이파이 설정 화면 표시"""
        # 현재 화면을 이전 화면으로 저장
        self.main_controller.set_previous_view(self.stack.currentWidget())
        
        # 화면 표시
        self.stack.setCurrentWidget(self.view)
        
        # 초기 스캔 수행
        self.scan_networks()
        
        # 스캔 타이머 시작
        self.scan_timer.start()
        
        # 뷰의 WiFi 상태 업데이트
        self.update_wifi_enabled(self.model.is_wifi_enabled())

    def on_back(self):
        """뒤로가기 버튼 클릭 처리"""
        print("[WiFiController] 뒤로가기 버튼 클릭됨")

        # 스캔 타이머 중지
        self.scan_timer.stop()

        # 이전 화면으로 돌아가기
        self.main_controller.show_previous_view()

    def scan_networks(self):
        """와이파이 네트워크 스캔"""
        print("[WiFiController] 와이파이 네트워크 스캔 중...")

        # 모델에서 네트워크 스캔
        networks = self.model.scan_networks()

        # 뷰 업데이트
        self.update_networks(networks)
    
    def connect_to_network(self, ssid, password):
        """와이파이 네트워크 연결
        
        Args:
            ssid (str): 연결할 네트워크 SSID
            password (str): 네트워크 비밀번호
        """
        print(f"[WiFiController] 네트워크 '{ssid}'에 연결 시도 중...")
        
        # 모델을 통해 네트워크 연결
        success = self.model.connect_to_network(ssid, password)
        
        if success:
            print(f"[WiFiController] 네트워크 '{ssid}'에 연결 성공")
        else:
            print(f"[WiFiController] 네트워크 '{ssid}'에 연결 실패")
        
        # 네트워크 목록 갱신
        networks = self.model.get_networks()
        self.update_networks(networks)
    
    def disconnect_from_network(self, ssid):
        """와이파이 네트워크 연결 해제
        
        Args:
            ssid (str): 연결 해제할 네트워크 SSID
        """
        print(f"[WiFiController] 네트워크 '{ssid}'에서 연결 해제 중...")
        
        # 모델을 통해 연결 해제
        success = self.model.disconnect()
        
        if success:
            print(f"[WiFiController] 네트워크 '{ssid}'에서 연결 해제 성공")
        else:
            print(f"[WiFiController] 네트워크 '{ssid}'에서 연결 해제 실패")
        
        # 네트워크 목록 갱신
        networks = self.model.get_networks()
        self.update_networks(networks)
    
    def toggle_wifi(self, enabled):
        """와이파이 활성화/비활성화
        
        Args:
            enabled (bool): 와이파이 활성화 여부
        """
        # 모델을 통해 와이파이 상태 변경
        success = self.model.set_wifi_enabled(enabled)
        
        if success:
            print(f"[WiFiController] 와이파이 {'활성화' if enabled else '비활성화'} 성공")
            
            # 뷰의 와이파이 상태 업데이트
            self.update_wifi_enabled(enabled)
            
            if enabled:
                # 와이파이 활성화 후 스캔 및 타이머 시작
                self.scan_networks()
                self.scan_timer.start()
            else:
                # 와이파이 비활성화 시 타이머 중지 및 네트워크 목록 비우기
                self.scan_timer.stop()
                self.update_networks([])
        else:
            print(f"[WiFiController] 와이파이 {'활성화' if enabled else '비활성화'} 실패")

    def update_wifi_enabled(self, enabled):
        """뷰의 와이파이 상태 업데이트

        Args:
            enabled (bool): 와이파이 활성화 여부
        """
        # 뷰의 와이파이 상태 저장
        self.view.wifi_enabled = enabled

        # 상태바의 와이파이 아이콘 업데이트
        if hasattr(self.view, 'status_bar'):
            self.view.status_bar.update_wifi_status(enabled)

        # 버튼 스타일 업데이트
        self._update_switch_style(enabled)

        # 설명 라벨 업데이트
        if hasattr(self.view.ui, 'description_label'):
            self.view.ui.description_label.setText(
                "사용 가능한 네트워크" if enabled
                else "Wi-Fi를 켜서 사용 가능한 네트워크를 확인하세요"
            )

        # 스캔 버튼과 네트워크 목록 활성화/비활성화
        self.view.ui.scan_button.setEnabled(enabled)
        self.view.ui.network_list.setEnabled(enabled)

        # 와이파이가 꺼져있을 때 네트워크 목록 비우기
        if not enabled:
            self.update_networks([])

    def _update_switch_style(self, enabled):
        """와이파이 스위치 버튼 스타일 업데이트"""
        # 버튼 텍스트만 변경 (스타일은 UI 파일에서 정의)
        self.view.ui.wifi_switch_button.setText("ON" if enabled else "OFF")

        # 버튼의 클래스 이름 변경 (CSS에서 선택자로 사용 가능)
        if enabled:
            self.view.ui.wifi_switch_button.setProperty("state", "on")
        else:
            self.view.ui.wifi_switch_button.setProperty("state", "off")

        # 스타일시트를 다시 적용하기 위해 필요
        self.view.ui.wifi_switch_button.style().unpolish(self.view.ui.wifi_switch_button)
        self.view.ui.wifi_switch_button.style().polish(self.view.ui.wifi_switch_button)

    def update_networks(self, networks):
        """네트워크 목록 업데이트"""
        self.view.ui.network_list.clear()

        if not networks:
            return

        for network in networks:
            item = QListWidgetItem(self.view.ui.network_list)
            network_widget = WiFiNetworkItem(network)
            network_widget.clicked.connect(self.view.on_network_clicked)

            # 아이템 너비를 충분히 확보하여 텍스트가 짤리지 않게 함
            item.setSizeHint(QSize(self.view.ui.network_list.width() - 30, network_widget.sizeHint().height()))
            self.view.ui.network_list.addItem(item)
            self.view.ui.network_list.setItemWidget(item, network_widget)
