from PySide6.QtCore import QObject
from views.InfoView import InfoView
from models.SystemInfoModel import SystemInfoModel

class InfoController(QObject):
    """정보 화면 컨트롤러

    정보 화면과 관련된 로직을 처리합니다.
    """

    def __init__(self, stack, main_controller):
        super().__init__()
        self.stack = stack
        self.main_controller = main_controller

        # 모델 초기화
        self.model = SystemInfoModel()

        # 뷰 초기화
        self.view = InfoView()

        # 스택에 추가
        self.stack.addWidget(self.view)

        # 시그널 연결
        self._connect_signals()

        # 초기 데이터 로드
        self._load_initial_data()

    def _connect_signals(self):
        """시그널 연결"""
        # 뒤로가기 시그널 연결
        self.view.back_signal.connect(self.on_back)

        # 심볼릭 링크 확인 시그널 연결
        self.view.check_symlinks_signal.connect(self.on_check_symlinks)

    def _load_initial_data(self):
        """초기 데이터 로드 및 뷰 업데이트"""
        # 버전 정보 설정
        self.view.set_version_info(self.model.get_version_info())

        # 기술 스택 설정
        self.view.set_tech_stack(self.model.get_tech_stack())

        # 개발자 정보 설정
        self.view.set_developer_info(self.model.get_developer_info())

    def show(self):
        """정보 화면 표시"""
        # 현재 화면을 이전 화면으로 저장
        self.main_controller.set_previous_view(self.stack.currentWidget())

        # 정보 화면으로 전환
        self.stack.setCurrentWidget(self.view)

    def on_back(self):
        """뒤로가기 버튼 클릭 처리"""
        print("[InfoController] 뒤로가기 버튼 클릭됨")
        # 이전 화면으로 돌아가기
        self.main_controller.show_previous_view()

    def on_check_symlinks(self):
        """심볼릭 링크 확인 버튼 클릭 처리"""
        # 모델에서 심볼릭 링크 정보 가져오기
        symlink_info = self.model.check_symlinks()

        # 뷰 업데이트
        self.view.update_symlink_info(symlink_info)