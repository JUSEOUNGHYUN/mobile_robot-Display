import os

class UIPathHelper:
    """UI 파일 경로를 관리하는 유틸리티 클래스"""
    
    @staticmethod
    def get_ui_path(ui_filename):
        """UI 파일의 전체 경로 반환
        
        Args:
            ui_filename (str): UI 파일 이름 (예: 'LocationAddView.ui')
            
        Returns:
            str: UI 파일의 전체 경로
        """
        # PROJECT_ROOT 환경 변수가 설정되어 있으면 사용
        project_root = os.environ.get('PROJECT_ROOT')
        if project_root:
            return os.path.join(project_root, 'UI', ui_filename)
        # 환경 변수가 없으면 상대 경로 사용
        return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'UI', ui_filename) 