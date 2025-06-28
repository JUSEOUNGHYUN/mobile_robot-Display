from PySide6.QtCore import QObject, Signal, QPointF
import os
import json

class MapImageControlModel(QObject):
    """맵 이미지 컨트롤 모델
    
    맵 이미지 변환 데이터를 관리하는 모델 클래스입니다.
    """
    # 시그널 정의
    data_loaded = Signal(dict)  # 데이터 로드 시그널
    data_saved = Signal(bool, str)  # 저장 결과 시그널 (성공 여부, 메시지)
    
    def __init__(self):
        super().__init__()
        
        # 맵 이미지 변환 데이터 초기화
        self.transform_data = {
            'translate_x': 0,
            'translate_y': 0,
            'scale': 1.0,
            'rotation': 0
        }
        
        # 맵 이미지 변환 정보 파일 경로
        self.transform_file = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                          "data", "map_transform.json")
        
        # 맵 이미지 파일 경로
        self.map_image_path = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                                         "resources", "map_original_image.png")
    
    def load_transform_data(self):
        """변환 데이터 로드"""
        try:
            # resources 디렉토리가 없으면 생성
            resources_dir = os.path.dirname(self.transform_file)
            if not os.path.exists(resources_dir):
                os.makedirs(resources_dir)
                print(f"리소스 디렉토리 생성됨: {resources_dir}")
            
            # 변환 데이터 파일이 있으면 로드
            if os.path.exists(self.transform_file):
                with open(self.transform_file, 'r', encoding='utf-8') as f:
                    self.transform_data = json.load(f)
                print(f"맵 변환 정보 로드됨: {self.transform_data}")
            else:
                print(f"맵 변환 정보 파일이 없음, 기본값 사용: {self.transform_data}")
            
            # 데이터 로드 시그널 발생
            self.data_loaded.emit(self.transform_data)
            return self.transform_data
            
        except Exception as e:
            print(f"맵 변환 정보 로드 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
            # 기본 데이터 반환
            self.data_loaded.emit(self.transform_data)
            return self.transform_data
    
    def save_transform_data(self, transform_data):
        """변환 데이터 저장"""
        try:
            # resources 디렉토리가 없으면 생성
            resources_dir = os.path.dirname(self.transform_file)
            if not os.path.exists(resources_dir):
                os.makedirs(resources_dir)
                print(f"리소스 디렉토리 생성됨: {resources_dir}")
            
            # 데이터 유효성 검사
            if not isinstance(transform_data, dict):
                self.data_saved.emit(False, "변환 데이터 형식이 잘못되었습니다.")
                return False
            
            # 변환 데이터 저장
            with open(self.transform_file, 'w', encoding='utf-8') as f:
                json.dump(transform_data, f, ensure_ascii=False, indent=2)
            
            # 내부 데이터 업데이트
            self.transform_data = transform_data
            
            print(f"맵 변환 정보 저장됨: {transform_data}")
            self.data_saved.emit(True, "맵 이미지 변환 정보가 저장되었습니다.")
            return True
            
        except Exception as e:
            print(f"맵 변환 정보 저장 중 오류 발생: {str(e)}")
            import traceback
            traceback.print_exc()
            self.data_saved.emit(False, f"저장 중 오류 발생: {str(e)}")
            return False
    
    def get_map_image_path(self):
        """맵 이미지 파일 경로 반환"""
        return self.map_image_path
    
    def reset_transform_data(self):
        """변환 데이터 초기화"""
        self.transform_data = {
            'translate_x': 0,
            'translate_y': 0,
            'scale': 1.0,
            'rotation': 0
        }
        self.data_loaded.emit(self.transform_data)
        return self.transform_data 