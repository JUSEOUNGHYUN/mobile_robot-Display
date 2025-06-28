import os
import json
from PySide6.QtCore import QObject, Signal
import uuid
from models.RingoBellManager import RingoBellManager




class LocationAddModel(QObject):
    # 위치 데이터 변경 시그널
    location_data_changed = Signal(dict)

    # 위치 저장 결과 시그널
    save_result = Signal(bool, str)  # 성공 여부, 메시지

    # 위치 데이터 업데이트 시그널 (x, y, yaw)
    position_data_updated = Signal(float, float, float)
    
    # 파일 변경 시그널
    file_updated = Signal(list)  # 업데이트된 전체 위치 목록

    def __init__(self, file_path='data/locations.txt'):
        super().__init__()
        # 파일 경로 설정
        self.file_path = file_path
        self._ensure_file_exists()

        # 현재 작업 중인 위치 데이터
        self._current_location_data = self._create_empty_location()
        self._edit_mode = False
        self._edit_id = None
        self._uuid_value = ""

    def _ensure_file_exists(self):
        """파일이 존재하지 않으면 기본 구조로 생성"""
        if not os.path.exists(os.path.dirname(self.file_path)):
            os.makedirs(os.path.dirname(self.file_path))

        if not os.path.exists(self.file_path):
            with open(self.file_path, 'w') as file:
                json.dump([], file)

    def _create_empty_location(self):
        """빈 위치 데이터 생성"""
        return {
            "name": "",
            "position": {
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0
            },
            "label": "No",
            "unique_id": ""  # 기본값은 빈 문자열
        }

    def read_locations(self):
        """모든 위치 목록 읽기"""
        # 먼저 RingoBellManager의 캐시 확인
        ring_manager = RingoBellManager.instance()
        cached_locations = ring_manager.get_cached_locations()
        
        # 캐시된 데이터가 있으면 사용
        if cached_locations:
            return cached_locations
            
        # 캐시가 없으면 파일에서 직접 읽기
        try:
            with open(self.file_path, 'r') as file:
                locations = json.load(file)
                return locations
        except (FileNotFoundError, json.JSONDecodeError):
            return []

    def load_location_for_edit(self, location_data, edit_id):
        """수정할 위치 데이터 로드"""
        self._current_location_data = location_data.copy() if location_data else self._create_empty_location()
        self._edit_mode = edit_id is not None
        self._edit_id = edit_id

        # 시그널 발생
        self.location_data_changed.emit(self._current_location_data)

        # 위치 데이터도 시그널로 발생
        if 'position' in self._current_location_data:
            position = self._current_location_data['position']
            try:
                x = float(position.get('x', 0.0))
                y = float(position.get('y', 0.0))
                yaw = float(position.get('yaw', 0.0))
                self.position_data_updated.emit(x, y, yaw)
            except (ValueError, TypeError):
                # 숫자로 변환할 수 없는 경우 기본값 사용
                self.position_data_updated.emit(0.0, 0.0, 0.0)

    def load_empty_location(self):
        """새 위치 추가용 빈 데이터 로드"""
        self._current_location_data = self._create_empty_location()
        self._edit_mode = False
        self._edit_id = None
        self._uuid_value = ""

        # 시그널 발생
        self.location_data_changed.emit(self._current_location_data)
        self.position_data_updated.emit(0.0, 0.0, 0.0)

    def update_position(self, x, y, yaw):
        """위치 정보 업데이트 (ROS에서 수신한 정보로)"""
        if 'position' not in self._current_location_data:
            self._current_location_data['position'] = {}

        self._current_location_data['position']['x'] = x
        self._current_location_data['position']['y'] = y
        self._current_location_data['position']['yaw'] = yaw

        # 시그널 발생
        self.position_data_updated.emit(x, y, yaw)

    def set_name(self, name):
        """위치 이름 설정"""
        self._current_location_data['name'] = name

    def set_uuid(self, uuid_value):
        """호출벨 UUID 설정"""
        print(f"UUID 설정: {uuid_value}")
        if uuid_value:
            self._uuid_value = uuid_value
            self._current_location_data['unique_id'] = uuid_value
            self._current_location_data['label'] = "Yes"
            print(f"현재 데이터: {self._current_location_data}")
        else:
            self._uuid_value = ""
            self._current_location_data['unique_id'] = ""
            self._current_location_data['label'] = "No"

    def save_location(self):
        """위치 저장"""
        print(f"LocationAddModel - 저장 시작: {self._current_location_data}")
        
        # 유효성 검사
        valid, error_message = self.validate_current_data()
        if not valid:
            print(f"LocationAddModel - 유효성 검사 실패: {error_message}")
            self.save_result.emit(False, error_message)
            return False
            
        # 파일에서 현재 목록 읽기
        locations = self.read_locations()
        
        # 중복 검사
        error_message = self.check_duplicate(self._current_location_data, locations, self._edit_id)
        if error_message:
            print(f"LocationAddModel - 중복 검사 실패: {error_message}")
            self.save_result.emit(False, error_message)
            return False
        
        # 저장 또는 업데이트
        if self._edit_mode and self._edit_id is not None and 0 <= self._edit_id < len(locations):
            # 기존 위치 수정
            locations[self._edit_id] = self._current_location_data
        else:
            # 새 위치 추가
            locations.append(self._current_location_data)
        
        # JSON 형식으로 저장
        try:
            with open(self.file_path, 'w') as file:
                json.dump(locations, file, indent=2)
                
            print(f"LocationAddModel - 저장 완료: {self._current_location_data}")
            
            # 파일 변경 시그널 발생
            self.file_updated.emit(locations)
            self.save_result.emit(True, "저장되었습니다.")
            return True
        except Exception as e:
            print(f"LocationAddModel - 저장 중 오류 발생: {str(e)}")
            self.save_result.emit(False, f"저장 중 오류 발생: {str(e)}")
            return False

    def is_edit_mode(self):
        """현재 편집 모드인지 확인"""
        return self._edit_mode

    def get_current_data(self):
        """현재 작업 중인 위치 데이터 반환"""
        return self._current_location_data.copy()

    def has_uuid(self):
        """UUID가 설정되어 있는지 확인"""
        return bool(self._uuid_value)

    def is_uuid_duplicate(self, uuid_str):
        """UUID 중복 여부 확인"""
        # 자신의 UUID면 중복 아님
        if self._edit_mode and self._current_location_data.get('unique_id') == uuid_str:
            return False
            
        # 메모리 캐시에서 UUID 확인
        return RingoBellManager.instance().is_uuid_registered(uuid_str)
        
    def find_location_by_uuid(self, uuid_str):
        """UUID로 위치 찾기
        Args:
            uuid_str (str): 찾을 UUID 문자열
        Returns:
            dict 또는 None: 일치하는 위치 데이터 또는 None
        """
        if not uuid_str:
            return None

        # RingoBellManager의 캐시된 데이터 활용
        ring_manager = RingoBellManager.instance()
        return ring_manager._get_location_by_uuid(uuid_str)

    def find_location_id(self, location_data):
        """위치 ID 찾기

        Args:
            location_data (dict): 찾을 위치 데이터

        Returns:
            int 또는 None: 일치하는 위치의 인덱스 또는 None
        """
        if not location_data:
            return None

        locations = self.read_locations()
        
        for i, loc in enumerate(locations):
            # 이름과 위치로 비교
            if (loc.get('name') == location_data.get('name') and 
                loc.get('position', {}).get('x') == location_data.get('position', {}).get('x') and
                loc.get('position', {}).get('y') == location_data.get('position', {}).get('y')):
                return i
        
        return None

    def validate_current_data(self):
        """현재 데이터 유효성 검사"""
        if not self._current_location_data.get('name'):
            return False, "이름은 필수 항목입니다."

        # 위치 정보 유효성 검사
        try:
            position = self._current_location_data.get('position', {})
            float(position.get('x', 0))
            float(position.get('y', 0))
            float(position.get('yaw', 0))
        except (ValueError, TypeError):
            return False, "좌표 값은 숫자여야 합니다."

        return True, None
        
    def check_duplicate(self, new_location, locations=None, edit_id=None):
        """중복 체크 로직"""
        # locations가 None이면 파일에서 읽기
        if locations is None:
            locations = self.read_locations()
            
        if not locations:
            return None
            
        # 위치와 이름 추출
        new_name = new_location.get('name')
        new_pos = new_location.get('position', {})
        new_x, new_y = new_pos.get('x'), new_pos.get('y')
        
        position_duplicate = False
        name_duplicate = False
        unique_id_duplicate = False
        
        # 호출벨 Unique ID가 있는 경우만 체크
        has_unique_id = 'unique_id' in new_location and new_location.get('label') == 'Yes'
        new_unique_id = new_location.get('unique_id', '') if has_unique_id else ''
        
        for i, loc in enumerate(locations):
            # 수정 모드에서 자기 자신은 비교에서 제외
            if edit_id is not None and i == edit_id:
                continue
                
            # 위치 중복 체크
            if 'position' in loc:
                loc_pos = loc['position']
                if 'x' in loc_pos and 'y' in loc_pos:
                    if loc_pos['x'] == new_x and loc_pos['y'] == new_y:
                        position_duplicate = True
            
            # 이름 중복 체크
            if 'name' in loc and loc['name'] == new_name:
                name_duplicate = True
            
            # Unique ID 중복 체크 (둘 다 호출벨인 경우만)
            if has_unique_id and loc.get('label') == 'Yes' and 'unique_id' in loc:
                if loc['unique_id'] == new_unique_id:
                    unique_id_duplicate = True
                
            # 모든 중복 체크가 완료되면 즉시 종료
            if position_duplicate and name_duplicate and (unique_id_duplicate or not has_unique_id):
                return "현재 위치, 이름이 이미 있습니다."
        
        # 개별 중복 체크
        if position_duplicate:
            return "위치가 이미 있습니다."
        if name_duplicate:
            return "이름이 이미 있습니다. 다른 이름을 생성해주세요."
        if unique_id_duplicate:
            return "호출벨 Unique ID가 이미 사용 중입니다. 다른 호출벨을 사용해주세요."
            
        return None  # 중복 없음