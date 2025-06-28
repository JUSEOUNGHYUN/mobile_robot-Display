import os
import json
from PySide6.QtCore import QObject, Signal
from models.RingoBellManager import RingoBellManager

"""
    1. 목적지 데이터 영구 저장
    -- JSON 파일(data/locations.txt)에 목적지 정보 저장
    -- 파일 기반 데이터베이스 역할 수행
    2. 목적지 CRUD 작업
    -- 목적지 추가, 읽기, 수정, 삭제 기능
    -- 인메모리 캐시와 파일 저장소 동기화
    3. 목적지 검색 및 필터링
    -- 이름 또는 UUID(호출벨 ID) 기반 검색
    -- 필터링 상태 관리
    4. 시그널 발생
    -- 데이터 변경 시 시그널을 통해 UI 업데이트 트리거
    -- Qt의 시그널-슬롯 메커니즘을 활용한 MVC 구현
"""


class LocationSelectionModel(QObject):
    # 위치 선택 시그널
    location_selected = Signal(dict, int)  # 선택된 위치 데이터, 인덱스
    
    # 위치 목록 변경 시그널
    locations_updated = Signal(list)  # 업데이트된 위치 목록
    
    # 위치 삭제 결과 시그널
    delete_result = Signal(bool, str)  # 성공 여부, 메시지
    
    def __init__(self, file_path='data/locations.txt'):
        super().__init__()
        # 파일 경로 설정
        self.file_path = file_path
        self._ensure_file_exists()
        
        # 현재 위치 목록
        self._locations = []
        self._selected_index = -1
        self._filtered_locations = []  # 검색 필터링된 위치 목록
        self._is_filtered = False      # 필터링 적용 여부
        
        # 초기 데이터 로드
        self._reload_locations()
    
    # Json 파일 존재 확인 및 생성
    def _ensure_file_exists(self):
        """파일이 존재하지 않으면 기본 구조로 생성"""
        if not os.path.exists(os.path.dirname(self.file_path)):
            os.makedirs(os.path.dirname(self.file_path))
        
        if not os.path.exists(self.file_path):
            with open(self.file_path, 'w') as file:
                json.dump([], file)
    
    def _reload_locations(self):
        """위치 목록 다시 로드"""
        self._locations = self.read_locations()
        self._is_filtered = False
        self._filtered_locations = []
        self.locations_updated.emit(self._locations)
    
    def select_location(self, index):
        """위치 선택"""
        if 0 <= index < len(self._locations):
            self._selected_index = index
            location_data = self._locations[index]
            self.location_selected.emit(location_data, index)
            return True
        return False
    
    def get_location_by_index(self, index):
        """인덱스로 위치 정보 가져오기"""
        if 0 <= index < len(self._locations):
            return self._locations[index]
        return None
    
    def get_selected_location(self):
        """현재 선택된 위치 정보 가져오기"""
        if 0 <= self._selected_index < len(self._locations):
            return self._locations[self._selected_index], self._selected_index
        return None, -1
    
    def get_all_locations(self):
        """모든 위치 목록 가져오기"""
        return self._locations.copy()
    
    def refresh(self):
        """위치 목록 새로고침"""
        self._reload_locations()
    
    def delete_location(self, index):
        """위치 삭제"""
        if 0 <= index < len(self._locations):
            try:
                # 위치 목록에서 삭제
                locations = self.read_locations()
                if 0 <= index < len(locations):
                    # 삭제할 위치 정보 기록 (로깅용)
                    deleted_location = locations[index].get('name', 'Unknown')

                    # 위치 삭제
                    del locations[index]

                    # JSON 형식으로 저장
                    with open(self.file_path, 'w') as file:
                        json.dump(locations, file, indent=2)

                    # 메모리 내 목록 업데이트
                    self._locations = locations

                    # 선택된 위치가 삭제된 경우 선택 해제
                    if self._selected_index == index:
                        self._selected_index = -1
                    elif self._selected_index > index:
                        # 삭제된 인덱스보다 큰 경우 하나 감소
                        self._selected_index -= 1

                    # 필터링이 적용된 경우 재적용
                    if self._is_filtered:
                        self._apply_current_filter()

                    # 목록 변경 시그널 발생
                    self.locations_updated.emit(self._get_current_locations())
                    self.delete_result.emit(True, f"'{deleted_location}' 목적지가 성공적으로 삭제되었습니다.")
                    return True
                else:
                    self.delete_result.emit(False, "삭제할 위치를 찾을 수 없습니다.")
                    return False
            except Exception as e:
                self.delete_result.emit(False, f"삭제 중 오류 발생: {str(e)}")
                return False
        else:
            self.delete_result.emit(False, "유효하지 않은 인덱스입니다.")
            return False
    
    def search_locations(self, search_text):
        """위치 검색"""
        if not search_text:
            # 검색어가 없으면 전체 목록 표시
            self._is_filtered = False
            self._filtered_locations = []
            self.locations_updated.emit(self._locations)
            return self._locations
            
        search_text = search_text.lower()
        self._filtered_locations = []
        
        for location in self._locations:
            # 이름으로 검색
            if 'name' in location and search_text in location['name'].lower():
                self._filtered_locations.append(location)
                continue
                
            # UUID로 검색 (호출벨이 있는 경우)
            if location.get('label') == 'Yes' and 'unique_id' in location:
                if search_text in location['unique_id'].lower():
                    self._filtered_locations.append(location)
        
        # 필터링 상태 설정
        self._is_filtered = True
                    
        # 검색 결과 시그널 발생
        self.locations_updated.emit(self._filtered_locations)
        return self._filtered_locations
    
    def _apply_current_filter(self):
        """현재 필터 재적용"""
        if not self._is_filtered:
            return
        
        # 필터링된 목록 재생성
        new_filtered = []
        for location in self._filtered_locations:
            # 삭제되지 않은 위치만 포함
            found = False
            for loc in self._locations:
                if (loc.get('name') == location.get('name') and 
                    loc.get('position', {}).get('x') == location.get('position', {}).get('x') and
                    loc.get('position', {}).get('y') == location.get('position', {}).get('y')):
                    new_filtered.append(loc)
                    found = True
                    break

        self._filtered_locations = new_filtered

    def _get_current_locations(self):
        """현재 표시할 위치 목록 반환 (필터링 상태에 따라)"""
        if self._is_filtered:
            return self._filtered_locations
        return self._locations

    def add_location(self, location_data):
        """새 위치 추가"""
        try:
            # 기존 목록 읽기
            locations = self.read_locations()

            # 새 위치 추가
            locations.append(location_data)

            # 파일에 저장
            with open(self.file_path, 'w') as file:
                json.dump(locations, file, indent=2)

            # 메모리 내 목록 업데이트
            self._locations = locations

            # 필터링이 적용된 경우 재적용
            if self._is_filtered:
                self._apply_current_filter()

            # 목록 변경 시그널 발생
            self.locations_updated.emit(self._get_current_locations())
            return True

        except Exception as e:
            print(f"위치 추가 중 오류 발생: {str(e)}")
            return False

    def update_location(self, index, updated_data):
        """위치 정보 업데이트"""
        try:
            # 기존 목록 읽기
            locations = self.read_locations()

            # 인덱스 유효성 검사
            if 0 <= index < len(locations):
                # 위치 정보 업데이트
                locations[index] = updated_data

                # 파일에 저장
                with open(self.file_path, 'w') as file:
                    json.dump(locations, file, indent=2)

                # 메모리 내 목록 업데이트
                self._locations = locations

                # 필터링이 적용된 경우 재적용
                if self._is_filtered:
                    self._apply_current_filter()

                # 목록 변경 시그널 발생
                self.locations_updated.emit(self._get_current_locations())
                return True
            return False
        except Exception as e:
            print(f"위치 업데이트 중 오류 발생: {str(e)}")
            return False
        
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

    def get_location_count(self):
        """위치 목록 개수 반환"""
        if self._is_filtered:
            return len(self._filtered_locations)
        return len(self._locations)

    def find_location_by_name(self, name):
        """이름으로 위치 찾기"""
        if not name:
            return None, -1

        name = name.lower()
        locations = self._get_current_locations()

        for i, location in enumerate(locations):
            if 'name' in location and location['name'].lower() == name:
                return location, i

        return None, -1

    def find_location_by_uuid(self, uuid_str):
        """UUID로 위치 찾기"""
        if not uuid_str:
            return None, -1

        uuid_str_lower = uuid_str.lower().strip()
        locations = self._get_current_locations()

        for i, location in enumerate(locations):
            if location.get('label') == 'Yes' and 'unique_id' in location:
                location_uuid = location.get('unique_id', '').lower().strip()
                if location_uuid == uuid_str_lower:
                    return location, i

        return None, -1

    def find_location_id(self, location_data):
        if not location_data:
            return None

        for i, loc in enumerate(self._locations):
            # 이름과 위치로 비교
            if (loc.get('name') == location_data.get('name') and 
                loc.get('position', {}).get('x') == location_data.get('position', {}).get('x') and
                loc.get('position', {}).get('y') == location_data.get('position', {}).get('y')):
                return i

        return None

    def file_updated_externally(self, locations=None):
        """외부에서 파일이 업데이트되었을 때 호출하는 메서드"""
        if locations is None:
            # 파일에서 다시 읽기
            self._reload_locations()
        else:
            # 제공된 위치 목록으로 업데이트
            self._locations = locations

            # 필터링이 적용된 경우 재적용
            if self._is_filtered:
                self._apply_current_filter()

            self.locations_updated.emit(self._get_current_locations())

            # 선택된 위치가 유효한지 확인
            if self._selected_index >= len(self._locations):
                self._selected_index = -1