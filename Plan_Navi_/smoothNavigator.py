import numpy as np
from typing import List, Tuple
from copy import deepcopy
import time
import heapq
from enum import Enum

class Direction(Enum):
    """격자 기반 이동 방향 정의"""
    UP = (0, 1)
    DOWN = (0, -1)
    LEFT = (-1, 0)
    RIGHT = (1, 0)

class SmoothNavigator:
    def __init__(self):
        """초기화: 경로 계획, 평활화, PID 제어에 필요한 변수들을 설정"""
        # 경로 관련 변수
        self.original_path = []
        self.smoothed_path = []
        self.current_path_index = 0
        
        # 목표 지점 (P2)
        self.goal = (18.5, 1.5)
        self.goal_grid = (18,1) #격자 좌표
        
        # 맵 크기와 랜드마크 정의
        self.map_size = (25, 15)
        self.landmarks = [
            (5.5, 11.5),   # 왼쪽 위
            (16.5, 9.5),   # 오른쪽 위
            (5.5, 4.5),    # 왼쪽 아래
            (15.5, 2.5)    # 오른쪽 아래
        ]
        
        # Path smoothing 파라미터
        self.weight_data = 0.9
        self.weight_smooth = 0.1
        self.smooth_tolerance = 0.00001
        
        # PID 제어 파라미터
        self.Kp = 0.3 #0.5
        self.Ki = 0.05 #0.1
        self.Kd = 0.4# 미분게인으로 진동억제
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
        # Navigation 상태
        self.navigation_state = "WAITING_FOR_LOCALIZATION"
        self.localization_confidence_threshold = 0.8
        
        
    def _plan_path(self, start_pose: Tuple[float, float]):
        """A* 알고리즘으로 초기 경로 생성 후 평활화"""
        # 연속 좌표를 격자 좌표로 변환
        # 경로 초기화 추가
        self.original_path = []
        self.smoothed_path = []
        self.current_path_index = 0
        self.integral = 0.0  # PID 제어 적분항 초기화
        start_grid = (int(round(start_pose[0])), int(round(start_pose[1])))
        print(f"Planning path from {start_grid} to {self.goal_grid}")
        
        # A* 알고리즘으로 경로 찾기
        path = self._astar(start_grid)
        if not path:
            print("No path found!")
            return False
            
        print(f"Found path: {path}")
        
        # 격자 경로를 연속 좌표로 변환 (중심점 사용)
        self.original_path = [(x + 0.5, y + 0.5) for x, y in path]
        
        # 경로 평활화
        self.smoothed_path = self._smooth_path(self.original_path)
        print("Path planning and smoothing completed")
        
    def _astar(self, start: Tuple[int, int]) -> List[Tuple[int, int]]:
        """A* 경로 탐색 알고리즘"""
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        while frontier:
            current = heapq.heappop(frontier)[1]
            
            if current == self.goal_grid:
                break
                
            for direction in Direction:
                dx, dy = direction.value
                next_pos = (current[0] + dx, current[1] + dy)
                
                if not self._is_valid_position(next_pos):
                    continue
                    
                new_cost = cost_so_far[current] + 1
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self._euclidean_distance(next_pos, self.goal_grid)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current
        
        # 경로 재구성
        if self.goal_grid not in came_from:
            return []
            
        path = []
        current = self.goal_grid
        while current is not None:
            path.append(current)
            current = came_from.get(current)
        path.reverse()
        
        return path
        
    def _smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """경로 평활화"""
        print("Smoothing path...")
        
        # Convert path to mutable numpy array for modification
        newpath = np.array(path, dtype=float)  # 변경 가능 구조로 변환
        path_array = np.array(path, dtype=float)  # 원본도 numpy로 변환

        error = self.smooth_tolerance + 1
        while error >= self.smooth_tolerance:
            error = 0
            for i in range(1, len(path_array) - 1):
                for j in range(2):  # x, y 좌표
                    old = newpath[i][j]
                    newpath[i][j] += self.weight_data * (path_array[i][j] - newpath[i][j])
                    newpath[i][j] += self.weight_smooth * (
                        newpath[i + 1][j] + newpath[i - 1][j] - 2 * newpath[i][j]
                    )
                    error += abs(old - newpath[i][j])

        print("Path smoothing completed")
        return [tuple(point) for point in newpath]  # 리스트의 튜플로 반환
    def _is_valid_position(self, pos: Tuple[int, int]) -> bool:
        """격자 위치가 유효한지 확인"""
        x, y = pos
        return 0 <= x < self.map_size[0] and 0 <= y < self.map_size[1]

    def _euclidean_distance(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """두 격자 점 사이의 유클리드 거리 계산 휴리스틱"""
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def _compute_lookahead_point(self, current_pose: Tuple[float, float, float]) -> Tuple[float, float]:
        """현재 위치에서 가장 적절한 목표점 계산"""
        if not self.smoothed_path:
            return current_pose[:2]
        current_point = np.array(current_pose[:2])
    
        # 현재 위치에서 가장 가까운 경로상의 점 찾기
        min_dist = float('inf')
        closest_index = self.current_path_index
        
        # 현재 인덱스부터 앞쪽 경로만 검색
        for i in range(self.current_path_index, len(self.smoothed_path)):
            path_point = np.array(self.smoothed_path[i])
            dist = np.linalg.norm(current_point - path_point)
            if dist < min_dist:
                min_dist = dist
                closest_index = i

        # lookahead distance만큼 앞선 점을 목표점으로 선택
        lookahead_index = min(closest_index + 2, len(self.smoothed_path) - 1)
        return self.smoothed_path[lookahead_index]
        
    def _compute_control(self, current_pose: Tuple[float, float, float]) -> Tuple[float, float]:
        """향상된 PID 제어"""
        if not self.smoothed_path:
            return 0.0, 0.0
            
        # 현재 상태에서 가장 적절한 목표점 찾기
        target = self._compute_lookahead_point(current_pose)
        current_time = time.time()
        
        dt = current_time - self.last_time
        # 제어 주기 최소값 설정 (0.1초 이상)
        min_control_interval =1
        if dt < min_control_interval:
            return 0.0, 0.0  # 너무 자주 계산하지 않도록 스킵
            
        if dt <= 0.01: #dt값 0으로 인한 미분항 발산 방지
            return 0.0, 0.0
            
        # 현재 위치와 목표점 사이의 오차 계산
        error_x = target[0] - current_pose[0]
        error_y = target[1] - current_pose[1]
        error = np.sqrt(error_x**2 + error_y**2)
        
        # PID 제어
        P = self.Kp * error
        self.integral += error * dt
        # 적분 항에 한계를 설정 (Anti-windup)
        self.integral = max(min(self.integral, 10), -10)  # 예: -10 ~ 10 사이로 제한
        
        
        I = self.Ki * self.integral
        D = self.Kd * (error - self.prev_error) / dt
        
        control = P + I + D
        
        # 제어 출력을 dx, dy로 변환
        angle = np.arctan2(error_y, error_x)
        dx = control * np.cos(angle)
        dy = control * np.sin(angle)
        
        #벡터의 크기가 너무작음 최소 이동거리 제한 강제 scale조정 로wlr 추가
        # 최대 이동거리 설정 최대 이동 거리를 늘림
        max_step = 2.0  # 한 번에 최대 1.0 단위까지 이동 가능하도록 증가
        min_step = 0.5  # 최소 이동 거리
        magnitude = np.sqrt(dx*dx + dy*dy)
        if magnitude > max_step:
            dx = dx * max_step / magnitude
            dy = dy * max_step / magnitude
        elif magnitude < min_step:
            if magnitude > 0:  # 0으로 나누는 문제 방지
                dx = dx * min_step / magnitude
                dy = dy * min_step / magnitude
            else:
                dx, dy = min_step, 0  # 기본 x 방향으로 최소 이동
        # 상태 업데이트
        self.prev_error = error
        self.last_time = current_time
        
        # 현재 목표점에 도달했는지 확인 도착했으면 다음점으로 업데이트
        if error < 0.5 and self.current_path_index < len(self.smoothed_path) - 1:
            self.current_path_index += 1
            self.integral = 0  # 적분항 리셋
            print(f"Updated target to point {self.current_path_index}")

            print(f"Computing control: dx={dx}, dy={dy}")
        return dx, dy
        
    def _is_at_goal(self, pose: Tuple[float, float, float], threshold: float = 0.1) -> bool:
        """목표 지점 도달 확인"""
        distance = np.sqrt(
            (pose[0] - self.goal[0])**2 + 
            (pose[1] - self.goal[1])**2
        )
        return distance < threshold
