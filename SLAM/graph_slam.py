import numpy as np

# 시스템 노이즈 파라미터
MOTION_NOISE = 1.0    # 로봇 이동시 발생하는 노이즈
MEASUREMENT_NOISE = 4.0 # 센서 측정시 발생하는 노이즈

class GraphSLAM:
    def __init__(self):
        self.omega = None  # 정보 행렬
        self.xi = None    # 정보 벡터
        self.data = {"steps": []}  # 모든 스텝의 데이터를 저장
        self.num_steps = 0
        self.num_landmarks = 0
        self.solution = None
        self.initial_orientation = 1.5 * np.pi  # Unity의 초기 방향(270도)을 라디안으로 변환
        
    def initialize_matrices(self):
        """정보 행렬과 벡터를 초기화합니다"""
        total_size = self.num_steps * 2 + self.num_landmarks * 2
        
        self.omega = np.zeros((total_size, total_size))
        self.xi = np.zeros((total_size, 1))
        
        # Unity의 초기 위치를 반영한 강한 초기 제약 설정
        very_certain = 1e6
        self.omega[0, 0] = very_certain
        self.omega[1, 1] = very_certain
        self.xi[0, 0] = 22.5 * very_certain  # Unity의 초기 X 위치
        self.xi[1, 0] = 11.5 * very_certain  # Unity의 초기 Y 위치

    def update_matrices(self, json_data):
        """Unity에서 받은 데이터로 행렬을 업데이트합니다"""
        try:
            # 로봇 위치 데이터 추출
            robot_pose = json_data["robot_pose"]
            landmarks = json_data["landmarks"]
            
            # 현재 스텝 데이터 저장
            step_data = {
                "motion": {
                    "orientation": robot_pose["orientation"],
                    "forwardDistance": self.calculate_forward_distance(robot_pose)
                    if self.num_steps > 0 else 0.0
                },
                "landmarks": [
                    {
                        "id": lm["id"],
                        "relative_x": lm["relative_x"],
                        "relative_y": lm["relative_y"]
                    } for lm in landmarks
                ]
            }
            
            self.data["steps"].append(step_data)
            self.num_steps = len(self.data["steps"])
            self.num_landmarks = max([lm["id"] for lm in landmarks]) + 1 if landmarks else 0
            
            self.initialize_matrices()
            self.process_motion()
            self.process_measurements()
            self.solve()
            
        except Exception as e:
            print(f"Error in update_matrices: {str(e)}")
            print(f"Input data structure: {json_data}")

    def calculate_forward_distance(self, current_pose):
        """이전 위치와 현재 위치 사이의 이동 거리를 계산합니다"""
        if self.solution is None or self.num_steps < 2:
            return 0.0
            
        prev_x = self.solution[self.num_steps * 2 - 2][0]
        prev_y = self.solution[self.num_steps * 2 - 1][0]
        curr_x = current_pose["x"]
        curr_y = current_pose["y"]  # Unity에서 이미 변환된 y값 사용
        
        return np.sqrt((curr_x - prev_x)**2 + (curr_y - prev_y)**2)

    def process_motion(self):
        """로봇의 이동을 처리합니다"""
        for i in range(self.num_steps - 1):
            idx_x = i * 2
            idx_y = idx_x + 1
            
            step_data = self.data["steps"][i]
            step_distance = 0.5  # Unity의 stepDistance
            current_orientation = self.initial_orientation + (i * (2 * np.pi / 50))
            
            dx = step_distance * np.cos(current_orientation)
            dy = step_distance * np.sin(current_orientation)

            motion_information = np.array([
                [1/MOTION_NOISE**2, 0],
                [0, 1/MOTION_NOISE**2]
            ])
            
            self.omega[idx_x:idx_x + 2, idx_x:idx_x + 2] += motion_information
            self.omega[idx_x + 2:idx_x + 4, idx_x + 2:idx_x + 4] += motion_information
            self.omega[idx_x:idx_x + 2, idx_x + 2:idx_x + 4] -= motion_information
            self.omega[idx_x + 2:idx_x + 4, idx_x:idx_x + 2] -= motion_information
            
            self.xi[idx_x:idx_x + 2] -= motion_information @ np.array([[dx], [dy]])
            self.xi[idx_x + 2:idx_x + 4] += motion_information @ np.array([[dx], [dy]])

    def process_measurements(self):
        """랜드마크 측정을 처리합니다"""
        measurement_information = np.eye(2) * (1/MEASUREMENT_NOISE**2)
        
        for i, step in enumerate(self.data["steps"]):
            robot_idx_x = i * 2
            robot_idx_y = robot_idx_x + 1
            
            for landmark in step["landmarks"]:
                landmark_id = landmark["id"]
                lm_idx_x = self.num_steps * 2 + landmark_id * 2
                lm_idx_y = lm_idx_x + 1
                
                rel_x = landmark["relative_x"]
                rel_y = landmark["relative_y"]

                self.omega[robot_idx_x:robot_idx_y + 1, robot_idx_x:robot_idx_y + 1] += measurement_information
                self.omega[lm_idx_x:lm_idx_y + 1, lm_idx_x:lm_idx_y + 1] += measurement_information
                self.omega[robot_idx_x:robot_idx_y + 1, lm_idx_x:lm_idx_y + 1] -= measurement_information
                self.omega[lm_idx_x:lm_idx_y + 1, robot_idx_x:robot_idx_y + 1] -= measurement_information

                measurement = np.array([[rel_x], [rel_y]])
                self.xi[robot_idx_x:robot_idx_y + 1] -= measurement_information @ measurement
                self.xi[lm_idx_x:lm_idx_y + 1] += measurement_information @ measurement

    def solve(self):
        """최적의 로봇 경로와 랜드마크 위치를 계산합니다"""
        try:
            self.solution = np.linalg.solve(self.omega, self.xi)
        except np.linalg.LinAlgError:
            print("Warning: Using pseudo-inverse due to singular matrix")
            self.solution = np.linalg.pinv(self.omega) @ self.xi

    def get_current_state(self):
        """현재 로봇의 상태를 반환합니다"""
        if self.solution is None:
            return None
        
        current_idx = (self.num_steps - 1) * 2
        return {
            'x': float(self.solution[current_idx]),
            'y': float(self.solution[current_idx + 1])
        }