import numpy as np
from scipy.stats import norm

class LandmarkParticleFilter:
    def __init__(self, num_particles=1000, map_size=(25, 15), motion_noise=0.1, measurement_noise=0.4):
        # 파티클 필터의 기본 파라미터 초기화
        self.num_particles = num_particles
        self.map_size = np.array(map_size)
        self.motion_noise = motion_noise
        self.measurement_noise = measurement_noise
        self.confidence_history = []  # 추가
        
        # 랜드마크 위치 초기화 (문제에서 주어진 위치)
        self.landmarks = np.array([
            [5, 11],   # 왼쪽 위
            [16, 9],   # 오른쪽 위
            [5, 4],    # 왼쪽 아래
            [15, 2]    # 오른쪽 아래
        ])
        
        # 파티클과 가중치 초기화
        self.particles = np.zeros((num_particles, 3))  # x, y, theta
        self.weights = np.ones(num_particles) / num_particles
        self._initialize_particles()
        # 이동 제한
        self.max_movement = 0.5  # 한 번에 최대 이동 거리
        
        # 측정 최대 거리 (문제 조건)
        self.max_range = 5.0

    def _initialize_particles(self):
        """파티클들을 맵 전체에 균일하게 분포시킵니다"""
        self.particles[:, 0] = np.random.uniform(0, self.map_size[0], self.num_particles)
        self.particles[:, 1] = np.random.uniform(0, self.map_size[1], self.num_particles)
        self.particles[:, 2] = np.random.uniform(0, 2*np.pi, self.num_particles)
        self.weights = np.ones(self.num_particles) / self.num_particles
                
    def predict(self, motion):
        """개선된 예측 단계: 벽 근처 파티클 처리 추가"""
        # 이동 제한
        dx = np.clip(motion[0] - self.particles[:, 0], -self.max_movement, self.max_movement)
        dy = np.clip(motion[1] - self.particles[:, 1], -self.max_movement, self.max_movement)
        
        # 노이즈 추가
        self.particles[:, 0] += dx + np.random.normal(0, self.motion_noise, self.num_particles)
        self.particles[:, 1] += dy + np.random.normal(0, self.motion_noise, self.num_particles)
        self.particles[:, 2] = np.radians(motion[2]) + np.random.normal(0, self.motion_noise*0.1, self.num_particles)
        
        # 경계 처리
        self.particles[:, 0] = np.clip(self.particles[:, 0], 0, self.map_size[0])
        self.particles[:, 1] = np.clip(self.particles[:, 1], 0, self.map_size[1])
        self.particles[:, 2] = self.particles[:, 2] % (2 * np.pi)
            
            
    def update(self, measurements):
        """측정값을 기반으로 파티클 가중치 업데이트"""

        for measurement in measurements:
            if not isinstance(measurement, dict):
                continue
                
            landmark_id = measurement.get('id')
            measured_distance = measurement.get('distance')
            
            if landmark_id is None or measured_distance is None:
                continue
                
            if landmark_id >= len(self.landmarks):
                continue
                
            # 각 파티클에서 해당 랜드마크까지의 예측 거리 계산
            landmark_pos = self.landmarks[landmark_id]
            predicted_distances = np.sqrt(
                (self.particles[:, 0] - landmark_pos[0])**2 + 
                (self.particles[:, 1] - landmark_pos[1])**2
            )
            
             # 더 점진적인 가중치 업데이트
            likelihood = norm.pdf(measured_distance, predicted_distances, self.measurement_noise)
            self.weights *= (0.5 + 0.5 * likelihood)  # 가중치 변화를 부드럽게

        # 가중치 정규화
        if np.sum(self.weights) > 0:
            self.weights /= np.sum(self.weights)
        else:
            self.weights = np.ones(self.num_particles) / self.num_particles

        # 유효 파티클 수가 임계값 이하면 리샘플링
        n_eff = 1.0 / np.sum(self.weights ** 2)
        if n_eff < self.num_particles / 2:
            self.resample()
            
    def resample(self):
        cumsum = np.cumsum(self.weights)
        cumsum[-1] = 1.0
        
        # 부드러운 리샘플링
        indices = []
        u = (np.arange(self.num_particles) + np.random.random()) / self.num_particles
        for u_i in u:
            indices.append(np.searchsorted(cumsum, u_i))
        
        # 새 파티클 생성시 약간의 분산 추가
        self.particles = self.particles[indices] + np.random.normal(0, self.motion_noise*0.1, self.particles.shape)
        self.weights = np.ones(self.num_particles) / self.num_particles
        
    def get_position(self):
        """현재 추정된 로봇의 위치를 반환합니다"""
        max_weight_idx = np.argmax(self.weights)
        return self.particles[max_weight_idx]

    def get_confidence(self):
        """현재 위치 추정의 확실성을 계산합니다"""
        # 파티클들의 분산이 작을수록 높은 확실성을 가짐
        position_variance = np.var(self.particles[:, :2], axis=0)
        total_variance = np.sum(position_variance)
        return 1.0 / (1.0 + total_variance)  # 0~1 사이 값으로 정규화