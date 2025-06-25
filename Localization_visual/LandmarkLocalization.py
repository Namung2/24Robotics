import numpy as np
import time
from LandmarkParticleFilter import LandmarkParticleFilter
from UnityInterfaceFor23 import UnityInterfaceFor23
from particle_filter_visualizer import ParticleFilterVisualizer

class LandmarkLocalization:
    def __init__(self):
        self.unity_interface = UnityInterfaceFor23()
        self.particle_filter = LandmarkParticleFilter()
        self.map_size = (25, 15)
        self.confidence_threshold = 0.8
        self.localization_complete = False
        
        # 랜드마크 위치 초기화
        self.landmarks = np.array([
            [5, 11],   # 왼쪽 위
            [16, 9],   # 오른쪽 위
            [5, 4],    # 왼쪽 아래
            [15, 2]    # 오른쪽 아래
        ])
        
        # 시각화 초기화
        self.particle_filter_visualizer = ParticleFilterVisualizer(self.landmarks, self.map_size)

    def update(self):
        """측정값을 받아 파티클 필터 업데이트"""
        if self.localization_complete:
            return

        data = self.unity_interface.get_latest_data()
        if not data:
            return

        try:
            robot_pose = data["robot_pose"]
            x, y, orientation = robot_pose["x"], robot_pose["y"], robot_pose["orientation"]
            
            # 파티클 이동 예측
            self.particle_filter.predict([x, y, orientation])
            
            # 측정값이 있으면 업데이트
            measurements = data.get("measurements", [])
            if measurements:
                self.particle_filter.update(measurements)
                
                # 위치 신뢰도 계산 및 시각화
                confidence = self.particle_filter.get_confidence()
                particles = self.particle_filter.particles
                weights = self.particle_filter.weights
                estimated_pose = self.particle_filter.get_position()
                
                self.particle_filter_visualizer.update(
                    particles, weights, estimated_pose, confidence)
                
                print(f"Current confidence: {confidence}")
                
                # 신뢰도가 임계값을 넘으면 종료
                if confidence >= self.confidence_threshold:
                    print(f"Localization completed with confidence: {confidence}")
                    self.localization_complete = True
                    return

        except Exception as e:
            print(f"Error during update: {str(e)}")

    def run(self):
        """메인 실행 루프"""
        try:
            while not self.localization_complete:
                self.update()
                time.sleep(0.1)  # 10Hz로 실행
            print("Localization completed successfully")
        except KeyboardInterrupt:
            print("\nProgram terminated")
            self.unity_interface.close()

if __name__ == "__main__":
    localizer = LandmarkLocalization()
    localizer.run()
