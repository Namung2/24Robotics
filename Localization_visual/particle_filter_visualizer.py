import matplotlib.pyplot as plt
import numpy as np

class ParticleFilterVisualizer:
    def __init__(self, landmarks, map_size):
        self.landmarks = landmarks
        self.map_size = map_size
        plt.ion() 
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.setup_visualization()
        plt.show(block=False)

    def setup_visualization(self):
        """시각화 초기화"""
        self.ax.set_xlim(0, self.map_size[0])
        self.ax.set_ylim(0, self.map_size[1])
        self.ax.set_title("Particle Filter Visualization")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.grid(True)

        # 랜드마크 표시
        for landmark in self.landmarks:
            self.ax.scatter(landmark[0], landmark[1], c='blue', marker='^', s=100, label='Landmarks')
        self.ax.legend()

    def update(self, particles, weights, estimated_pose, confidence):
        """시각화 업데이트"""
        self.ax.clear()
        self.setup_visualization()

        # 파티클 표시
        self.ax.scatter(particles[:, 0], particles[:, 1], c='red', marker='x', s=weights * 3000, label='Particles')

        # 추정된 로봇 위치 표시
        self.ax.scatter(estimated_pose[0], estimated_pose[1], c='black', marker='o', s=100, label='Estimated Position')

        # 로봇의 방향 표시
        arrow_length = 0.5
        dx = arrow_length * np.cos(estimated_pose[2])
        dy = arrow_length * np.sin(estimated_pose[2])
        #self.ax.arrow(estimated_pose[0], estimated_pose[1], dx, dy, head_width=0.2, head_length=0.3, fc='red', ec='red')

        # 신뢰도 표시
        self.ax.set_title(f"Confidence: {confidence:.2f}")
        self.ax.legend()
        plt.pause(1.0)
