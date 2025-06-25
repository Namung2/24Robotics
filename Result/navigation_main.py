import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List
import time
from BaseLocalization import BaseLocalization
from smoothNavigator import SmoothNavigator
from UnityInterfaceFor3 import UnityInterface3

class NavigationSystem:
    def __init__(self):
        print("Initializing Navigation System...")
        # 컴포넌트 초기화
        self.interface = UnityInterface3()
        self.localizer = BaseLocalization()
        self.navigator = SmoothNavigator()
        
        # 시스템 상태 변수
        self.is_running = True
        self.localization_complete = False
        self.path_planned = False
        
        self.last_localization_time = 0.0
        self.last_pid_time = 0.0
        self.localization_interval = 0.01  # Localization 주기 (1초)
        self.pid_interval = 3.0  # PID 계산 및 명령 전송 주기 (3초)
        
        # 시각화 설정
        self.setup_visualization()
        print("Navigation System initialized")
        
        
    def setup_visualization(self):
        """매트랩 시각화 설정"""
        self.fig, self.ax = plt.subplots(figsize=(10,6))
        self.ax.set_xlim(0, 25)
        self.ax.set_ylim(0, 15)
        self.ax.grid(True)
        self.ax.set_xticks(range(0, 26, 1))  # x축 격자 간격
        self.ax.set_yticks(range(0, 16, 1))  # y축 격자 간격
        self.ax.grid(True, which='both', color='black', linestyle='-', linewidth=0.5)  # 선 굵기와 색상 설정
            # 기본 장애물 위치 정의 (격자 중심점 사용)
        base_obstacles = [
            # 왼쪽 첫 번째 그룹
            *[(3, y) for y in range(9, 13)],
            *[(4, y) for y in range(9, 13)],
            
            # 왼쪽에서 두 번째 그룹
            *[(8, y) for y in range(9, 13)],
            *[(9, y) for y in range(9, 13)],
            
            # 오른쪽에서 두 번째 그룹
            *[(14, y) for y in range(9, 13)],
            *[(15, y) for y in range(9, 13)],
            
            # 니은자 모양 장애물
            *[(18, y) for y in range(6, 13)],
            *[(x, 6) for x in range(9, 19)],
            
            # 맨 오른쪽 장애물
            *[(x, y) for x in range(21, 25) for y in range(8, 10)],
            
            # 아래쪽 장애물들
            *[(x, y) for x in range(3, 7) for y in range(5, 7)],
            
            # 2x2 크기의 장애물들
            *[(x, y) for x in [12, 13] for y in [2, 3]],
            *[(x, y) for x in [16, 17] for y in [2, 3]],
            *[(x, y) for x in [20, 21] for y in [2, 3]]
            ]
        # 장애물 그리기
        obstacles = [(x + 0.5, y + 0.5) for x, y in base_obstacles]
        obstacle_x = [x for x, y in obstacles]
        obstacle_y = [y for x, y in obstacles]
        self.ax.scatter(obstacle_x, obstacle_y, c='cornflowerblue', s=200, marker='s', label='Obstacles')
    
        # 랜드마크 표시
        landmarks = self.navigator.landmarks
        landmark_x = [x for x, y in landmarks]
        landmark_y = [y for x, y in landmarks]
        self.ax.plot(landmark_x, landmark_y, 'bs', markersize=15, label='Landmarks')
        self.ax.plot(18.5, 1.5, 'r*', markersize=15, label='Goal (P2)')
        # 로봇과 경로 표시
        # 동적으로 업데이트될 요소들 초기화
        self.robot_pos, = self.ax.plot([], [], 'ro', markersize=10, label='Robot')
        self.original_path, = self.ax.plot([], [], 'b-', label='Original Path', alpha=0.5)
        self.smooth_path, = self.ax.plot([], [], 'r-', label='Smoothed Path')
        self.robot_trail, = self.ax.plot([], [], 'g--', label='Robot Trail', alpha=0.7)
        self.particles, = self.ax.plot([], [], 'k.', markersize=1, alpha=0.3, label='Particles')
        
        # 제어 벡터 표시 (로봇의 이동 방향)
        self.control_vector = self.ax.quiver([], [], [], [], color='k', scale=1)
        self.position_history = []
        self.ax.legend()
        plt.ion()
        plt.show()
        
    def update_visualization(self, robot_pose: Tuple[float, float, float], 
                           control_output: Tuple[float, float] = None):
        """시각화 업데이트"""
        # 로봇 위치 기록 및 표시
        if self.path_planned:
            self.position_history.append(robot_pose[:2])
        self.robot_pos.set_data([robot_pose[0]], [robot_pose[1]])
                
        # 파티클 표시 (항상)
        particles = self.localizer.get_particles()
        self.particles.set_data(particles[:, 0], particles[:, 1])
        # 경로 표시
        if self.path_planned:
            if self.navigator.original_path:
                orig_x = [p[0] for p in self.navigator.original_path]
                orig_y = [p[1] for p in self.navigator.original_path]
                self.original_path.set_data(orig_x, orig_y)
            
            if self.navigator.smoothed_path:
                smooth_x = [p[0] for p in self.navigator.smoothed_path]
                smooth_y = [p[1] for p in self.navigator.smoothed_path]
                self.smooth_path.set_data(smooth_x, smooth_y)
        # 로봇의 이동 궤적 표시
        if self.position_history:
            trail_x = [pos[0] for pos in self.position_history]
            trail_y = [pos[1] for pos in self.position_history]
            self.robot_trail.set_data(trail_x, trail_y)
        
        # 제어 벡터 표시 (있을 경우)
        if control_output:
            dx, dy = control_output
            self.control_vector.set_offsets([[robot_pose[0], robot_pose[1]]])
            self.control_vector.set_UVC([dx], [dy])
        
        plt.draw()
        plt.pause(0.2)
        
    def run(self):
        """메인 실행 루프"""
        print("\nStarting Navigation...")
        try:
            while self.is_running:
                current_time = time.time()

                # Localization (1초마다 실행)
                if current_time - self.last_localization_time >= self.localization_interval:
                    data = self.interface.get_latest_data()
                    if not data:
                        continue

                    robot_pose = data["robot_pose"]
                    x, y, orientation = robot_pose["x"], robot_pose["y"], robot_pose["orientation"]
                    self.localizer.predict([x, y, orientation])
                    measurements = data.get("measurements", [])
                    self.localizer.update(measurements)

                    # 시각화: Localization 상태 업데이트
                    estimated_pose = self.localizer.get_position()
                    self.update_visualization(estimated_pose)  # 로봇 위치 갱신

                    self.last_localization_time = current_time

                # Localization 신뢰도 확인 및 경로 계획
                confidence = self.localizer.get_confidence()
                if confidence >= self.navigator.localization_confidence_threshold:
                    if not self.path_planned:
                        print("\n신뢰도 0.8 달성! 경로 계획 시작...")
                        estimated_pose = self.localizer.get_position()  # 추정 위치 확인
                        self.navigator._plan_path(estimated_pose)  # 경로 계획
                        self.path_planned = True  # 경로 계획 완료
                        self.localization_complete = True

                # PID 제어 및 명령 전송 (3초마다 실행)
                if self.path_planned and current_time - self.last_pid_time >= self.pid_interval:
                    estimated_pose = self.localizer.get_position()
                    dx, dy = self.navigator._compute_control(estimated_pose)

                    if dx != 0 or dy != 0:  # 이동 벡터가 유효하면 Unity로 명령 전송
                        self.interface.send_movement_command(dx, dy)
                        self.last_pid_time = current_time

                        # 시각화: PID 제어 벡터 업데이트
                        self.update_visualization(estimated_pose,(dx,dy))  # 로봇 위치와 제어 벡터 갱신

                    # 목표 도달 확인
                    if self.navigator._is_at_goal(estimated_pose):
                        print("\nGoal reached! Navigation complete.")
                        break

                # 신뢰도 0.8 미만일 때 경로 재계획 필요
                elif self.path_planned and confidence < self.navigator.localization_confidence_threshold:
                    print("\n신뢰도가 0.8 미만입니다. 경로를 다시 계획합니다.")
                    self.path_planned = False

                time.sleep(0.1)  # 메인 루프 주기 (0.1초)

        except KeyboardInterrupt:
            print("\nNavigation terminated by user.")
        finally:
            self.cleanup()
    def cleanup(self):
        """시스템 종료 시 리소스 정리"""
        print("\nCleaning up...")
        self.interface.close()
        plt.close()

if __name__ == "__main__":
    navigation_system = NavigationSystem()
    navigation_system.run()
