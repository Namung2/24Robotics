import matplotlib.pyplot as plt
import matplotlib.animation as animation
from UnityInterface import UnityInterface
from graph_slam import GraphSLAM
import numpy as np

def animate_slam():
    unity_interface = UnityInterface()
    slam = GraphSLAM()

    # 그래프 설정
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.set_xlim(-10, 40)  # Unity의 맵 크기에 맞춤
    ax.set_ylim(0, 35)
    # 격자 설정
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.set_title('Real-time SLAM Visualization')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    
    # 플롯 요소 초기화
    robot_positions = []
    robot_path, = ax.plot([], [], 'r.-', label="Robot Path", markersize=8)
    landmarks, = ax.plot([], [], 'b*', label="Landmarks", markersize=10)
    current_pos, = ax.plot([], [], 'go', label="Current Position", markersize=12)
    expected_path, = ax.plot([], [], 'g--', label="Expected Path", markersize=8)
    # 로봇 방향 표시를 위한 화살표 초기화
    #robot_direction = ax.quiver([], [], [], [], color='g', scale=20)
    
    plt.legend()

    def init():
        """애니메이션 초기화 함수"""
        robot_path.set_data([], [])
        landmarks.set_data([], [])
        current_pos.set_data([], [])
        expected_path.set_data([], [])
        # robot_direction.set_UVC([], [])
        # robot_direction.set_offsets(np.c_[[], []])
        return robot_path, landmarks, current_pos, expected_path#, robot_direction    
    
    def transform_coordinates(x, y):
        """Unity 좌표계를 SLAM 좌표계로 변환"""
        # 주석: 기존의 90도 회전을 유지하면서 좌표계의 중심과 스케일을 조정합니다
        
        # 1. 첫 번째로, Unity의 초기 위치를 원점으로 이동
        x_centered = x - 22.5  # Unity 초기 X 위치를 빼서 중심 이동
        y_centered = y - 11.5  # Unity 초기 Y 위치를 빼서 중심 이동
        
        # 2. 90도 회전 변환 적용 (반시계 방향)
        theta = np.pi  # 양의 90도 회전 (반시계 방향)
        x_rotated = x_centered * np.cos(theta) - y_centered * np.sin(theta)
        y_rotated = x_centered * np.sin(theta) + y_centered * np.cos(theta)
        
        # 3. 필요한 경우 스케일 조정
        scale = 1.0  # 스케일 팩터 (필요한 경우 조정)
        x_scaled = x_rotated * scale
        y_scaled = y_rotated * scale
        
        # 4. 시각화를 위한 오프셋 적용 (맵을 보기 좋은 위치로 이동)
        x_final = x_scaled + 22.5  # 맵의 중앙으로 이동
        y_final = y_scaled + 11.5
        
        return x_final, y_final
    
    def update(frame):
        """프레임별 업데이트"""
        try:
            latest_data = unity_interface.get_latest_data()
            if latest_data and "robot_pose" in latest_data:
                # SLAM 업데이트
                 # 로봇 위치 데이터 추출 및 변환
                original_x = latest_data["robot_pose"]["x"]
                original_y = latest_data["robot_pose"]["y"]
                # 좌표 변환 적용
                robot_x, robot_y = transform_coordinates(original_x, original_y)                
                
                 # Unity에서 이미 변환된 y값 사용
                robot_positions.append([robot_x, robot_y])
                
                # 로봇 경로 그리기
                positions = np.array(robot_positions)
                robot_path.set_data(positions[:, 0], positions[:, 1])
                
                # 현재 위치 표시
                current_pos.set_data([robot_x], [robot_y])
                
                # 로봇 방향 표시 업데이트 (방향도 90도 회전 적용)
                # orientation = latest_data["robot_pose"]["orientation"]
                # dx = np.cos(orientation - np.pi)  # 90도 회전 적용
                # dy = np.sin(orientation - np.pi)  # 90도 회전 적용
                # robot_direction.set_offsets(np.c_[robot_x, robot_y])
                # robot_direction.set_UVC(dx, dy)
                
                # SLAM 업데이트 및 예상 경로 표시
                slam.update_matrices(latest_data)
                if slam.solution is not None:
                    path = []
                    for i in range(slam.num_steps):
                        idx_x = i * 2
                        idx_y = idx_x + 1
                        path.append([slam.solution[idx_x, 0], slam.solution[idx_y, 0]])
                    path = np.array(path)
                    expected_path.set_data(path[:, 0], path[:, 1])
                # 랜드마크 위치 업데이트
                if "landmarks" in latest_data and latest_data["landmarks"]:
                    landmark_x = []
                    landmark_y = []
                    for lm in latest_data["landmarks"]:
                        # 상대 위치를 먼저 변환한 후 로봇 위치에 더하기
                        absolute_x = original_x + lm["relative_x"]
                        absolute_y = original_y + lm["relative_y"]
                        transformed_x, transformed_y = transform_coordinates(absolute_x, absolute_y)
                        
                        landmark_x.append(transformed_x)
                        landmark_y.append(transformed_y)
                    landmarks.set_data(landmark_x, landmark_y)
                slam.update_matrices(latest_data)
                
        except Exception as e:
            print(f"Error in update: {str(e)}")
            if 'latest_data' in locals():
                print(f"Latest data structure: {latest_data}")            
        return robot_path, landmarks, current_pos, expected_path#,robot_direction

    # 애니메이션 설정 및 시작
    ani = animation.FuncAnimation(
        fig, 
        update, 
        init_func=init,
        interval=1000,  # 1초 간격으로 업데이트
        blit=True, 
        cache_frame_data=False
    )

    plt.show()

if __name__ == "__main__":
    animate_slam()