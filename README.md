Robotics Project: SLAM, Localization & Navigation Implementation
로봇공학 기본 알고리즘들(SLAM, Localization, Planning & Navigation)을 Unity 시뮬레이션 환경에서 구현한 프로젝트입니다.
📋 프로젝트 개요
이 프로젝트는 로봇의 위치 추정과 경로 계획 문제를 해결하기 위한 시뮬레이션 시스템입니다. Unity와 Python을 활용하여 실제 로봇과 유사한 환경에서 다양한 로봇공학 알고리즘을 구현하고 테스트했습니다.
🎯 주요 목표

SLAM (Simultaneous Localization and Mapping): 로봇의 동시 위치 추정 및 지도 작성
Localization: 파티클 필터를 이용한 로봇 위치 추정
Planning & Navigation: A* 알고리즘과 PID 제어를 통한 경로 계획 및 추종

🏗️ 시스템 아키텍처
Unity 환경 (C#)

RobotController: 로봇의 움직임 제어 (Bicycle 모델)
SensorSystem: 랜드마크 및 장애물 센싱
RobotDataTransmitter: TCP 통신을 통한 실시간 데이터 전송
MapManager: 맵 및 장애물 관리

Python 환경

Graph SLAM: 그래프 기반 SLAM 알고리즘
Particle Filter: 파티클 필터 기반 위치 추정
A Planning*: 최적 경로 탐색
PID Navigation: 경로 추종 제어

🗂️ 프로젝트 구조
📁 Robotics-Project/
├── 📁 SLAM/                      # 문제 1: Graph SLAM
│   ├── SLAMmain.py               # SLAM 메인 실행 파일
│   ├── graph_slam.py             # Graph SLAM 알고리즘
│   └── UnityInterface.py         # Unity 통신 인터페이스
│
├── 📁 Localization_visual/       # 문제 2: Particle Filter Localization
│   ├── LandmarkLocalization.py   # 메인 실행 파일
│   ├── LandmarkParticleFilter.py # 파티클 필터 구현
│   └── particle_filter_visualizer.py # 시각화
│
├── 📁 Plan_Navi_/               # 문제 3: Planning & Navigation
│   ├── navigation_main.py        # 메인 실행 파일
│   ├── smoothNavigator.py        # A* + Smoothing + PID
│   └── BaseLocalization.py       # 기본 위치 추정
│
├── 📁 Result/                   # 문제 4,5: 고급 기능
│   ├── navigation_main.py        # 장애물 회피 포함
│   └── smoothNavigator.py        # 개선된 스무딩 알고리즘
│
└── 📁 Unity_C3_Script/          # Unity C# 스크립트
    ├── RobotController.cs        # 로봇 제어
    ├── SensorSystem.cs          # 센서 시스템
    ├── RobotDataTransmitter.cs  # 데이터 전송
    └── MapManager.cs            # 맵 관리
🚀 실행 방법
1. 환경 설정
Python 패키지 설치:
bashpip install numpy scipy matplotlib threading socket json
Unity 설정:

Unity 2022.3 LTS 이상 권장
Newtonsoft Json 패키지 설치

2. 문제별 실행 가이드
문제 1: SLAM
bash# Python 측
cd SLAM/
python SLAMmain.py

# Unity 측: OperationMode를 SLAM으로 설정
문제 2: Localization
bash# Python 측
cd Localization_visual/
python LandmarkLocalization.py

# Unity 측: OperationMode를 Landmark로 설정
문제 3: Planning & Navigation
bash# Python 측
cd Plan_Navi_/
python navigation_main.py

# Unity 측: OperationMode를 Landmark로 설정, Port: 5002
문제 4,5: 고급 기능
bash# Python 측
cd Result/
python navigation_main.py

# Unity 측: OperationMode를 RangeFinder로 설정, Port: 5002
⚙️ 주요 파라미터
로봇 파라미터

맵 크기: 25 x 15 격자
셀 크기: 1m x 1m
초기 위치: (22.5, 11.5, 1.5π) - SLAM 모드
이동 속도: 0.5m/step
조향각: 2π/50 rad/step (SLAM)

노이즈 설정

Motion Noise: 0.1 (기본)
Measurement Noise: 0.4 (기본)
가우시안 노이즈: Box-Muller 변환 사용

PID 제어 파라미터

Kp (비례 게인): 0.3
Ki (적분 게인): 0.05
Kd (미분 게인): 0.4

📊 주요 알고리즘
1. Graph SLAM

정보 행렬(Ω)과 정보 벡터(ξ) 기반
랜드마크 관측 및 모션 모델 통합
의사역행렬을 통한 robust한 해 계산

2. Particle Filter

1000개 파티클 사용
가중치 기반 리샘플링
유효 파티클 수(n_eff) 기반 적응적 리샘플링

3. A* 경로 계획

유클리드 거리 휴리스틱
격자 기반 경로 탐색
장애물 회피 기능

4. 경로 스무딩

선형 시스템 기반 스무딩
곡률 제한 기능
장애물 회피력 포함 (고급 버전)

📈 실험 결과
노이즈 영향 분석

노이즈 없음: 완벽한 위치 추정 및 경로 추종
하드웨어 노이즈만: 약 41% 오차 발생
SLAM 노이즈 큼: 센싱 데이터 의존성 증가
측정 노이즈 큼: 액션 데이터 의존성 증가

랜드마크 개수별 성능

4개 랜드마크: 약 3초 수렴
2개 랜드마크: 약 5.53초 수렴
랜드마크 개수 감소 시 수렴 속도 저하 확인

🔧 트러블슈팅
연결 문제

Unity-Python 연결 실패: 포트 번호 확인 (5000, 5002)
데이터 전송 오류: JSON 형식 및 TCP 연결 상태 확인

성능 문제

파티클 필터 수렴 느림: 파티클 수 조정 또는 노이즈 파라미터 튜닝
PID 진동: Kd 값 증가로 미분 제어 강화

시각화 문제

matplotlib 실시간 업데이트: plt.ion() 및 plt.pause() 사용
Unity 디버그 라인: showDebugLines 옵션 활성화

📚 참고 자료

Probabilistic Robotics (Sebastian Thrun, Wolfram Burgard, Dieter Fox)
Planning Algorithms (Steven M. LaValle)
Unity 공식 문서
ROS Navigation Stack

👥 기여자

조남웅 - 컴퓨터공학과, 학번: 32224332
지도교수: 최용근 교수님
과목: 로봇공학개론

📄 라이선스
이 프로젝트는 교육 목적으로 제작되었습니다.

제출일: 2024.12.13
프로젝트 기간: 2024.11 - 2024.12
