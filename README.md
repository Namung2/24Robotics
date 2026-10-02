# Probabilistic Localization and Navigation for a Mobile Robot

센서·모션 노이즈가 있는 환경에서 이동 로봇의 상태 추정과 자율 주행을 구현한 Unity–Python 시뮬레이션 프로젝트입니다. 로봇은 **Graph SLAM**으로 랜드마크 지도를 만들고, 초기 위치를 모르는 상태에서 **파티클 필터**로 자기 위치를 추정한 뒤, **A\* 경로 planning → 경로 smoothing → PID 추종**으로 목표 지점까지 이동합니다. 경로 planning은 위치 추정 신뢰도가 기준을 넘었을 때만 수행되며, 로봇은 자기 위치를 충분히 확신할 때만 움직입니다.

> 단국대학교 *로봇공학개론* 수업 프로젝트 (2024년 2학기)
> 상세 보고서: [`로봇 공학 개론 보고서 (1).pdf`](./로봇%20공학%20개론%20보고서%20(1).pdf)

## System Overview

<img width="1890" height="953" alt="Image" src="https://github.com/user-attachments/assets/bfdbdd38-9bbe-4079-8ecd-5b94430afdbe" />

시뮬레이터는 Box–Muller 변환으로 생성한 가우시안 노이즈를 모션과 센싱 양쪽에 주입합니다. 랜드마크 모드에서는 랜드마크까지의 거리가 멀수록 측정 노이즈가 커지고, 거리 센서(RangeFinder) 모드에서는 레이캐스트로 주변 장애물과 랜드마크까지의 거리를 측정합니다. 측정값은 매 스텝 JSON 형태로 TCP를 통해 Python 에이전트에 전달됩니다.

<img width="352" height="270" alt="Image" src="https://github.com/user-attachments/assets/b039fc61-75b1-495f-a4ef-c4229e02351b" />
## Components

### 1. Graph SLAM (`SLAM/`)
- 연속한 포즈 사이의 이동 제약과 랜드마크의 상대 좌표(x, y) 관측 제약을 정보 행렬 Ω와 정보 벡터 ξ에 누적합니다.
- 알려진 초기 포즈에 강한 사전 제약을 걸고 Ω μ = ξ를 풀어 로봇 궤적과 랜드마크 위치를 동시에 추정합니다.
- 로봇은 bicycle 모델을 따라 0.5 m씩 50스텝 전진하며, 매 스텝 조향각이 2π/50씩 증가합니다.

### 2. Particle-filter localization (`Localization_visual/`)
- 전역 불확실성에서 시작합니다. 파티클 1,000개를 25 × 15 m 맵 전체에 균일하게 분포시킵니다.
- 알려진 랜드마크까지의 거리 측정값에 가우시안 우도를 적용해 파티클 가중치를 갱신합니다.
- 유효 파티클 수(ESS)가 N/2 아래로 떨어지면 저분산(systematic) 리샘플링을 수행합니다.
- 파티클 집합의 위치 분산으로부터 신뢰도 점수를 계산합니다.

### 3. Planning and navigation (`Plan_Navi_/`, `Result/`)
- **신뢰도 기반 경로 planning**: 위치 추정 신뢰도가 0.8에 도달해야 경로를 planning하고, 주행 중 신뢰도가 0.8 아래로 떨어지면 경로를 다시 계획합니다.
- **A\***: 격자 맵에서 장애물과 랜드마크를 피해 경로를 탐색합니다.
- **경로 smoothing**: 원래 경로에 대한 충실도와 매끄러움을 균형 있게 반영하는 반복 갱신 방식에, 곡률 제한과 장애물로부터 거리에 반비례하는 반발력 항을 추가했습니다.
- **경로 추종**: lookahead 점을 목표로 PID 제어를 수행합니다(적분 항 anti-windup, 스텝 크기 제한 포함).
- `Plan_Navi_/`는 랜드마크만 사용하는 버전이고, `Result/`는 거리 센서(8 m), 장애물을 고려한 A\*, 개선된 smoothing를 추가한 버전입니다.

## Experiments

모든 결과는 시뮬레이터에서 얻은 것이며, 상세 내용과 그림은 보고서에 있습니다.

| 실험 | 관찰 |
|---|---|
| SLAM 노이즈 설정 변화 | 모션 노이즈를 크게 가정하면 추정이 측정값에 더 의존하고, 반대의 경우 모션 모델에 더 의존함. 실제 센서·모션 노이즈가 모두 있을 때는 궤적과 랜드마크 추정이 노이즈 없는 해와 눈에 띄게 어긋남. |
| 랜드마크 개수 (파티클 필터) | 신뢰도 0.8 도달 시간: 랜드마크 4개일 때 약 **3.0초**, 2개일 때 약 **5.5초**. |
| 센서 범위 5 m → 8 m, 장애물을 참조점에 추가 | 참조점이 늘어 위치 추정은 개선되었으나 연산량이 증가해 수렴 시간이 길어짐. |
| PID 게인 | Kd를 높이면 경유점 근처의 진동이 줄어듦. Kp·Ki가 높고 Kd가 낮으면 오버슈트와 진동이 발생. |
| smoothing 가중치 | 허용 오차를 충분히 작게 두면 data/smooth 가중치 설정과 무관하게 사실상 같은 경로로 수렴. |

## Repository Structure

```
24Robotics/
├── SLAM/                   # 문제 1: Graph SLAM
│   ├── SLAMmain.py         # 실행 파일 (실시간 플롯)
│   ├── graph_slam.py       # 정보 행렬 기반 Graph SLAM
│   └── UnityInterface.py   # TCP 서버 (포트 5000)
├── Localization_visual/    # 문제 2: 파티클 필터 위치 추정
│   ├── LandmarkLocalization.py
│   ├── LandmarkParticleFilter.py
│   ├── particle_filter_visualizer.py
│   └── UnityInterfaceFor23.py   # TCP 서버 (포트 5000)
├── Plan_Navi_/             # 문제 3: 경로 planning과 주행 (랜드마크만 사용)
│   ├── navigation_main.py
│   ├── smoothNavigator.py  # A*, smoothing, PID
│   ├── BaseLocalization.py
│   └── UnityInterfaceFor3.py    # TCP 서버 (포트 5002)
├── Result/                 # 문제 4–5: 거리 센서, 장애물 고려 path planning
│   └── (Plan_Navi_/와 동일한 구성)
└── Unity_C3_Script/        # Unity C# 스크립트
    ├── RobotController.cs
    ├── SensorSystem.cs
    ├── RobotDataTransmitter.cs
    ├── MapManager.cs
    └── GridManager.cs
```

## Running

**요구 사항:** Unity 2022.3 LTS (Newtonsoft Json 패키지 포함), Python 3.8+, `numpy`, `scipy`, `matplotlib`

```bash
pip install numpy scipy matplotlib
```

Python 쪽을 먼저 실행한 뒤, Unity에서 `SensorSystem`과 전송 컴포넌트의 `OperationMode`와 포트를 아래 표에 맞게 설정하고 Play를 누릅니다.

| 문제 | Python | Unity `OperationMode` | 포트 |
|---|---|---|---|
| Graph SLAM | `cd SLAM && python SLAMmain.py` | `SLAM` | 5000 |
| 위치 추정 | `cd Localization_visual && python LandmarkLocalization.py` | `Landmark` | 5000 |
| 경로 planning과 주행 | `cd Plan_Navi_ && python navigation_main.py` | `Landmark` | 5002 |
| 거리 센서 기반 주행 | `cd Result && python navigation_main.py` | `RangeFinder` | 5002 |

## Limitations

- Graph SLAM은 2차원 위치만 추정합니다. 방향은 추정하지 않고 알려진 모션 모델에서 가져오며, 그 덕분에 시스템이 선형으로 유지됩니다.
- 파티클 필터는 랜드마크 지도가 주어졌다고 가정하고, 거리 측정값만으로 포즈를 추정합니다.
- 파티클 필터의 예측 단계가 odometry(이동량) 대신 시뮬레이터가 보고한 포즈를 입력으로 사용합니다. 실제 로봇에 적용하려면 제어 입력 기반 예측으로 바꿔야 합니다.
- 신뢰도 점수 1 / (1 + 위치 분산)은 휴리스틱이며, 파티클이 여러 군집으로 나뉜 경우를 구분하지 못합니다.
- 경로 planning은 1 m 고정 격자와 알려진 장애물 지도 위에서 동작합니다.

## Author

조남웅 (Namwoong Cho) — 단국대학교 컴퓨터공학과
로봇공학개론 (최용근 교수님), 2024년 12월 제출
