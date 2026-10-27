# URRC_PROJECT

ROS 2 Jazzy + Gazebo Harmonic 기반 Team 01 LiDAR 자율주행 참가 패키지입니다.
카메라나 사전 맵/주행 좌표 없이 전방 5 m LiDAR ROI로 경계를 복원하고,
0.10 m 간격의 예측 centerline과 곡률를 사용해 조향합니다.

## 구조

```text
participant_ws/
├── team01/             # 제출 소스
│   ├── race_entry.json
│   └── src/team01_race/
└── team01-final.zip   # 공식 pack_entry.py로 생성한 제출 ZIP
```

주행 구조와 연습 명령은
[`participant_ws/team01/README.md`](participant_ws/team01/README.md)를 참고하세요.

## 깨끗한 빌드와 테스트

```bash
source /opt/ros/jazzy/setup.bash
export ROS_LOG_DIR=/tmp/team01_ros_log

colcon --log-base /tmp/team01_log build \
  --base-paths participant_ws/team01/src \
  --build-base /tmp/team01_build \
  --install-base /tmp/team01_install

colcon --log-base /tmp/team01_log test \
  --base-paths participant_ws/team01/src \
  --build-base /tmp/team01_build \
  --install-base /tmp/team01_install

colcon test-result --test-result-base /tmp/team01_build --verbose
```

실행 launch는 대회 실행기가 주입하는 토픽 인자를 그대로 사용합니다.

```bash
source /tmp/team01_install/setup.bash
ros2 launch team01_race race.launch.py
```

## 핵심 조건

- 운영 상태: `WAIT_START`, `RUNNING`, `FAULT`, `EMERGENCY_STOP`
- 전방 계획 거리: 5.0 m
- 동적 lookahead: 1.0–3.0 m
- 기본 목표 속도: 3.5 m/s
- 시작 조건: race start true + fresh LiDAR + valid trajectory + controller ready
- 공식 조향 부호: +우회전 / -좌회전

실제 대회 전에는 공식 practice 환경에서 마진, TTC, 최대 횡가속도를 차량 동역학에 맞게 최종 튜닝해야 합니다.
