# Team 01 URRC predictive LiDAR participant

이 폴더는 공식 URRC 실행기에서 독립적으로 접수할 수 있는 참가 패키지다. 기존 `urrc_racing`, `urrc_vehicle_description`, `urrc_vehicle_gazebo` 패키지는 workspace에 reference로 보존되어 있으며 제출물에는 포함되지 않는다.

## 주행 구조

```text
race_start + controller ready
        ↓
전방 5 m LiDAR ROI
        ↓
좌우 boundary 재구성 및 track width 추정
        ↓
0.10 m 간격 centerline trajectory
        ↓
0.5초 temporal fusion
        ↓
곡률 예측 + 1~3 m dynamic lookahead
        ↓
차량 크기 cluster 추적 및 통과공간 선택
        ↓
곡률·confidence·TTC 속도 제어
        ↓
Float32 speed / steering adapter
```

운영 상태는 `WAIT_START`, `RUNNING`, `FAULT`, `EMERGENCY_STOP`만 사용한다. 주행 코드는 odometry, map, GPS, IMU, world pose, track 이름, grid 좌표를 구독하거나 읽지 않는다. Monza 좌표는 연습용 spawn helper에만 있다.

차량 LiDAR의 물리 측정 한계는 6 m로 유지하고, perception과 planner가 전방 5 m만 사용한다. 기본 목표 속도는 3.5 m/s이며 곡률, trajectory confidence, obstacle TTC에 따라 낮아진다. 경계가 0.5 m라는 이유만으로 정지하지 않는다.

## 빌드와 테스트

```bash
cd ~/다운로드/urrc_2-main
source /opt/ros/jazzy/setup.bash
colcon --log-base participant_ws/log build \
  --base-paths participant_ws/team01/src \
  --build-base participant_ws/build \
  --install-base participant_ws/install
colcon --log-base participant_ws/log test \
  --base-paths participant_ws/team01/src \
  --build-base participant_ws/build \
  --install-base participant_ws/install
colcon --log-base participant_ws/log test-result \
  --test-result-base participant_ws/build --verbose
source participant_ws/install/setup.bash
```

## 공식 practice

터미널 1:

```bash
cd ~/다운로드/urrc_2-main/official_urrc
bash run_practice.sh --cars 0
```

터미널 2:

```bash
cd ~/다운로드/urrc_2-main
source participant_ws/install/setup.bash
export GZ_PARTITION=urrc_practice_$(id -u)
participant_ws/team01/src/team01_race/scripts/practice_spawn.sh
ros2 launch team01_race race.launch.py
```

터미널 3에서 로컬 출발 신호를 보낸다.

```bash
cd ~/다운로드/urrc_2-main
source participant_ws/install/setup.bash
participant_ws/team01/src/team01_race/scripts/practice_start.sh stop
participant_ws/team01/src/team01_race/scripts/practice_start.sh go
```

helper는 공식 `race.launch.py`에서 실행되지 않는다.

## 제출 ZIP

```bash
cd ~/다운로드/urrc_2-main/official_urrc
source /opt/ros/jazzy/setup.bash
python3 race/scripts/pack_entry.py ../participant_ws/team01 --output /tmp/team01.zip
cp /tmp/team01.zip race/inbox/
bash competition.sh prepare
bash competition.sh new-event --track monza
bash competition.sh qualifying
```

핵심 파라미터는 `src/team01_race/config/racing.yaml`에 있다. LiDAR 기반 상대 속도와 횡방향 여유는 실제 차량 변위의 직접 측정값이 아니므로 practice 차량을 이용해 cluster 크기, margin, TTC, lateral acceleration을 추가 조정해야 한다.
