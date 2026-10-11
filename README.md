# 2026 KASA Autonomous Driving Stack (K-City)

2026 대학생 자율주행 경진대회(KASA) 참가를 위한 ROS 1 Noetic 기반 K-City 통합 자율주행 소프트웨어 스택 저장소입니다.

센서 인지(LiDAR/Camera), 정밀 측위(GPS/IMU/HD Map), 전역 및 지역 경로 계획, 미션 상태 제어, MPC 기반 차량 제어 및 AEB 안전 중재 모듈을 포함하고 있습니다.

---

## 📦 주요 패키지 구성

### 1. 통신 및 인터페이스 (Bridge & Hardware)
- `morai_udp_bridge`: MORAI 시뮬레이터와 UDP 7개 채널 양방향 통신 (제어 명령 송신 및 센서 데이터 수신)
- `morai_msgs`: MORAI 시뮬레이터 연동 전용 ROS 메시지
- `erp_driver` / `serial`: ERP42 실차 플랫폼 시리얼 통신 인터페이스

### 2. 센서 인지 (Perception)
- **LiDAR**
  - `lidar_ttc_tracker`: LiDAR 포인트클라우드 장애물 검출, TTC 계산 및 객체 추적 (`/tracked_objects` 발행)
  - `lidar_perception_pkg`: 포인트클라우드 전처리, 지면 제거 및 클러스터링 (음영/터널 구간용)
  - `roundabout_lidar_tracker`: 회전교차로 원형 ROI 전용 주변 차량 추적
  - `perception_adapters`: 정본 트래커 메시지를 각 미션별 입력 메시지 포맷으로 변환
- **Camera**
  - `lane_perception_pkg`: TwinLiteNet 딥러닝 기반 차선 세그멘테이션 및 슬라이딩 윈도우 조향 추출
  - `yolo_test`: YOLOv11s 기반 객체 인식 및 전방 장애물 감속 제어
  - `yolo_distance_pkg`: 신호등 인식 및 전방 거리 추정 모델 관리

### 3. 측위 및 맵 (Localization & Mapping)
- `odom_information`: Odometry 통합 및 `map -> base_link` TF 발행
- `gps_common`: GPS WGS84 위경도 좌표 → UTM Odometry 변환
- `kcity_localizer`: mgeo HD 맵 기반 정지선, 차선 및 신호등 정보 연계
- `ekf_localizer` / `morai_sensor_fusion` / `amathutils_lib`: GPS/IMU 센서 융합 확장 칼만 필터(EKF)
- `map_loader`: mgeo 정밀 도로 지도(HD Map) 로더
- `tunnel_xicp`: GPS 음영(터널) 구간 2D/3D LiDAR ICP 기반 상대 측위

### 4. 경로 계획 (Planning)
- `waypoint_system`: 전역 경로(CSV) 로드 및 지역 경로 후보군 생성
- `waypoint_maker`: 웨이포인트 생성, 스플라인 곡선 보간 및 경로 저장 도구
- `planning_mux`: 주행 모드 및 미션 상태에 따른 최적 경로 승격 (`/local_path` 최종 발행)
- `smpc_lane_change`: 고속/고주로 구간 차선 변경 판단 및 ACC 속도 계획
- `new_lane`: Frenet 프레임 기반 정적/동적 장애물 회피 경로 생성

### 5. 미션 판단 및 감독 (Mission & Behavior)
- `mission_planner`: 회전교차로(진입/회전/탈출) 상태 머신 및 Gap Acceptance 진입 판단
- `mission_manager`: 각 미션별 제약 조건을 취합하여 `/mission/behavior_constraint` 종합 발행
- `mission_supervisor`: 미션 구간 라우팅(`section_router`), 속도 상한 변환 및 시퀀스 상태 모니터링
- `mission_pkg`: 정적 및 동적 장애물 미션 시나리오 처리
- `mission_planner_msgs`: 미션 행동 제약 조건(`BehaviorConstraint`) 인터페이스 메시지

### 6. 제어 및 안전 (Control & Safety)
- `mpc_path_tracking`: MPC(Model Predictive Control) 기반 종·횡방향 통합 제어기 (`/control/raw_cmd`)
- `aeb_safety`: 전방 충돌 감지 긴급제동(AEB) 및 최종 차량 제어 명령 중재(`control_arbiter`)
- `control_pkg`: GPS 음영 구간 전용 제어기
- `CppAD` / `Ipopt_pkg`: MPC 비선형 최적화 솔버 라이브러리

---

## 🚗 시스템 아키텍처 및 토픽 파이프라인

```
[ MORAI Sim / Sensors ]
           │
           ▼
[ Perception & Localization ]  (LiDAR / Camera / GPS / EKF / ICP)
           │
           ▼
[ Mission & Supervisor ]       (Section Router, Roundabout, Constraints)
           │
           ▼
[ Planning & Mux ]             (Waypoint / Frenet Avoidance / Lane Change)
           │
           ▼
[ MPC Path Tracking ]          (Raw Control Command)
           │
           ▼
[ AEB Safety Arbiter ]         (Collision Check & Velocity Limiter)
           │
           ▼
[ Final /ctrl_cmd ] ─────────► [ Vehicle / Simulator ]
```

### ⚠️ 핵심 토픽 단일 발행자 원칙
시스템의 일관성과 TF 진동 방지를 위해 아래 토픽은 반드시 지정된 단일 노드에서만 발행됩니다:
- `/ctrl_cmd`: `aeb_safety` (최종 중재 노드)
- `/control/raw_cmd`: `mpc_path_tracking`
- `/local_path`: `planning_mux`
- `/odom` 및 `map -> base_link` TF: `odom_information`
- `/tracked_objects`: `lidar_ttc_tracker`
- `/mission/behavior_constraint`: `mission_manager`

---

## ⚙️ 공통 환경 설정 및 제원

| 항목 | 설정값 | 비고 |
|---|---|---|
| 기준 차량 | Ioniq 5 | 축거: 3.000 m / 최대 조향각: 40° |
| LiDAR 장착 위치 | `x: 1.00 m, z: 1.25 m` | 차량 원점 기준 |
| 제어 모드 | Cmd Type: **1** (Accel/Brake), Ctrl Mode: **2** | 규정 2-8 준수 |
| 최고 속도 제한 | 60 km/h | 고주로 구간 별도 속도 계획 적용 |
| 통신 방식 | MORAI UDP 7개 전용 채널 | Rosbridge 미사용 |

---

## 🛠️ 빌드 방법 (Build)

```bash
# 1. 워크스페이스 이동 및 환경 로드
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash

# 2. 전체 패키지 빌드
catkin_make

# 3. 환경 변수 등록
source devel/setup.bash
```

> **메시지 패키지만 먼저 빌드할 경우:**
> ```bash
> catkin_make -DCATKIN_WHITELIST_PACKAGES="morai_msgs;non_gps_msgs;mission_planner_msgs;mission_supervisor;waypoint_maker;lidar_ttc_tracker;smpc_lane_change;aeb_safety;erp_driver"
> # 전체 빌드로 원복 시
> catkin_make -DCATKIN_WHITELIST_PACKAGES=""
> ```

---

## 🚀 실행 방법 (Execution)

### 1. 통합 자율주행 실행
```bash
roslaunch total_bringup competition.launch sim_ip:=127.0.0.1
```

### 2. 단계별 검증 옵션
- **기본 주행만 테스트:**
  ```bash
  roslaunch total_bringup competition.launch enable_mission:=false enable_behavior:=false
  ```
- **미션 라우터 및 상태기 테스트:**
  ```bash
  roslaunch total_bringup competition.launch enable_mission:=true enable_behavior:=false
  ```
- **고주로 차선변경 포함 전체 주행:**
  ```bash
  roslaunch total_bringup competition.launch enable_mission:=true enable_behavior:=true
  ```
- **Dry-run (제어 명령 미전송, 센서/노드 구동 확인용):**
  ```bash
  roslaunch total_bringup competition.launch dry_run:=true
  ```

---

## 📌 주의사항
1. **대용량 가중치 파일 관리**:
   - `morai_sensor_fusion/models/tunnel_ufld_res18.pth` (207MB) 모델 가중치는 GitHub 단일 파일 용량 제한(100MB)으로 인해 Git 추적에서 제외되어 있습니다.
   - 로컬 구동 시 해당 파일이 지정된 위치에 존재하는지 확인해 주세요.
2. **솔버 의존성 경로**:
   - `CppAD` 및 `Ipopt_pkg`는 최상위 `src/` 경로에 위치해야 `mpc_path_tracking` 빌드가 정상적으로 수행됩니다.
