# 2026 KASA 자율주행

2026 K-City 자율주행 경진대회 통합 ROS 패키지

## 주요 패키지 구성
- morai_udp_bridge: 시뮬레이터 UDP 7채널 송수신
- lidar_ttc_tracker: 라이다 객체인식 및 TTC 트래킹 (/tracked_objects)
- lane_perception_pkg: UFLD / TwinLiteNet 차선인식
- yolo_test: 객체인식 및 카메라 감속 제어
- yolo_distance_pkg: 신호등 및 거리 추정 모델
- odom_information: 오도메트리 통합 및 TF (/odom, map -> base_link)
- waypoint_system / waypoint_maker: 웨이포인트 경로 생성 및 로더
- planning_mux: 주행/회피 경로 승격 (/local_path)
- mission_planner: 회전교차로 상태기 및 Gap 계산
- mission_supervisor: 미션 구간 라우팅 (section_router)
- new_lane / smpc_lane_change: 차선변경 및 장애물 회피
- mpc_path_tracking: MPC 종횡방향 경로추종
- aeb_safety: AEB 판단 및 최종 제어 중재 (/ctrl_cmd)

## 미션별 주요 로직 및 개발 메모

### 고주로
- 차선변경 양보차량 조건 앞뒤 범위 히스테리시스 조건 충족 추가
- 가속 천천히 하도록 수정 (테스트 진행)
- 차량 간격 TTC 기준으로 관리

### 정적 / 동적 장애물
- 정적 장애물 구간 신호 대기 시 장애물이 기준경로에 침범해 있는 정도에 따라 정지거리 유동적으로 변경
- 장애물 회피 후 기준경로로 복귀하는 성향 적용
  - 긴 버스 등 차폐 장애물: 전면 회피 후 측면 포인트 찍히면 회피 방향 유지하며 경로 연장
- 정적 장애물 및 교차로 구간 연석 표시 곡선화 (이산적 형태 대신 자연스러운 궤적)
- 회피경로 생성: 기존 Frenet (d, s) 대신 Apollo 기반으로 변경
  - NMPC가 곡률 기반 속도 프로파일을 사용하므로 Apollo 생성 경로 스무딩 필수 (작은 선분 연결 형태 완화)

### 회전교차로
- 고주로 구간과 유사한 앞뒤 Gap Select 방식으로 수정

### 음영구간 (터널)
- 차선추종: 준혁 데이터 기반 UFLD v2 학습 모델 적용
- 랜덤 장애물: Dead Reckoning (DR) 기반 처리
- 동적 장애물이 들어와서 앞에 정지했을 때 회피 반경 부족 시 후진으로 회피 반경 확보
  - 후진 경로는 Hybrid A* 적용 고려 중

## 핵심 토픽
- /ctrl_cmd: aeb_safety/control_arbiter (최종 제어 단독 발행)
- /control/raw_cmd: mpc_path_tracking
- /local_path: planning_mux
- /odom: odom_information (TF map -> base_link 단독)
- /tracked_objects: lidar_ttc_tracker
- /mission/behavior_constraint: mission_manager

## 빌드 및 실행

### 빌드
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 실행
```bash
# 통합 실행
roslaunch total_bringup competition.launch sim_ip:=127.0.0.1

# 옵션
# enable_mission:=false (기본 주행만)
# enable_behavior:=true (고주로 차선변경 활성화)
# dry_run:=true (제어 명령 미전송)
```

## 주의사항
- morai_sensor_fusion/models/tunnel_ufld_res18.pth (207MB) 모델 파일은 용량 초과로 git 제외됨. 로컬 해당 경로에 직접 배치하고 사용.
- CppAD, Ipopt_pkg는 src/ 최상위에 위치해야 빌드 가능.
