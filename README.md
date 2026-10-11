# catkin_ws/src — K-City 통합 스택

`src_0901`(고주로+기본주행 MPC)을 기반으로 5개 트리를 하나로 병합한 워크스페이스다.
구조는 기존 `src_0901`과 같은 **평면 배치**를 유지한다 — 패키지가 최상위에 그대로 나열된다.

- 롤백용 원본 트리: `catkin_ws/src_병합전/` (빌드 대상 아님)
- 병합에서 뺀 패키지: `catkin_ws/src_제외/` (빌드 대상 아님)

---

## 담당 영역별 패키지

### 통신 — MORAI 경계
| 패키지 | 역할 | 출처 |
|---|---|---|
| `morai_udp_bridge` | UDP 7채널 송수신. `/ctrl_cmd` 송신 포함 | ekf |

규정 2-8의 허용 채널은 UDP 7종뿐이다. rosbridge 경로는 쓰지 않는다.

### 인지 — LiDAR
| 패키지 | 역할 | 출처 |
|---|---|---|
| `lidar_ttc_tracker` | **트래킹 정본.** `/tracked_objects` 발행 | 0901 |
| `lidar_perception_pkg` | 지면제거·클러스터링·추적 (음영구간용) | 음영 |
| `roundabout_lidar_tracker` | 회전교차로 원형 ROI 전용 추적 | 교차로 |
| `perception_adapters` | 정본 트랙을 타 미션 메시지 타입으로 변환 | **신규** |

### 인지 — 카메라
| 패키지 | 역할 | 출처 |
|---|---|---|
| `lane_perception_pkg` | TwinLiteNet 차선 세그멘테이션 + sliding window 조향 | 음영 |
| `yolo_test` | 장애물 인지, 카메라 속도킥 | 정동적 |
| `yolo_distance_pkg` | 신호등 YOLO 모델 보관 | ekf |

### 측위
| 패키지 | 역할 | 출처 |
|---|---|---|
| `gps_common` | `/gps` → UTM odometry | 0901 |
| `odom_information` | **`/odom`과 `map→base_link` TF의 유일한 소유자** | 0901 |
| `kcity_localizer` | mgeo 기반 정지선·신호등 정보 제공 | ekf |
| `ekf_localizer` · `morai_sensor_fusion` · `amathutils_lib` | GPS·IMU 노이즈 대응 EKF (옵션) | ekf |
| `map_loader` | mgeo HD맵 로드 | ekf |

`kcity_localizer`는 `publish_odom:=false`, `publish_tf:=false`로 띄운다.
`/odom` publisher가 둘이면 TF가 진동한다.

### 경로
| 패키지 | 역할 | 출처 |
|---|---|---|
| `waypoint_system` | CSV 웨이포인트 로드 → 지역경로 후보 | 0901 |
| `waypoint_maker` | `Lane` / `State` / `Waypoint` 메시지 | 0901 |
| `planning_mux` | **`/local_path`의 유일한 publisher.** 후보 중 하나를 승격 | **신규** |

### 판단 · 미션
| 패키지 | 역할 | 출처 |
|---|---|---|
| `mission_planner_msgs` | `BehaviorConstraint` 계약 | 교차로 |
| `mission_manager` | 활성 제약 합성 → `/mission/behavior_constraint` | 교차로 |
| `mission_planner` | 회전교차로 상태기 (검증값 유지, 수정 금지) | 교차로 |
| `mission_pkg` | 정적·동적 장애물 미션 | 음영 |
| `mission_supervisor` | 구간 라우터 · 속도캡 변환 · 시퀀스 관측기 | **신규** |

새 미션은 `/mission/constraints/<name>`으로 `BehaviorConstraint`만 발행하고
`mission_manager.yaml`의 `input_topics`에 등록한다. 조향이나 `/ctrl_cmd`를 직접 건드리지 않는다.

### 행동
| 패키지 | 역할 | 출처 |
|---|---|---|
| `smpc_lane_change` | 고주로 차선변경 결정 + ACC 속도계획 | 0901 |
| `new_lane` | Frenet 정적 장애물 회피 (`obstacle_avoidance_node`) | 0901 + 정동적 |

### 제어
| 패키지 | 역할 | 출처 |
|---|---|---|
| `mpc_path_tracking` | **종·횡 주 실행기.** `/control/raw_cmd` 발행 | 0901 |
| `control_pkg` | 음영구간(M8) 전용 제어 | 음영 |
| `aeb_safety` | AEB 판단 + **`/ctrl_cmd` 최종 중재** | ekf |
| `erp_driver` · `serial` | 실차 인터페이스 (시뮬에서는 빌드 의존만) | 0901 |
| `CppAD` · `Ipopt_pkg` | MPC 솔버. `mpc_path_tracking`이 상대경로로 참조 | 0901 |

`CppAD`와 `Ipopt_pkg`는 `mpc_path_tracking/CMakeLists.txt`가 `../CppAD`,
`../Ipopt_pkg`로 참조한다. **최상위에서 옮기면 빌드가 깨진다.**

### 메시지
| 패키지 | 역할 |
|---|---|
| `morai_msgs` | MORAI 인터페이스 (91개, 음영구간판이 정본) |
| `non_gps_msgs` | 음영구간 자체 계약 |
| `ackermann_msgs` | — |

`EgoVehicleStatus`는 트리마다 필드 순서가 달랐다. 91개판으로 통일했으므로
다른 버전을 섞으면 MD5 불일치로 통신이 끊긴다.

### 런치
| 패키지 | 역할 |
|---|---|
| `total_bringup` | 통합 진입점, 키보드 오버라이드, 텔레포트 |
| `non_gps_bringup` | 음영구간 스택 런치 (참조용) |

---

## 단일 publisher 계약

아래 토픽은 **발행자가 정확히 하나**여야 한다. 통합 검증의 첫 관문이다.

```bash
rostopic info /ctrl_cmd     # control_arbiter 하나여야 한다
rostopic info /local_path   # planning_mux
rostopic info /odom         # odom_information
rosrun tf tf_monitor        # base_link 부모가 하나인가
```

| 토픽 | 유일 publisher |
|---|---|
| `/ctrl_cmd` | `aeb_safety/control_arbiter_node.py` |
| `/control/raw_cmd` | `mpc_path_tracking` (remap) |
| `/local_path` | `planning_mux` |
| `/odom` + TF | `odom_information` |
| `/tracked_objects` | `lidar_ttc_tracker` |
| `/mission/behavior_constraint` | `mission_manager` |

---

## 확정된 공통 값

| 항목 | 값 | 근거 |
|---|---|---|
| LiDAR 장착 | `x 1.00, z 1.25` | 팀 확정 |
| 차량 | Ioniq5 · 축거 3.000 m · 최대 조향 40° | 규정 2-6-1 |
| 제어 명령 | cmd type **1**(accel/brake), ctrl mode **2** | 규정 2-8 |
| 도심 제한 | 60 km/h (고주로 구간 예외) | 규정 |
| 제한 시간 | 15분 | 규정 |

값이 여러 파일에 흩어져 불일치가 반복됐다. 새로 값을 넣을 때는 이 표를 먼저 확인한다.

---

## 실행

```bash
roslaunch total_bringup competition.launch sim_ip:=127.0.0.1
```

레이어별로 켜고 끌 수 있다. 단계 검증 순서는 다음과 같다.

| 단계 | 명령 | 확인할 것 |
|---|---|---|
| 기본 주행 | `enable_mission:=false enable_behavior:=false` | 랩 완주, `/ctrl_cmd` publisher 1개 |
| 라우터 확인 | 위 + `enable_mission:=true` | `/mission/sequence_state` 가 M1→M8 순서로 지나가는가 |
| 미션 추가 | `missions.yaml` 의 `executors` 를 하나씩 채움 | 미션별 단독 시나리오 |
| 고주로 | `enable_behavior:=true` | M6/M7 에서만 차선변경 |

`dry_run:=true` 로 띄우면 `control_arbiter` 를 실행하지 않는다. UDP 브리지가
`send_before_first_cmd: false` 라 차량이 움직이지 않으므로, 전 노드 준비를
확인한 뒤 주행을 시작하는 절차에 쓴다.

### 새로 만든 노드

| 노드 | 패키지 | 역할 |
|---|---|---|
| `section_router` | mission_supervisor | 위치 → 현재 미션 구간. GPS 상실 시 속도 적산 폴백 |
| `constraint_to_speed_cap` | mission_supervisor | `BehaviorConstraint` → 속도 상한 하나 |
| `sequence_monitor` | mission_supervisor | 미션 순서·체류시간 기록 (**차단 안 함**) |
| `local_path_mux` | planning_mux | 경로 후보 중 하나를 `/local_path` 로 승격 |
| `tracked_object_adapter` | perception_adapters | 정본 트랙 → 타 미션 메시지 타입 |
| `control_arbiter` | aeb_safety | 소스 선택 + AEB + 속도리미터. `/ctrl_cmd` 단독 |

구간 정의는 `total_bringup/config/missions.yaml` 에 있다.
**M5·M6·M7 은 규정 링크 ID 로 확정**됐고, M1~M4·M8 은 인덱스 범위가 미확정이라
`executors` 를 비워 두었다 — 채우기 전에는 해당 미션이 무장되지 않는다.

## 빌드

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
```

메시지 패키지만 먼저 확인하려면:

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="morai_msgs;non_gps_msgs;\
mission_planner_msgs;mission_supervisor;waypoint_maker;lidar_ttc_tracker;\
smpc_lane_change;aeb_safety;erp_driver"
```

전체 빌드로 되돌릴 때는 화이트리스트를 비운다:

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```
