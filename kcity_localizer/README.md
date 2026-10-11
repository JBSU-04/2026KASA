# kcity_localizer

MORAI **K-City 2025** 맵의 mgeo(HD맵) 기반 **차량 측위(localization)** ROS1 패키지.

런처 Unity 번들에서 mgeo 를 추출하고, MORAI 시뮬레이터의 `/gps`(+`/imu`)를 받아
맵 로컬좌표로 변환한 뒤 링크망에 정합하여 **현재 주행 중인 링크/노드/가로오프셋**을 인식한다.

## 데이터 출처

- 씬: `MoraiLauncher_Lin_카사/MoraiLauncher_Lin_Data/Bundle/v.S4.251001.MolitComp03/r_kr_pr_k-city_2025.scene`
- 좌표계: **UTM52N**, 원점 `local_origin_in_global = [302595.0, 4124145.0, 0.0]`
- 규모: node 480, link 895 (link 폴리라인 점 75,938)
- mgeo 포맷: maj_ver 3 / min_ver 0 (raw list 포맷)

추출은 이미 `data/` 에 완료되어 있다. 재추출하려면:

```bash
python3 scripts/extract_mgeo.py            # data/ 에 *_set.json 덤프
python3 scripts/extract_mgeo.py --list     # TextAsset 목록만
```
(UnityPy 필요. 862MB 씬 로드 시 RAM 사용 크므로 동시에 여러 번 돌리지 말 것.)

## 빌드 (ROS1 noetic, docker 컨테이너 `eungyun`)

순수 파이썬 패키지라 컴파일 불필요하지만 catkin 등록은 필요:

```bash
source /opt/ros/noetic/setup.bash
cd /home/kuuve/catkin_ws/catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="kcity_localizer"
source devel/setup.bash
```

런타임 의존: `python3-pyproj python3-scipy python3-numpy`, 그리고 `morai_msgs`(빌드되어 있어야 함).

## 실행

```bash
# MORAI /gps + /imu 입력 (기본)
roslaunch kcity_localizer localizer.launch

# 이미 /odom 을 발행 중이면 (예: gpsimu_parser)
roslaunch kcity_localizer localizer.launch use_odom:=true

# RViz 끄기
roslaunch kcity_localizer localizer.launch use_rviz:=false
```

## 토픽

입력:
- `/gps` (morai_msgs/GPSMessage) — lat/lon + east/northOffset(맵 원점)
- `/imu` (sensor_msgs/Imu) — heading
- `/odom` (nav_msgs/Odometry) — `use_odom:=true` 시

출력:
- `/localization/current_pose` (geometry_msgs/PoseStamped) — 현재 pose (frame=map)
- `/localization/current_link` (std_msgs/String) — 현재 주행 링크 idx
- `/localization/state` (std_msgs/Float32MultiArray) — `[x, y, yaw, lateral, s, max_speed, match_dist]`
  - `lateral`: 링크 중심선 기준 가로오프셋(좌+/우-, m)
  - `s`: 링크 시작부터 투영점까지 누적거리(m)
  - `match_dist`: 가장 가까운 링크까지 거리(m). 5m 초과면 도로 이탈로 간주(ego 마커 빨강)
- `/localization/map_markers` (visualization_msgs/MarkerArray, latched) — 링크망/노드
- `/localization/ego_marker` (visualization_msgs/Marker) — 차량 화살표
- `/odom` (nav_msgs/Odometry) — gps 입력일 때 하위호환용
- tf: `map -> base_link`

## 동작 원리

`/gps` 의 (lat,lon) → pyproj 로 UTM52N(x,y) → `(eastOffset, northOffset)` 차감 → 맵 로컬좌표.
이 좌표는 mgeo node/link 의 좌표와 동일 프레임이므로 별도 정합 보정이 필요 없다
(맵 원점이 GPS 메시지에 실려오기 때문에 맵을 바꿔도 코드 수정 불필요).
링크 폴리라인 정점 KD-tree(scipy)로 최근접 링크를 찾고 세그먼트에 투영해
현재 링크/노드/가로오프셋/진행거리를 산출한다.

## 구성

```
data/            추출된 mgeo JSON (node_set, link_set, global_info, ...)
scripts/
  extract_mgeo.py   Unity .scene -> mgeo JSON 추출
  mgeo_loader.py    JSON 로드 + KD-tree 링크 매칭 (단독 실행 시 self-test)
  map_localizer.py  ROS 측위 노드
launch/localizer.launch
rviz/localizer.rviz
```
