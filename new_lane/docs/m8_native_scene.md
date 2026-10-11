# M8 공통 인지 전달과 단계별 지도 갱신

## 적용 범위

M8의 내부 장면 전달만 네이티브 ROS 메시지로 바꾼다. MPC 설정, 다른 미션의 검출 파라미터, 속도 설정, 기준경로 코리도는 변경하지 않는다. 현재 GPS 3D 지도와 기존 벽 프로파일은 기본값으로 유지한다. 새 지도는 지연 정상화 확인 후 사용자가 확인한 **새 GPS 정상 빈 터널 주행 두 회**로 제작한다.

## 입력 흐름

`중력 보정 점군 + /nonground → 같은 스캔의 검증된 3D 배경 일치 제거 → 기존 공용 검출·필터·추적 → /tunnel/detections_native → 기존 머저의 물체별 보존 → /tunnel/detector_native → /tunnel/perception_scene_native`

최종 장면을 전진 계획, 정지, 선감속, 후진 판단에 전달한다. `/obstacle_info`는 같은 장면의 호환 출력이며 다시 머저의 관측 입력으로 돌려보내지 않는다. 내부 메시지에는 원본 시각, 좌표·지도 버전, 스캔 자세, 물체 ID와 원래 확정 상태, 실제 지지점과 보존 점유를 넣는다. 세계좌표는 float64이다. 잔여 속성용 작은 JSON은 유지하지만 점유·광선·점군을 JSON으로 왕복하지 않는다.

확정 상태나 높이를 만들어 넣지 않는다. 미확정 점유를 정적 확정으로 승격하지 않으며 실제 차체 근접 대응은 유지한다. 검색용 큰 상자의 빈 내부를 점유로 채우지 않는다. 원시 광선의 세 스캔 해제 증거와 해당 물체의 검증된 배경 일치 증거를 구분한다. 이미 해제한 오래된 추적 결과의 재발행은 새 관측 증거가 나오기 전까지 점유를 되살리지 않는다.

350ms 만료 기준과 배경 일치 거리 0.12m를 유지한다. 맵 검색 작업과 큐 잠금을 분리하고 최신 입력만 대기시킨다. 동일 스캔의 맵 비교 자세를 비지면 필터에서도 재사용한다. 지도 미확인·누락을 빈 공간으로 만들지 않는다.

## 실주행 확인

토탈 런치를 재시작해 **기존 지도에서 빈 터널 주행을 먼저 확인**한다. 자동으로 제어 런치를 시작하거나 백을 재생하지 않는다.

측정 토픽:

- `/tunnel/scene_processing_ms`: 장면 추가 처리 시간, p95 목표 50ms 이하.
- `/tunnel/scene_input_age_ms`: 장면 입력 시각 지연.
- `/tunnel/planning_input_age_ms`: 원본 스캔부터 계획 입력까지 지연, p95 목표 250ms 이하.
- `/tunnel/nonground_filter_state`: 배경 일치 제외 개수·유효 여부·처리 시간.

전방 준비 구간은 인지만 예열하고 M8 제어를 무장하지 않는다. 실제 센서 단절이나 차체 충돌 위험에 의한 정지는 지연 오정지와 구분한다. 합성 입력 검사는 실제 주행의 지연 또는 회피 성공을 입증하지 않는다.

## 새 지도 제작

아래 `NEW_*` 경로를 **사용자가 GPS 정상·장애물 없음으로 확인한 새 자료**로 교체한다. 기존 metadata는 좌표 원점만 참조하며 새 지도에 기존 장애물 주행 점군을 섞지 않는다.

```bash
cd /home/kuuve/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/new_lane/scripts/build_tunnel_static_map.py \
  --metadata src/new_lane/config/tunnel_static_gps_20261009.npz.json \
  --empty-pair /absolute/path/NEW_REGULAR_1.bag /absolute/path/NEW_HEAVY_1.bag \
  --empty-pair /absolute/path/NEW_REGULAR_2.bag /absolute/path/NEW_HEAVY_2.bag \
  --out /absolute/path/NEW_TUNNEL.npz \
  --wall-out /absolute/path/NEW_WALL_PROFILE.yaml
python3 src/new_lane/scripts/export_tunnel_runtime_map.py \
  /absolute/path/NEW_TUNNEL.npz /absolute/path/NEW_TUNNEL.bin
```

지도 제작은 현재 외부 파라미터와 실측 높이, 원본 시각 보간 자세, GPS normal 자료만 사용한다. 각 주행의 세 스캔 반복 관측과 두 주행의 0.12m 이내 일치를 요구한다. 모순·미관측 구역은 미확인이다. 물리적 벽·턱 프로파일은 차체 여유를 미리 빼지 않는다. 관측된 프로파일 끝에 zero knot를 넣고 입구·출구 열린 구간에 벽을 연장하지 않는다. 지도 보고서의 센서 해시와 버전을 함께 보관한다.

기하 검토 후 새 지도를 선택할 때:

```bash
roslaunch total_bringup competition.launch \
  tunnel_static_map_binary:=/absolute/path/NEW_TUNNEL.bin \
  tunnel_wall_profile_file:=/absolute/path/NEW_WALL_PROFILE.yaml \
  tunnel_physical_boundary_policy:=true
```

새 물리 경계 정책은 반폭 0.95m+여유 0.1m를 한 번만 적용한다. 입구 미리보기와 끊긴 실측 벽의 제한선 외삽을 사용하지 않는다. 관측된 정적 프로파일의 각 경계는 계속 사용한다. 벽 주변 점을 일괄 삭제하는 정책은 추가하지 않는다.

## 복귀 설정과 백업

네이티브 전달만 이전 JSON 전달로 복귀:

```bash
roslaunch total_bringup competition.launch tunnel_native_scene_enabled:=false
```

새 비지면 배경 필터만 끄기:

```bash
roslaunch total_bringup competition.launch tunnel_verified_background_filter_enabled:=false
```

이전 공통 장면 정책 자체로 복귀하려면 `tunnel_common_scene_enabled:=false`도 지정한다. 새 지도·물리 경계로 전환했으면 기본 기존 지도 경로와 `tunnel_physical_boundary_policy:=false`로 복귀한다. 설정 복귀 시에도 높이·확정 상태를 조작하는 버그 수정은 유지한다.

변경 전 파일과 기존 지도는 `analysis/m8_native_scene_20261010/backup/`에 보관했다. 같은 디렉터리에 MPC 전후 SHA-256, 빌드·단위·격리 연결 검사 결과를 저장한다. 백업 복원은 해당 파일을 선택해 수행하며 전체 src를 덮어쓰지 않는다.

## 검증 범위

Python 단위 검사, C++ 맵·계획 검사, 별도 ROS master의 합성 입력 연결 검사를 수행했다. 관측 상태, 실제 높이, 빈 스캔, 물체별 셀, 배경 일치 제외, 저상 돌출점 보존, 준비 구간 미무장, 세 스캔 해제와 재발행 방지, 새 물리 경계의 열린 입구 처리를 검사한다. 실제 빈 터널 지연 정상화, 새 두 주행 지도 제작, 저상·동적 장애물 반응과 폐루프 통과는 사용자 실주행 후 확인한다. 추가 백 재생은 하지 않는다.
