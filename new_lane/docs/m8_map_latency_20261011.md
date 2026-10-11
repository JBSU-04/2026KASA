# M8 맵 처리 지연·점유 보존·X-ICP 전용 모드

## 이번 적용

M8의 지도 검색과 전달을 개선했다. 현재 GPS 지도·벽 지도, MPC 설정, 다른 미션의 파라미터, 코리도 1.3m, 배경 거리 0.12m와 인지 만료 350ms는 유지한다. 실제 주행의 지연 목표 충족 여부는 아직 확인하지 않았다.

지도 비교는 Autoware의 pinned voxel-distance 알고리즘을 ROS1 C++에 적용했다. 원본 `voxel_grid_map_loader.cpp`와 해시는 `vendor/autoware_compare_map/UPSTREAM.json`에 보존했다. 원본 ROS2 노드를 실행하지 않는다. 긴 터널의 dense PCL leaf layout을 sparse 0.05m voxel index로 바꾸고, 같은 복셀의 최대 대각선보다 검색 문턱이 클 때만 빠른 일치를 인정한다. 그 외에는 원본 지도 점의 radius search를 사용한다. 0.055m 노면 판정은 정확한 거리 검색을 유지한다. 입력 점을 다운샘플링하지 않는다.

Raw와 비지면 처리에서 같은 스캔 자세를 사용하고 동일 지도 질의 결과를 재사용한다. 맵 색인·노면 색인은 시작 시 구성한다. 원본 시각은 바꾸지 않으며 X-ICP 결과를 기다리지 않는다.

M8 머저는 입력 콜백과 단일 계산 작업자를 분리했다. 대기 입력은 최신 스캔 하나이며 계산 중 입력 잠금을 잡지 않는다. 경로 선분 검색과 반환 광선 확인은 double precision C++ 커널로 수행한다. 광선 해제는 실제 반환점이 보존 표면을 통과한 세 스캔을 요구하고, 광선이 없거나 가려진 곳을 빈 공간으로 만들지 않는다. 큰 지지점·광선 배열의 중복 복사를 줄였다.

## 판단 변경

`motion_state`와 `static_certified`를 분리했다. 검증된 정적 부모는 `unknown`으로 바뀌어도 계획·정지·탈출에서 같은 실제 점유를 사용한다. 신규 미확정 물체에 정적 증명을 만들어 넣지 않는다. 검증된 배경·실제 광선 확인·통과에 따른 기존 해제와 실제 이동 증거에 따른 동적 전환을 유지한다.

선감속은 기존 20m·6m/s를 유지한다. 미확정 물체가 좌우 감시 범위 안에 있다는 이유만으로 제한하지 않고, 승인 경로 또는 신선한 기준경로의 차체 이동 영역과 비교한다. 독립 이동 증거가 있는 대상은 최대 4초의 이동 예측도 확인한다. 속도가 미확정이면 현재 실제 점유로 판단하며, 현재 차체 위험과 판단 정보 부족은 별도로 유지한다. 안전하게 피하는 확정 정적 대상의 제한은 해제한다.

## 첫 주행과 X-ICP 전환

첫 성능 확인용 기본값은 **DR**이다. 이때 UFLD 모델과 X-ICP 노드는 실행하지 않으며 지도 기반 인지·기존 회피는 계속 실행한다.

```bash
roslaunch total_bringup competition.launch
```

빈 터널의 지연·오회피·반복 정지를 확인한 뒤, 런치를 재시작하면서 다음을 지정한다.

```bash
roslaunch total_bringup competition.launch tunnel_heading_mode:=xicp
```

X-ICP는 기존 원본 백엔드·벽 지도·정합 품질 기준을 사용한다. 검증된 yaw 결과로 IMU/DR의 bias만 보정하고 회피 중 실제 회전을 유지한다. 입력은 300ms 이내이며 보정은 최대 0.2도/관측으로 연결한다. 불량·만료 시 마지막으로 보정한 DR을 유지하고 벽 직선·카메라로 자동 전환하지 않는다. M8 미션 상태가 신선할 때만 헤딩 보정을 적용한다.

X-ICP 모드에서 지도 필터가 X-ICP의 comparison pose를 별도로 다시 적용하지 않도록 했다. 지도 비교는 스캔 자세와 기존 맵 조회용 벽 정렬을 사용하며, 이미 반영된 yaw를 이중 적용하지 않는다. GPS 재획득 좌표 버전 검사는 그대로 유지한다.

UFLD는 `tunnel_heading_mode:=ufld`, 이전 혼합 헤딩 정책은 `tunnel_heading_mode:=existing`으로 수동 복귀한다. DR/X-ICP 모드에서는 UFLD 추론이 실행되지 않는다.

## 측정과 RViz

| 토픽 | 의미 | 실주행 p95 목표 |
|---|---|---:|
| `/tunnel/map_total_processing_ms` | 같은 스캔 Raw+비지면 맵 계산 | 60ms |
| `/tunnel/map_input_age_ms` | Raw가 맵 계산을 시작할 때 원본 시각 대비 지연 | 별도 원인 분석 |
| `/tunnel/merger_processing_ms` | 머저 추가 계산 | 30ms |
| `/tunnel/merger_queue_ms` | 콜백 수신 후 계산 시작까지 대기 | 별도 원인 분석 |
| `/tunnel/scene_processing_ms` | 공통 장면 추가 계산 | 10ms |
| `/tunnel/planning_input_age_ms` | 원본 스캔부터 계획 입력까지 | 250ms |

`map_total_processing_ms`는 같은 스캔의 두 계산이 끝났을 때만 발행한다. 건너뛴 스캔의 지연이 0인 것처럼 기록하지 않는다. 입력 갱신 빈도와 만료 횟수도 함께 확인한다. `nonground_filter_state`에는 지도 질의 재사용 횟수를 기록한다.

`/tunnel/heading/state`에는 모드·보정 출처·정합 시각·bias·좌표 버전을 2Hz로 기록한다. RViz에는 간단한 DR/X-ICP 상태 텍스트를 추가하고 UFLD 이미지 창은 기본 비활성화했다. 기존 차체·경로·점유·정지 사유 표시는 유지한다.

## 검증 범위와 복귀

빌드, Python 단위 검사, C++ 지도 커널·전진/탈출 엔진·X-ICP 백엔드 검사, 독립 ROS master의 합성 연결 검사를 수행했다. 합성 검사에는 실제 검출→머저→장면 연결, 정적 부모의 static→unknown 전환, 저상 점 보존, 준비 구간 미무장과 빈 스캔을 포함한다. 추가 백 재생이나 MORAI 제어 런치 실행은 하지 않았다.

합성 입력의 처리 시간은 실제 토탈 런치 성능을 보장하지 않는다. 기존 지도에서 빈 터널을 먼저 확인하고, 이어 X-ICP 활성화의 지연 증가량과 헤딩 안정성, 저상·동적 장애물 반응을 실제 주행에서 확인한다. 정상화 후 사용자 확인을 받은 새 GPS 정상 빈 터널 두 회로 지도와 물리 경계를 갱신한다.

정책별 복귀는 `config/tunnel_recovery.yaml`에서 다음을 false로 바꾸고 런치를 재시작한다.

- `tunnel_map_filter.voxel_search_enabled`: 정확한 KD-tree 검색만 사용.
- `tunnel_recovery.native_geometry_enabled`: 기존 Python 광선·경로 계산.
- `obstacle_info_merger.native_worker_enabled`: 기존 동기 처리.
- `tunnel_recovery.preserve_static_certificate`: 이전 순간 운동 상태 기반 변환.
- `tunnel_wall_detector.object_risk_preslow_enabled`: 이전 넓은 감시 범위 선감속 정책.

소스·설정·활성 지도 백업, MPC/지도 전후 해시, 검사 로그는 `analysis/m8_map_latency_20261011/`에 있다. 복귀 플래그는 원인 비교용이며 수정 전 판단 결함을 다시 나타낼 수 있다. 실주행 지연 목표를 달성하기 위해 만료 기준·안전 여유를 풀지 않는다.
