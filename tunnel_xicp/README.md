# Tunnel X-ICP observer (comparison only)

음영구간 벽 매칭의 신뢰도와 비교 자세를 관찰하는 1단계 구현입니다.
**기존 odom/TF, 벽 제거, 장애물 분류, 회피 경로와 차량 제어에는 연결하지 않습니다.**
양쪽 벽, 대응점 수/비율, 잔차, 공식 X-ICP의 횡방향/yaw 관측 가능성을
검증합니다. 한쪽 벽이면 DEGRADED, 오래된 입력·부족한 대응점·과도한 보정은 INVALID입니다.
비교 자세에는 관측 가능한 횡위치/yaw만 반영하며 진행방향·높이·roll/pitch는 유지합니다.
일반 ICP로 대체하는 fallback은 없습니다. 출처/패치는 [UPSTREAM.md](UPSTREAM.md)에 있습니다.

## Build / run

기존 실행 중인 차량 노드를 재시작하지 않은 별도 빌드입니다.

```bash
cd /home/kuuve/catkin_ws
# 이미 설치한 로컬 Ceres를 재생성할 때만 필요:
bash src/tunnel_xicp/scripts/bootstrap_ceres.sh
cmake -S src -B build_xicp -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_WHITELIST_PACKAGES=tunnel_xicp -DCATKIN_DEVEL_PREFIX="$PWD/devel_xicp" \
  -DCMAKE_PREFIX_PATH="$PWD/devel;/opt/ros/noetic" \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_ENABLE_TESTING=ON
cmake --build build_xicp --target tunnel_xicp_node test_xicp_backend -j2
source devel_xicp/setup.bash
roslaunch tunnel_xicp observer.launch
```

대회 launch도 `use_tunnel_xicp:=true` 기본값으로 observer를 포함합니다.
새 터미널에서 위 overlay를 source한 뒤 실행해야 합니다. 별도 observer와
대회 launch를 동시에 실행하지 마세요. 끄려면 `use_tunnel_xicp:=false`입니다.

대회/debug RViz에는 MarkerArray `/tunnel/xicp/markers`를 추가했습니다.
다른 설정에서는 Fixed Frame을 `map`으로 두고 같은 MarkerArray를 추가하세요.
참조벽 회색, 현재 관측 청록, 매칭 결과 초록/주황/빨강(VALID/DEGRADED/INVALID),
대응선 노랑, 기존 벽 제한선 자홍입니다. 전체 ICP 결과는 진단 점군에만 표시합니다.
`/tunnel/xicp/comparison_pose`는 제한된 비교 자세이며 제어용이 아닙니다.
`/tunnel/xicp/status`에는 관측 가능성, 양쪽 벽 여부, 잔차, 대응비율,
보정량, 반복/제약 수, 처리시간과 덮어쓴 스캔 수가 있습니다.
status/pose는 light bag, markers는 heavy bag 기록 목록에 추가했습니다.

## Map provenance / limits

`maps/tunnel_walls.pcd`는 2026-10-02 00:45:38 백의 **실측 3D 점**입니다.
M8의 16–58초에서 24–26초와 42–53초를 제외하고 기록된 물체를 마스킹합니다.
기존 배경 높이맵의 비팽창 셀은 위치 prior로만 사용하며 벽을 수직으로 생성하지 않습니다.
센서 z 0–1.5m, 0.2m voxel, 수직면 법선 검증으로 252스캔 / 7,142점을 저장했습니다.
같은 주행에서 만든 맵이므로 그 주행에 대한 독립적인 성능 검증이 아닙니다.
백 재생 테스트는 수행하지 않았습니다. 맵 정확도와 30ms p95 예산은
독립 주행 데이터로 추가 검증해야 하며 현재 제어 적용을 의미하지 않습니다.

```bash
python3 src/tunnel_xicp/scripts/build_wall_map.py \
  --pose-bag bags/competition_2026-10-02-00-45-38.bag \
  --cloud-bag bags/competition_heavy_2026-10-02-00-45-38.bag \
  --background-map src/new_lane/config/tunnel_bgmap.npz \
  --out src/tunnel_xicp/maps/tunnel_walls.pcd
```

JSON에 원점·외부파라미터·구간·점 수·입력/출력 해시를 보관합니다.
동명 출력은 `--overwrite` 없이는 덮어쓰지 않습니다.
맵 없음, 좌표계/외부파라미터 불일치, 오래된 자세/점군은 INVALID로 처리합니다.
latest-only 입력으로 밀린 스캔을 누적하지 않고 30ms 초과는 경고합니다.
X-ICP 자체는 여전히 바닥/천장 점 제거와 차량 제어에 연결하지 않습니다.
별도 터널 안전 로직은 실제 회피 경로 제동 판단과 지역 노면 모델을 사용합니다.
`XicpStatus`에 preprocessing_ms / matching_ms / verification_ms를 추가했습니다.
전체 처리시간의 30ms p95 충족 여부는 새 독립 주행에서 확인해야 합니다.

## Tests (no bag replay)

```bash
devel_xicp/lib/tunnel_xicp/test_xicp_backend
python3 src/tunnel_xicp/test/test_map_builder.py
```

합성 양벽/단벽, 회피 yaw, 관측 불가능 축 차단, 저품질/큰 보정 거부,
반복 프레임, 외부파라미터/시간, 바닥 법선 제거와 비교 전용 출력을 검증합니다.
