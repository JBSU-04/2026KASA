# 회전교차로 준비 서행 / 2026-10-09

기존 미션 상태와 BehaviorConstraint 인터페이스를 유지한다. 준비는 APPROACH/YIELD 내부 단계이며 COMMIT은 최종 승인에만 사용한다. 기존 차량 추적기/MPC 설정은 수정하지 않는다.

## 활성 설정

`kcity_roundabout_utm.yaml`의 `roundabout/gap_select`:

- `preparation_enabled: true`: 준비 접근과 연속 가속 궤적 탐색 활성화.
- `preparation_wait_distance_m: 3.0`: 승인 불가 시 양보선 전 기본 대기 거리. 의무 정지와 다르다.
- `preparation_max_speed_mps: 1.5`: 후보 0/0.5/1.0/1.5m/s의 상한.
- `preparation_margin_m: 0.5`: 첫 차체 점유 영역 접촉까지의 경로거리에서 뺀 준비 경계 여유. 응답/제동거리도 적용한다.
- `preparation_horizon_s: 4.0`: 가속 시작 지연을 0.1초 간격으로 탐색.
- `spatial_front_gap_m: 2.0`, `spatial_rear_gap_m: 1.5`, `ego_length: 4.6`: 범퍼 사이 물리적 공간 후보 8.1m.
- `prediction_follow_standstill_gap: 2.0`, `prediction_follow_time_gap: 0.35`: 회전교차로 앞차 ACC. 주행 시 시간 간격과 앞차 제동 보호도 적용된다.

기본 생성자에서는 새 모드가 꺼져 있어 다른 사이트/기존 모델에 준비 정책을 자동 적용하지 않는다. `preparation_enabled: false`로 준비 적용 전 흐름으로 돌아갈 수 있다. 기존 간격까지 정확히 복귀하려면 변경 전 사이트 YAML을 복구한다.

## 판단 계약

추적 차량 쌍의 물리적 갭과 연속 관측 빈 길이는 별도 값이다. 물리적 길이를 미관측 셀로 잘라내지 않는다. 대신 계획한 차체 궤적의 새 점유 영역에 미관측/미추적 점유가 있으면 승인하지 않는다. 현재 자차 점유 영역은 자기 차체의 라이다 사각으로 인해 무효화하지 않는다. 확인된 이동 차량의 현재 점유는 예측으로 처리하지만 별도 차체 충돌 검사는 계속 적용한다. 미확정 앞차는 승인/준비 대상이 될 수 없으며 미확정 뒤차는 기존 보수적 속도 상한으로 평가한다.

열린 갭은 확인된 실제 빈 구간으로 제한하며 신선한 서로 다른 3스캔과 합류 동안의 관측 범위가 필요하다. 센서 부재/가림을 빈 교차로로 바꾸지 않는다.

각 후보는 현재 속도, 출발 지연, 준비 속도/정지 경계, 가속, 곡률 속도 상한, 앞차 ACC, 뒤차의 진입 후 양보를 포함한다. 전체 합류 완료 후 0.5초까지 검사하며 통과 후보 중 완료가 가장 빠른 것을 선택한다. 준비와 가속 예측 모두 실측 출발 곡선을 MPC 가속 한계로 제한한다. 합류 속도 상한은 앞차의 물리적 속도로 맞추며 승인 당시 값을 COMMIT 동안 보존한다.

준비는 동일 차량 쌍/조건 0.1초와 서로 다른 라이다·트래커 프레임 확인 후 발행한다. 준비 조건 실패 시 진입 영역 앞 대기 목표로 돌아간다. 최종 승인은 별도로 양보선 8m 이내, 동일 쌍 0.1초·서로 다른 프레임 확인이 필요하다. 승인 이후에는 뒤차/갭 재평가로 정지·취소하지 않으며 앞차 ACC와 센서·측위 고장 처리만 유지한다.

## 진단

기존 `/mission/roundabout/distance_debug` JSON에 `physical_gap_m`, `entry_phase`, `trajectory_observation`, `preparation_speed_mps`, `preparation_stop_distance_m`, `acceleration_start_delay_s`를 추가했다. `observed_free_m`은 물리적 갭과 별개인 실제 관측 길이이다. 관측/승인 시각은 소수점 정밀도를 보존한다. RViz 기존 gap_markers 토픽에서 준비 단계와 주황색 준비 경계를 표시한다.

백 재생은 기록된 자차/NPC 궤적의 판단 비교다. 새 대기 위치에서 달라지는 센서 가시성과 NPC 반응은 재현하지 않으므로 성공 주행 증거로 사용하지 않는다. MORAI 폐루프에서 실제 준비 접근/합류 시점/ACC 반응을 확인해야 한다.

이번 변경은 기존 ROS1 구현에 정책을 추가한 것이며 새 외부 코드를 복사하지 않았다. 앞선 Autoware/SUMO 참조 출처는 roundabout_observed_entry.md에 유지한다.
