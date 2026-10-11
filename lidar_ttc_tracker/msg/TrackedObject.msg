# 추적 장애물 1개 (map=ENU 좌표계, Ego_topic position 기준 평면)

uint32  id           # 트랙 ID (헝가리안 매칭으로 프레임 간 유지)
float64 x            # 중심 x [m] (map)
float64 y            # 중심 y [m] (map)
float64 z            # 중심 z [m] (박스 중앙 높이)
float64 yaw          # BEV heading [rad] (map 기준)
float64 size_x       # length: 진행(yaw)방향 크기 [m]
float64 size_y       # width : 횡방향 크기 [m]
float64 size_z       # height: 높이 [m]

float64 vx           # 절대속도 x [m/s] (map, 칼만 추정)
float64 vy           # 절대속도 y [m/s]
float64 speed        # 절대속력 |v| [m/s]
float64 rvx          # 자차 기준 상대속도 x [m/s] (= vx - ego_vx)
float64 rvy          # 자차 기준 상대속도 y [m/s]

float64 distance     # 자차 중심 -> 객체 중심 거리 [m]
float64 closing_speed # 접근속도 [m/s] (양수=가까워지는 중)
float64 ttc          # Time-To-Collision [s]. 접근 중이 아니면 -1
bool    is_dynamic   # true=동적 장애물, false=정적 (속도 히스테리시스 분류)

# Geometry-based vehicle-likeness, temporally filtered in the tracker.
# is_vehicle is deliberately independent from is_dynamic so stopped cars can
# still be published as vehicles.
float64 vehicle_confidence  # 0=structure-like, 1=vehicle-like
bool    is_vehicle           # confirmed vehicle candidate

# Evidence that a real physical object is here at all, independent from what
# shape it has.  A low obstacle and a single-ring far return are both poor
# vehicle candidates, but dropping them is a safety decision, not a shape one -
# so the ACC/AEB retention path keys on this instead of vehicle_confidence.
float64 obstacle_confidence  # 0=ground residue/noise, 1=solid object

# Reliability of the PCA long-axis yaw. 0=near-isotropic/sparse cluster or
# yaw warm-up not complete; 1=well-supported elongated, motion-mature track.
# This is separate from vehicle likeness.
float64 yaw_confidence

uint32  hits         # 누적 매칭 횟수 (신뢰도 참고)
uint32  age          # 트랙 생성 후 경과 프레임

# Velocity confidence is independent of box yaw confidence. Invalid is unknown,
# never proof of a static object. Velocities and scalar std are in map m/s.
bool velocity_valid
float64 velocity_std
float64 observation_age
