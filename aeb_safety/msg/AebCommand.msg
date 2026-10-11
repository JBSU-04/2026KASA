# 안전/행동 결정층이 arbiter 로 보내는 통합 명령.
# 여러 producer(aeb_decision, intersection_manager)가 각자 발행하고 arbiter 가 보수적으로 융합한다.

Header header
string  source          # "aeb" | "cutin" | "intersection"

# 단계
uint8 STAGE_NONE=0      # 개입 없음(정상 주행)
uint8 STAGE_FCW=1       # 전방충돌경고 = 완만 감속(target_speed 캡)
uint8 STAGE_PB1=2       # 부분제동 1
uint8 STAGE_PB2=3       # 부분제동 2
uint8 STAGE_FULL=4      # 완전제동(긴급) — emergency=true
uint8 STAGE_HOLD=5      # 정지선 앞 정지 대기(교차로 gap-accept 실패 등)
uint8   stage

bool    emergency       # true 면 arbiter 가 즉시 최우선 완전제동
float64 target_speed    # 완만감속 목표속도 [km/h]. <0 = 캡 없음
float64 brake           # 제동 페달 [0..1] (PB1/PB2/FULL 단계에서 유효)
float64 decel           # 요구/명령 감속도 [m/s^2] (참고·로깅)

uint32  target_id       # 위협/충돌 대상 트랙 ID (0=없음)
float64 ttc             # 대상 TTC 또는 time-to-conflict [s] (-1=없음)
float64 distance        # 대상 종방향 거리 [m] (-1=없음)
string  reason          # 디버그 문자열 (예: "CCRb id=7 ttc=1.2" / "cutin id=3" / "cross id=9")
