std_msgs/Header header

uint8 SOURCE_NONE=0
uint8 SOURCE_ROUNDABOUT=1
uint8 SOURCE_COMPOSITE=255

uint8 PRIORITY_NONE=0
uint8 PRIORITY_SPEED=50
uint8 PRIORITY_YIELD=100
uint8 PRIORITY_EMERGENCY=200

uint8 source
uint8 priority
bool active

# 음수면 제한 없음, 그 외 단위는 m/s
float64 max_speed

# 차량 기준 경로방향 정지선 잔여거리[m]
bool stop_required
float64 stop_distance

# 진입 후 같은 진행방향의 선행차 정보
bool lead_vehicle_active
int32 lead_vehicle_id
float64 lead_distance
float64 lead_speed

# Optional profile follows the selected lead through constraint arbitration.
bool lead_profile_override
float64 lead_standstill_distance
float64 lead_time_gap

float64 comfortable_decel
float64 emergency_decel
string reason
