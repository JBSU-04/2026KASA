# 센서 추적기가 출력하는 제어용 최소 객체 정보
int32 id
geometry_msgs/Point position     # 배열 header 좌표계 기준[m]
geometry_msgs/Vector3 velocity   # 배열 header 좌표계 기준[m/s]
geometry_msgs/Vector3 size       # 지도축 정렬 bounding box 크기[m]
float32 confidence               # 0~1
uint32 hit_count
bool velocity_valid
bool has_moved                   # 관측 중 실제 이동이 연속 확인됐는지 여부
