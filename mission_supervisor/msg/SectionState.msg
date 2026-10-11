std_msgs/Header header

string  section_id          # "M1".."M8". 구간 밖이면 빈 문자열
string  section_name
int32   csv                 # 현재 path_number
int32   index               # 현재 CSV 인덱스
float64 progress            # 구간 내 진행률 0~1
int32   speed_state         # CSV mission_state (참고용)
bool    speed_limit_exempt  # 60kph 제한 면제 구간인가
bool    index_valid         # false = 위치 상실, 속도 적산으로 추정 중
float64 dwell_sec           # 현재 구간 체류 시간
string[] executors          # 이 구간에서 활성화할 미션 실행기 이름
