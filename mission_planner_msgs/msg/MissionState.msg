std_msgs/Header header

uint8 IDLE=0
uint8 APPROACH=1
uint8 YIELD=2
uint8 COMMIT=3
uint8 CIRCULATE=4
uint8 EXIT=5
uint8 FAILSAFE=6

uint8 state
bool active
bool gap_safe
bool localization_valid
bool tracker_stream_valid
int32 blocking_object_id
int32 tracked_object_count
float64 distance_to_yield
float64 next_npc_ttc
float64 front_gap
string reason
