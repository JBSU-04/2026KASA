uint8 UNKNOWN=0
uint8 STATIC=1
uint8 DYNAMIC=2
int64 id
int64 observed_track_id
bool confirmed
uint8 motion_state
bool static_certified
bool dynamic_confirmed
float64 top_elevation
geometry_msgs/Point position_world
geometry_msgs/Vector3 velocity_world
float64 last_seen
bool in_trigger_corridor
# Remaining legacy detector attributes; no points/rays/cells in this adapter.
string metadata_json
