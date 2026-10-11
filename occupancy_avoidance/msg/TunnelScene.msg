# Native M8 scene. World coordinates use geometry_msgs/Point (float64).
std_msgs/Header header
uint64 scene_id
uint64 coordinate_version
string map_version
geometry_msgs/Pose ego_pose
float64 speed
float64 curvature
float64 offset_x
float64 offset_y
# Small compatibility metadata; no point arrays or ray arrays in JSON.
occupancy_avoidance/TunnelObject[] objects
string metadata_json
geometry_msgs/Point[] cells
int64[] owner_ids
bool[] fixed
float64[] seen
bool[] observed
# Observed floor cells [grid_x, grid_y, original_scan_stamp].
float64[] visibility
# Detector-to-merger beam evidence only; omitted on the final scene.
geometry_msgs/Point ray_origin
geometry_msgs/Point[] ray_endpoints
string mission_mode
