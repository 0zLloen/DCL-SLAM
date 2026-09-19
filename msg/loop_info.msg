# Loop Info
Header header

int32 robot0
int32 robot1
int32 index0
int32 index1
float32 noise
int32 init_yaw
geometry_msgs/Transform pose0
geometry_msgs/Transform pose1
sensor_msgs/PointCloud2 scan_cloud
geometry_msgs/Transform pose_between
# T2 attack attribution (all default to empty/false for honest loops).
# An attack node sets attack=true and a stable attack_event_id on the loop it
# submits; DCL echoes the id back with attack_report=true at each stage
# (graph_inserted, pcm_retained, pcm_removed) so fabrication can be proven.
bool attack
string attack_event_id
string attack_mode
float32 attack_dx
float32 attack_dy
float32 attack_alpha
float32 attack_separation
bool attack_report
string attack_stage
int32 attack_reporter
string attack_reason
