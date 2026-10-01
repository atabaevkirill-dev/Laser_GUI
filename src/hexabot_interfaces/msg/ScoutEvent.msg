# Video analytics event.
std_msgs/Header header

# new_track | zone_intrusion | line_crossing | loitering | drone | target_ranged
string type
string camera
int32 track_id
string label
float32 confidence
# Position in header.frame_id when has_position is true.
bool has_position
geometry_msgs/Point position
float32 range_m
string snapshot_path
string description
