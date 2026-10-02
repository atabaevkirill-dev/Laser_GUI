# Located target (person, vehicle, drone, ...).
std_msgs/Header header

int32 track_id
string label
float32 confidence
float32 range_m
# lrf | depth | ground_plane | none
string range_source
# Bearing from base_link (radians), elevation positive up.
float32 azimuth
float32 elevation
bool has_position
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
bool has_geo
float64 latitude
float64 longitude
float64 altitude
