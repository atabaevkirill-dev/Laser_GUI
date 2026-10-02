# One ranging result of the 3 km eye-safe laser rangefinder.
std_msgs/Header header

uint8 TARGET_FIRST = 1
uint8 TARGET_LAST = 2
uint8 TARGET_MULTI = 3

bool valid
uint8 target_mode
# Up to 3 targets (multi-target mode), closest first, metres.
float32[] ranges
# Raw status byte of every range (see the module manual, 6.2.2).
uint8[] statuses
bool out_of_range
bool front_target
bool rear_target
