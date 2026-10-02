std_msgs/Header header

# unknown | standing_up | standing | sitting_down | sitting | fallen
string mode
# tripod | ripple | wave
string gait
bool walking
bool crouched
float32 phase
# Order: lf, lm, lr, rf, rm, rr
bool[6] stance
float32 body_height
float32 step_height
# Command that is being executed (after limits and slew).
geometry_msgs/Twist command
float32 roll
float32 pitch
