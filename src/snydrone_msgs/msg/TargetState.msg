# Estimated target state from the vision tracker.

std_msgs/Header header

# ID of the tracked target (0 if single subject)
uint8 target_id

# Target position and velocity in world (or local) frame [m, m/s].
geometry_msgs/Point position
geometry_msgs/Vector3 velocity

# Distance to drone [m] if known (can be 0 or NaN if unknown).
float32 distance

# Framing error in normalized image coordinates:
# (0,0) means perfectly at desired framing point, +/-1 is full image width/height.
float32 frame_error_x
float32 frame_error_y
