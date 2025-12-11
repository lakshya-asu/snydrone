# High-level cinematic shot description produced by the LLM agent.

std_msgs/Header header

# e.g. "ORBIT", "FOLLOW", "REVEAL"
string shot_type

# ID of the target actor (0 if single subject)
uint8 target_id

# Generic geometric/temporal params (use per-shot-type as needed)
float32 radius          # [m] orbit radius or distance
float32 height_offset   # [m] camera height offset relative to target
float32 duration        # [s] desired shot duration
float32 speed           # [m/s] nominal motion speed

# Desired framing in normalized image coordinates (0..1)
# (0.5, 0.5) is image center, (0,0) top-left, (1,1) bottom-right.
float32 framing_x
float32 framing_y

# Approximate subject size in frame (fraction of image height)
float32 framing_scale

# Optional JSON blob for extra parameters per-shot-type
string params_json
