# A fused field pose for the companion to send back to the RIO in Stream 2. Subscribed on
# fused_pose. Produced by TavSLAM in production and by SyntheticPoseNode in tests.
#
# controller_timestamp MUST be the exact float64 received on spine/triggers for the trigger that
# exposed the frame, carried in float64 fields end to end. Never put it in a header.stamp or any
# builtin_interfaces/Time: those are sec+nanosec and do not round-trip a double.

float64 controller_timestamp  # RIO time (s) of the trigger that exposed the frame; copied, never computed
int64 trigger_seq             # informational only (logs); the companion never looks it up

float64 x      # m, map frame (the companion applies field_from_map to get the WPILib field frame)
float64 y      # m, map frame
float64 theta  # rad, CCW positive

bool valid     # false = no fix; the companion sends nothing for it
