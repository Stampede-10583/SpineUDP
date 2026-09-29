# One camera trigger from the Stream 3 trigger table. Published on spine/triggers, oldest first,
# exactly once per trigger per RIO run. A trigger_seq that goes backwards means the RIO restarted.
#
# No header, on purpose (see SpineOdometry.msg).

int64 trigger_seq             # triggerSeq from Stream 3; starts at 0, +1 per pulse
int64 packet_seq              # packetSeq of the Stream 3 packet it first arrived in (logs only)
float64 controller_timestamp  # RIO time (s) of the trigger pulse, bit-for-bit from Stream 3
