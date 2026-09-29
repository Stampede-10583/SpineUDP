# A rare, notable link event, published by the companion on spine/events and logged by
# spine_stats.

uint8 RIO_RESTART = 1   # robot code restarted or the RIO rebooted; trigger history and order gate reset
uint8 TRIGGER_GAP = 2   # triggerSeq values skipped (5+ Stream 3 packets lost); count in detail
uint8 DECODE_ERROR = 3  # a datagram was rejected; reason in detail
uint8 ODOM_GAP = 4      # Stream 1 seq values skipped; count in detail

builtin_interfaces/Time stamp  # Orin ROS time of the event. Diagnostics only, never protocol time
uint8 kind
float64 controller_timestamp   # RIO time involved, if any; NaN otherwise
string detail                  # short human-readable detail
