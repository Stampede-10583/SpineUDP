# Orin-side Spine link counters, published by the companion about once per second on
# spine/stats. All counters are cumulative since the companion started.

builtin_interfaces/Time stamp  # Orin ROS time of this snapshot. Diagnostics only, never protocol time

# Stream 2 (poses to the RIO)
uint64 poses_sent
uint64 withheld_no_fix
uint64 withheld_bad_timestamp
uint64 withheld_stale
uint64 withheld_out_of_order
uint64 send_errors

# Stream 1 and Stream 3 (from the RIO)
uint64 odom_recvd
uint64 odom_seq_gaps     # missing Stream 1 seq values
uint64 trig_recvd        # Stream 3 packets received
uint64 triggers_missed   # triggerSeq values never seen (5+ Stream 3 packets lost in a row)
uint64 decode_errors     # wrong-length datagrams, or an inconsistent Stream 3 table

# RIO restarts
uint64 rio_restarts
float64 last_restart_controller_timestamp  # RIO time of the first packet after the latest restart; NaN if none
