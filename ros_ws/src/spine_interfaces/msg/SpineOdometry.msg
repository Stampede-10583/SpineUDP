# One Stream 1 packet (RIO -> Orin), as received by the companion. Published on spine/odometry.
#
# No header, on purpose: there is no Orin receive time to tempt anyone into pairing by time.
# controller_timestamp is the RIO's float64, bit-for-bit (docs/spine-link.md section 3).

int64 seq                     # Stream 1 packet sequence number; 0 after a robot-code restart
float64 controller_timestamp  # RIO time (s) the sensors were read. Opaque; compare only with RIO time

float64 x      # m, odometry frame (pure dead reckoning, arbitrary origin; use deltas)
float64 y      # m, odometry frame
float64 theta  # rad, (-pi, pi], CCW positive
float64 vx     # m/s, odometry frame
float64 vy     # m/s, odometry frame
float64 omega  # rad/s, CCW positive
