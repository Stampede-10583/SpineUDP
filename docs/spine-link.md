# Spine link reference (RIO ↔ Orin UDP protocol)

This is the reference for anyone writing the **coprocessor end** of the Spine link: the Orin's companion node, the adapter node that feeds TavSLAM, or a synthetic stand-in such as a ROS 2 test node. It describes the protocol exactly as the robot-side code implements it, and the behaviour the robot side has that the code comments don't spell out.

Spine was formerly called **KatanaUDP**; old commits and notes use that name. TavSLAM is the team's name for the SLAM stack on the Orin.

Source of truth on the robot side (this repo):
- `src/main/java/frc/lib/SpineUDP/SpineUDP.java`: the protocol, sockets and threads. `VisionLinkProtocol` holds every offset.
- `src/main/java/frc/robot/Constants.java`, `VisionLinkConstants`: addresses, ports and rates for the bench build.
- `src/main/java/frc/robot/bench/SyntheticOdometrySource.java`: what the bench RIO puts into Stream 1 (a smooth synthetic path).
- `src/main/java/frc/robot/bench/BenchTriggerPulser.java`: what drives Stream 3 on the bench (60 Hz, optional DIO pulse).
- `src/test/java/frc/lib/SpineUDP/SpineUDPTest.java`: working encode/decode examples over loopback.
- `ros_ws/src/spine_link/test/data/spine-link-vectors.txt`: golden packets (field values and expected little-endian hex) that both ends' tests check.

This repo ships only **synthetic producers**. The production Stream 1 sampler (real swerve encoders and gyro) and the production DIO camera pulser live in the robot project. They feed the same library classes, so everything below applies to them unchanged.

Source of truth on the coprocessor side: `ros_ws/src/spine_link/` (the companion and synthetic nodes, with `include/spine_link/protocol.hpp` mirroring `VisionLinkProtocol`) and `ros_ws/src/spine_interfaces/` (the ROS messages in §3).

---

## 1. What the link is for

The robot controller (a roboRIO 2 today; the library is also meant to run on SystemCore) drives the swerve drivetrain and owns the robot's pose estimate. An Orin-class coprocessor runs TavSLAM, which fuses camera AprilTag detections with the robot's odometry and produces a globally corrected field pose. Spine is the transport between them: three independent, one-directional UDP streams, each with its own socket and port.

| # | Name | Direction | Rate | Size | Default port (bound by) |
|---|---|---|---|---|---|
| 1 | Odometry | RIO → Orin | 200 Hz, fixed | 64 B | 5801 (Orin binds) |
| 2 | Vision pose | Orin → RIO | event-driven, one per fused result | 40 B | 5802 (RIO binds, all interfaces) |
| 3 | Trigger table | RIO → Orin | one per camera trigger pulse, ~60 Hz | 88 B | 5803 (Orin binds) |

- **Addresses:** the RIO sends to `kOrinIp`, which defaults to `10.105.83.11` (a placeholder; see §8), or `127.0.0.1` in simulation. The `Spine/orinHost` Preference overrides it at robot-program startup, so a real RIO can be pointed at a PC without redeploying. The roboRIO itself is `10.105.83.2` (team 10583).
- **Source ports:** RIO senders use ephemeral source ports, so don't filter on them. The RIO accepts Stream 2 datagrams from any source address.

**The overall loop:**
1. The RIO fires a hardware trigger on a DIO pin. That pulse exposes the cameras.
2. At the same moment, the RIO records the controller timestamp of the pulse and sends it in Stream 3.
3. On the Orin, the adapter node attaches that timestamp to the frame the pulse exposed, by an exact frame-count match (§3.1).
4. TavSLAM fuses those frames with the Stream 1 odometry, carrying each frame's timestamp through.
5. The companion node sends the fused field pose back in Stream 2, stamped with that trigger's timestamp, copied through unchanged.
6. The RIO feeds that pose into its pose estimator *at that timestamp* and replays its own odometry forward from there.

---

## 2. Wire format

### Conventions (all streams)
- **Byte order:** little-endian on the wire, always, on both ends. Never "native order."
- **Field types:** every field is exactly 8 bytes, either `int64` (two's-complement) or `float64` (IEEE-754 double). Every offset is a multiple of 8 and there is no padding, header, magic number, version field or checksum.
- **Packet identification:** a packet is identified only by the port it arrives on and its exact length.
- **Parsing:** read and write each field at its absolute byte offset. In C/C++, do **not** cast a struct over the buffer; use `memcpy` per field, or a packed struct with `static_assert`s on `offsetof` and `sizeof`. In Python, use `struct` with the `<` prefix.

### Types: sequence numbers are int64; timestamps are float64 seconds
- **Sequence numbers are `int64`:** `seq`, `packetSeq` and `triggerSeq`. Don't decode them as doubles.
- **Timestamps are `float64` seconds, not integer microseconds or nanoseconds.** This applies to every timestamp field in every stream. Decoding a timestamp field as an `int64` produces garbage.
  - A `float64` holds a controller uptime to well under a microsecond for any realistic uptime, so precision is not a concern *as long as the value is never converted*.

### Stream 1: odometry, RIO → Orin, 64 bytes

| Offset | Field | Type | Unit / meaning |
|---|---|---|---|
| 0 | `seq` | int64 | Packet sequence number. 0 on the first packet after robot code starts, +1 per packet sent |
| 8 | `timestamp` | float64 | Controller time (s) at which the sensors were read (see §3) |
| 16 | `x` | float64 | m, odometry frame |
| 24 | `y` | float64 | m, odometry frame |
| 32 | `theta` | float64 | rad, wrapped to (−π, π], CCW positive |
| 40 | `vx` | float64 | m/s, odometry frame (not robot frame) |
| 48 | `vy` | float64 | m/s, odometry frame |
| 56 | `omega` | float64 | rad/s from the gyro, CCW positive |

**What the pose is:** pure wheel+gyro dead reckoning. It is **not** the RIO's vision-corrected pose; sending that would feed TavSLAM its own output. (The bench RIO sends a synthetic path instead; its fields are mutually consistent, but it is not a real robot.)
- **Frame:** it starts at `x = 0, y = 0` on the first tick, with `theta` equal to the gyro heading at that moment (not 0). It is never reset while the code runs.
- **Treat it as relative.** Use deltas between samples; its absolute origin is meaningless.
- **Rate:** 200 Hz with a fixed schedule. The sensors underneath refresh every 5 ms, so consecutive samples normally carry new data.
- **Sequence gaps:** a gap in `seq` means the packet was lost on the network, or the RIO's `send()` failed. Sequence numbers are not used up by skipped ticks.

### Stream 2: vision pose, Orin → RIO, 40 bytes

| Offset | Field | Type | Unit / meaning |
|---|---|---|---|
| 0 | `seq` | int64 | Orin-assigned. The RIO reads it but does not validate or reorder on it |
| 8 | `timestamp` | float64 | **Byte-for-byte copy** of the Stream 3 `timestamp` of the trigger that produced this pose |
| 16 | `x` | float64 | m, **field frame** (see below) |
| 24 | `y` | float64 | m, field frame |
| 32 | `theta` | float64 | rad, field frame, CCW positive |

**Field frame:** the pose replaces the robot's pose estimate outright (see §4), and PathPlanner and the drive code consume that estimate. So it must be in WPILib's field convention:
- Origin at the blue-alliance corner, +x pointing toward the red alliance wall, +y to the left looking downfield, θ counter-clockwise from +x.
- The frame does not flip for red alliance.

**Length:** the datagram must be **exactly 40 bytes**. Any other length is dropped (see §5).

### Stream 3: trigger table, RIO → Orin, 88 bytes

| Offset | Field | Type | Meaning |
|---|---|---|---|
| 0 | `packetSeq` | int64 | Packet counter, independent of trigger numbering. Starts at 0 |
| 8 + 16·i | `triggerSeq[i]` | int64 | i = 0..4, **most recent first** |
| 16 + 16·i | `timestamp[i]` | float64 | Controller time (s) of that trigger pulse |

- **Five triggers per packet.** Each packet carries the five most recent triggers, not just the newest, so a single dropped datagram loses nothing: every trigger appears in five consecutive packets.
- **Trigger numbering:** `triggerSeq` starts at 0 and increases by exactly 1 per pulse, with no gaps on the RIO side.
- **Empty slots:** until five triggers have fired, the unused slots are `triggerSeq = -1`, `timestamp = 0.0`.
- **Expected handling:** keep a `lastSeenTriggerSeq`, starting at -1. For each packet, take the entries with `triggerSeq > lastSeenTriggerSeq`, process them oldest-first (walk from i = 4 down to i = 0), then update `lastSeenTriggerSeq`. The `-1` sentinel is rejected by this rule automatically.
- **Coalescing:** the RIO may merge triggers under load, so one packet can carry two or more new triggers. That is expected; the table design covers it. Don't assume one new trigger per packet.

---

## 3. Time: one clock, owned by the robot controller

This is the part most likely to be got wrong.

- **One clock.** The robot controller's clock is the only clock in the system. Every timestamp on the wire was produced by the RIO calling `MathSharedStore.getTimestamp()`. On the roboRIO that is the FPGA clock, in seconds since the controller booted.
- **No clock sync.** There is deliberately no clock synchronization and no offset estimation anywhere. Adding them would reintroduce the very problem the design exists to avoid.
- **Not Unix, ROS or wall-clock time.** It is not comparable to the Orin's clock, and it is not monotonic across RIO reboots: it restarts near 0 when the RIO reboots. Restarting only the robot program does *not* reset it, but it *does* reset every sequence number to 0 and the Stream 1 odometry origin.
- **Treat it as opaque.** A controller timestamp is an opaque value you may compare with other controller timestamps and copy. Never convert it to or from your own clock.
- **Echo it exactly.** The Stream 2 `timestamp` must be the exact 8 bytes the RIO sent in Stream 3 for that frame's trigger.
  - Don't recompute it, add latency to it, round it, or store it in a `float32`.
  - Don't carry it through a ROS `builtin_interfaces/Time` (split into sec/nanosec and rebuilt); that does not return the same double bit-for-bit.
  - Keep the original `float64`, or the raw 8 bytes, alongside the frame and write it back out unchanged.
  - A wrong timestamp does not raise an error on the RIO. It silently shifts every correction in time and degrades the pose.
- **Pose timestamps are RIO-original.** A Stream 2 timestamp is attached exactly once, upstream at the adapter node, by an exact frame-count -> trigger-table match, before TavSLAM ever sees the frame (§3.1). Nothing downstream of that lookup may touch, re-derive or round-trip it through `builtin_interfaces/Time`. Nothing on the Orin pairs anything by time, on any clock.
- **Detecting a RIO restart.**
  - On Stream 1, the same idea with a wider window (Stream 1 runs at 200 Hz): the RIO is treated as restarted if `seq` is more than 50 behind the last one, or `seq` went back while `timestamp` went forward, or `seq` went forward while `timestamp` went back. Reset your odometry tracking. Any other `seq <= last` is a duplicate or reordered packet; drop it. The companion reports a Stream 1 restart as a `LinkEvent` only; `rioRestarts` counts Stream 3 detections, since those are what reset protocol state.
  - On Stream 3, the RIO is treated as restarted if either `triggerSeq[0] < lastSeenTriggerSeq - 5` (the sequence went back further than one table, which LAN reordering can't produce), or `triggerSeq[0] <= lastSeenTriggerSeq` while `timestamp[0]` is greater than the newest timestamp seen (the sequence went back but RIO time went forward, which a reordered packet can't produce), or `triggerSeq[0] > lastSeenTriggerSeq` while `timestamp[0]` is less than the newest timestamp seen (the sequence went forward but RIO time went back: the RIO rebooted and the first packets of the new run were missed). Reset `lastSeenTriggerSeq` to -1 and discard every trigger kept from the old run. Any other packet with `triggerSeq[0] <= lastSeenTriggerSeq` is a duplicate or reordered packet and adds nothing.
  - All these tests use RIO data only; none reads an Orin clock.
- **Synthetic stand-ins:** use the timestamps from Stream 3; never generate your own. A test node that sends Stream 2 without first receiving Stream 3 has no valid timestamp to send. The RIO's unit tests fake one as "now minus 50 ms" only because they run inside the RIO process. The repo's `SyntheticPoseNode` takes each timestamp from `spine/triggers` by exact `triggerSeq`, standing in for the adapter and TavSLAM.

### 3.1 The timestamp pipeline on the Orin

```
RIO --Stream 3--> companion --spine/triggers--> adapter --frame + float64--> TavSLAM
                                                                               |
RIO <--Stream 2-- companion <------------ fused_pose (FusedPose) --------------+
```

1. **Companion (the Spine UDP node)** receives Stream 3, applies the §2 read rule, and publishes each new trigger once, oldest first, as `spine_interfaces/SpineTrigger` on `spine/triggers`:
   ```
   int64   trigger_seq           # triggerSeq from Stream 3
   int64   packet_seq            # packetSeq of the Stream 3 packet it first arrived in (logs only)
   float64 controller_timestamp  # the Stream 3 timestamp, bit-for-bit
   ```
   It also publishes Stream 1 as `spine_interfaces/SpineOdometry` on `spine/odometry` (`seq`, `controller_timestamp` and the six motion fields, same units as §2). Neither message has a `header`, so there is no Orin receive time for anyone to pair against.
2. **Adapter** (outside this repo) gives each camera frame its trigger's `controller_timestamp` by an exact frame-count match, and hands TavSLAM the frame plus that float64.
3. **TavSLAM** (outside this repo) computes with that timestamp and outputs the fused pose carrying the same float64, as `spine_interfaces/FusedPose` (§3.3).
4. **Companion** copies `FusedPose.controller_timestamp` into Stream 2 bytes 8..15 as its exact 8 bytes. It does no pairing, lookup, arrival-time estimation or tolerance window, and never reads an Orin clock for protocol time.

**Carrier rule:** at every hop the timestamp travels in a real `float64` field. Never put it in `header.stamp` or any `builtin_interfaces/Time`: that type is sec+nanosec, so a double written into it is rounded to the nearest nanosecond and does not come back bit-for-bit. CDR serializes a `float64` as its raw IEEE-754 bits, so a `float64` field is exact across ROS.

### 3.2 Interface to the upstream nodes (adapter and TavSLAM)

The adapter and TavSLAM are outside this repo, and how they work internally is their authors' business. This is everything the companion gives them and expects back.

**The companion provides:**
- `spine/triggers` (`SpineTrigger`): each new trigger exactly once per RIO run, in increasing `trigger_seq` order.
  - A gap in `trigger_seq` means five or more Stream 3 packets in a row were lost (counted as `triggersMissed`).
  - A `trigger_seq` that goes backwards means the RIO restarted.
- `spine/odometry` (`SpineOdometry`): every Stream 1 packet, with its controller timestamp.
- `spine/events` (`LinkEvent`): RIO restarts, trigger gaps and decode errors.

**The companion expects:**
- `fused_pose` (`FusedPose`, §3.3).
- `controller_timestamp` must be the exact float64 received on `spine/triggers` for the trigger that exposed the frame, carried in `float64` fields end to end (the carrier rule above).
- Poses the companion can't use are withheld and counted (§3.3), never corrected.

**Optional:** if the upstream side publishes `spine/adapter_stats` (a `spine_interfaces` message carrying at least `frames_unpaired`), `spine_stats` forwards it to NT as `Spine/Orin/framesUnpaired`.

### 3.3 FusedPose contract (what the companion consumes)

The companion subscribes to `fused_pose` as `spine_interfaces/FusedPose`. In production that is TavSLAM's output (remapped in `orin.launch.py`); in test it's `SyntheticPoseNode` publishing the same type on the same topic.

```
float64 controller_timestamp  # RIO time of the trigger that exposed the frame; the adapter's float64, unmodified
int64   trigger_seq           # informational only (logs); the companion never looks it up
float64 x                     # m, map frame (see field_from_map below)
float64 y                     # m
float64 theta                 # rad, CCW positive
bool    valid                 # false = no fix
```

For each message, the companion applies these steps in order. A message that fails a step is withheld (not sent) and counted under `Spine/Orin/*` (§6):

1. **No fix:** `valid` is false, or `x`, `y` or `theta` is not finite -> `withheldNoFix`.
2. **Bad timestamp:** `controller_timestamp` is NaN -> `withheldBadTimestamp`. There is no lookup or matching here; the only lookup happened in the adapter.
3. **Stale:** `newest_trigger_ts - controller_timestamp > max_age_s` (default 1.4 s, under the RIO's 1.5 s) -> `withheldStale`. This compares RIO time only with RIO time. If no trigger has arrived since the companion started or the RIO last restarted, the pose is also withheld as stale.
4. **Order gate:** `controller_timestamp <= last_sent_ts` -> `withheldOutOfOrder`. The companion never sends a pose older than, or equal to, one it has already sent (see §4 for why). On a detected RIO restart the gate is kept if the new run's trigger times are still later than the last sent pose (a robot-program restart: the RIO clock kept running), and reset only if RIO time went backwards (a reboot), since otherwise the new run could never send anything.
5. **Field frame:** apply the static 2-D transform `field_from_map_{x,y,yaw}` (default identity) to `x`, `y` and `theta`, so Stream 2 is in the WPILib field frame (§2).
6. **Encode** the 40-byte Stream 2 packet: the companion's own `seq`, then `controller_timestamp` as its exact 8 bytes (little-endian bit copy, no arithmetic), then `x`, `y`, `theta`.
7. **Send** to `rio_address:5802` -> `posesSent`, or `sendErrors` on failure.

If there is no pose, or no good pose, nothing is sent: silence is the no-fix signal (§4).

---

## 4. What the RIO does with a Stream 2 pose

1. **Receive (network thread).** A daemon thread blocks in `receive()`. For each datagram it:
   - drops it if the length is not exactly 40;
   - drops it if the timestamp is NaN, in the future, or more than 1.5 s old, measured against the RIO's clock at arrival;
   - drops it if `x`, `y` or `theta` is not finite;
   - otherwise copies it into a lock-free 64-slot ring. If the ring is full, the packet is dropped.
   Every drop is counted (§6).
2. **Apply (main robot loop, 50 Hz).** Once per 20 ms loop, the ring is drained in arrival order. Each pose is passed to WPILib's `SwerveDrivePoseEstimator.addVisionMeasurement(pose, timestamp, stdDevs)` with **std-devs of 0.0**.
3. **Why 0.0 std-devs work.** The estimator's closed-form Kalman gain per axis is `q / (q + sqrt(q·r))`, where `r = stddev²`. With `r = 0` and nonzero state std-devs `q`, the gain is exactly 1. The estimator also records its own odometry for the last 1.5 s. So a pose stamped at time *t*:
   - *replaces* the estimate at time *t* (TavSLAM has already fused odometry, so it isn't blended again);
   - then has the RIO's odometry since *t* replayed on top.
   The result is the Orin's pose with its latency compensated. With end-to-end latency L, the pose the robot drives on is the Orin pose from time *t* plus L of local odometry.

**Consequences for the sender:**
- **Send poses in timestamp order.** When WPILib accepts a measurement at time *t*, it **deletes every vision update newer than *t***. A late-arriving older pose therefore rolls back any newer correction already applied. The RIO does not reorder, dedupe or check `seq`. If you might produce an older result after a newer one, drop the older one on the Orin side. The companion's order gate (§3.3) does exactly this.
- **Keep end-to-end latency well under 1.5 s.** Older poses are dropped on arrival, and WPILib would ignore them anyway.
- **Every pose is trusted completely.** There is no outlier rejection on the RIO beyond the validity checks above. One bad pose teleports the robot's estimate, and PathPlanner will steer toward it. Gate quality on the Orin.
- **There is no "no fix" message.** If you have nothing good, send nothing. When Stream 2 goes quiet, the RIO continues on odometry alone. It has no link-health timeout and no fallback behaviour.

---

## 5. Delivery guarantees (there are none beyond UDP's)

- **No retransmission, acknowledgement, handshake or heartbeat** on any stream. Each side just sends. Ports can come up in any order, and neither side waits for the other.
- **Loss:**
  - Stream 1: lost samples are simply gone. Use `seq` gaps to notice.
  - Stream 3: loss is harmless unless five or more consecutive packets are lost.
  - Stream 2: a lost pose is a missed correction; nothing retries it.
- **Reordering and duplication** are possible in principle on a switched robot LAN, though rare.
  - Stream 1: order by `seq`.
  - Stream 3: the `triggerSeq > lastSeen` rule handles both.
  - Stream 2: see the ordering requirement in §4.
- **Nothing listening:** the RIO's send sockets are unconnected, so ICMP port-unreachable errors don't come back to it. If nothing is listening on 5801 or 5803, packets are silently discarded and the RIO's send counters look healthy. You can't infer from the RIO side that the Orin is receiving.
- **Rates are not negotiable** from the Orin side. Stream 1 is fixed at 200 Hz, and Stream 3 follows the physical trigger rate of 60 Hz.

---

## 6. Observing the RIO side

The RIO publishes counters to NetworkTables under the `VisionLink` table, about once per second:

| Topic | Meaning |
|---|---|
| `odomSent`, `odomSendErrors`, `odomSkipped` | Stream 1 send health |
| `odomLateTicks` | 5 ms schedule slips |
| `odomStaleTicks` | Ticks where no sensor value had changed |
| `trigSent`, `trigSendErrors`, `trigRecorded`, `trigSeqlockRetries`, `trigLateTicks` | Stream 3 |
| `poseRecvd` | Stream 2 poses accepted |
| `poseDroppedSize` | Dropped for wrong length |
| `poseDroppedStale` | Bad or old timestamp, or non-finite pose |
| `poseDroppedRingFull` | Main loop didn't drain in time |
| `*ThreadAllocBytes` | Per-thread allocation gauges (should stop climbing) |

If you're building a stand-in and `poseDroppedSize` or `poseDroppedStale` climbs, your encoding or your timestamp handling is wrong.

The bench RIO also publishes, under `SpineBench`:

| Topic | Meaning |
|---|---|
| `poseOutOfOrder` | Stream 2 poses whose timestamp was `<=` the newest one already seen. The RIO-side end-to-end check of the Orin's ordering guarantee; should stay at 0 |
| `visionPose`, `visionPoseAgeSec`, `visionPosesSeen` | Latest Stream 2 pose as `[x, y, theta]`, controller time minus its timestamp (-1 before the first), and poses drained |
| `odomPose` | The synthetic Stream 1 pose as `[x, y, theta]`, for side-by-side viewing |

### Orin side: `Spine/Orin/*`

The `spine_stats` node connects to the RIO as an NT4 client (`nt_server`, with a team-number fallback of 10583) and publishes these integer topics about once per second. They cover only what the Orin knows, so they don't duplicate `VisionLink/*`.

| Topic | Meaning |
|---|---|
| `posesSent` | Stream 2 poses handed to the socket |
| `withheldNoFix` | `FusedPose` with `valid = false` or a non-finite pose |
| `withheldBadTimestamp` | `FusedPose` with a NaN `controller_timestamp` |
| `withheldStale` | Older than `max_age_s` relative to the newest trigger, or no trigger yet |
| `withheldOutOfOrder` | Failed the order gate (`ts <= last sent`) |
| `framesUnpaired` | Frames the adapter dropped for having no matching trigger. Present only if the adapter publishes `spine/adapter_stats` |
| `odomRecvd`, `odomSeqGaps` | Stream 1 packets received, and missing `seq` values |
| `trigRecvd`, `triggersMissed` | Stream 3 packets received, and `triggerSeq` values never seen (5+ packets lost in a row) |
| `decodeErrors` | Wrong-length datagrams, or a Stream 3 packet whose entries aren't consecutive and descending |
| `sendErrors` | Stream 2 `sendto` failures |
| `rioRestarts` | RIO restarts detected on Stream 3 (Stream 1 detections are logged as events) |
| `statsAgeMs`, `heartbeat` | Health of the stats path itself |

`spine_stats` also subscribes read-only to the RIO's `VisionLink/*`, and writes both ends side by side into rotating JSON Lines logs on the Orin (`spine_logs/`, 10 MB x 20 files by default), together with every `LinkEvent` (RIO restart, trigger gap, decode error) and NT connects and disconnects. That makes "the RIO sends fine but the Orin never receives" easy to spot afterwards. `ros_ws/scripts/pull_logs.sh` copies them to another machine (rsync over SSH). Whether that pull becomes part of the match routine, or the logs stay Orin-only, is still an **open decision**; see `ros_ws/README.md`.

The companion also publishes `/diagnostics` (`diagnostic_msgs/DiagnosticArray`), WARN when a RIO stream stops, for standard ROS tooling.

---

## 7. Robot-side design constraints (why the code looks the way it does)

You don't need to reproduce these, but they explain choices that constrain the protocol:

- **Zero allocation, lock-free threads.** Every Spine thread on the RIO is measured at zero bytes allocated in steady state, and the hot paths take no locks. That rules out variable-length or self-describing packets: fixed-size records at fixed offsets are decoded straight from preallocated off-heap buffers. Please don't propose JSON, protobuf or variable-length framing.
- **Seqlock on the trigger table.**
  - One thread (the pulser, driven by a hardware-timer alarm) writes trigger records. The Stream 3 sender thread reads them.
  - The writer bumps a version counter to odd, writes the slot, then bumps it back to even.
  - The reader copies the table, then retries if the version was odd or changed during the copy.
  - A packet is therefore always an internally consistent snapshot: entries are consecutive, `triggerSeq` descends by exactly 1, and each timestamp belongs to its sequence number. On the Orin side, you can rely on that consistency within one packet.
- **Periodic threads use the HAL notifier** on the same clock as the timestamps. That is why Stream 1's 5 ms cadence and its timestamps agree with each other.
- **Stream 2 hand-off** from the network thread to the 50 Hz main loop goes through a single-producer/single-consumer ring. That, plus the drain cadence, is why a pose can wait up to about 20 ms on the RIO before it is applied. This latency is compensated automatically, because the replay works from the timestamp, not the arrival time.

---

## 8. Status: done vs. not done

**Implemented and tested on a desktop:**
- The RIO side of all three streams:
  - loopback encode/decode tests;
  - a seqlock stress test under tens of millions of forced collisions;
  - zero-allocation measurement per thread;
  - a check that the odometry math matches WPILib's to 1e-9;
  - a check that zero std-devs give gain = 1, and that the replay works.

**Coprocessor side, implemented and tested on x86 (Ubuntu 26.04, ROS 2 Lyrical):**
- `ros_ws/` (see `ros_ws/README.md`): `spine_interfaces`; `spine_link` (the companion node, a synthetic fused-pose node that plays the adapter+TavSLAM role, and ROS-free protocol / trigger-table / order-gate / pipeline code); `spine_stats` (`Spine/Orin/*` over NT4, rotating JSONL logs); `spine_bringup` (launch files, parameters).
- gtests: golden vectors, bit-exact timestamp echo, every trigger-table case in the plan, the order gate, and the companion's withhold pipeline. pytest: the JSONL logger and the stats mapping.
- A `launch_testing` integration test: a Python fake RIO drives the composed companion and synthetic nodes over loopback UDP, with coalesced, duplicate, reordered and lost trigger packets, a junk datagram and a robot-program restart. It checks that every Stream 2 datagram is 40 B, that every timestamp is bit-identical to a sent trigger's, that timestamps strictly increase, and that each injected fault is counted.
- **Bench RIO** (`frc.robot.bench`): a synthetic Stream 1 source, a 60 Hz trigger pulser (DIO optional) and a Stream 2 monitor. It runs in simulation and builds for the roboRIO.
- **Golden vectors** in `ros_ws/src/spine_link/test/data/spine-link-vectors.txt`, checked by both the Java and the C++ tests.
- **End to end in simulation (2026-09-29):** `./gradlew simulateJava` plus `bench.launch.py sim:=true` for about 45 s: `VisionLink/poseRecvd` 1172 with every `poseDropped*` counter at 0, `SpineBench/poseOutOfOrder` 0, and `Spine/Orin/*` visible in NT. The link threads' allocation gauges stayed flat after startup.

**Bench workflows (PC as a fake coprocessor):**
- **Sim:** run `./gradlew simulateJava` (Orin host `127.0.0.1`) alongside `source ros_ws/scripts/env.sh && ros2 launch spine_bringup bench.launch.py sim:=true`. Under WSL2, `networkingMode=mirrored` is required, and `env.sh` switches Fast DDS to shared memory because WSL2 doesn't deliver DDS discovery multicast.
- **Real RIO:** deploy the bench build, set the `Spine/orinHost` Preference to the PC's IP, restart the robot code, and run `bench.launch.py` on the PC (defaults to the RIO at 10.105.83.2).
- **Healthy link:** `VisionLink/poseRecvd` climbs; `poseDroppedSize`, `poseDroppedStale` and the bench's `poseOutOfOrder` stay at 0; `Spine/Orin/*` is visible in NT. With the synthetic node's default latency jitter, `Spine/Orin/withheldOutOfOrder` climbs too; that is the order gate working.

**Not done or unverified:**
- **Never run on a real roboRIO or Orin.** 200 Hz timing, CAN bus load, trigger pulse timing and the JVM allocation behaviour on the roboRIO are all unmeasured. The Isaac ROS container path (`setup_host.sh --isaac`: Ubuntu 24.04 with NVIDIA's Lyrical build) and the `--plain` Docker path are written but unexercised.
- **Adapter and TavSLAM:** outside this repo. The interface to them is §3.2.
- **Orin address:** `kOrinIp = 10.105.83.11` is a placeholder. The Orin must be given that static address, or the constant (or the `Spine/orinHost` Preference) changed.
- **Trigger wiring:** `kTriggerDioChannel` is disabled (-1) on the bench until the cameras are wired. The 100 µs pulse width has not been checked against the cameras' minimum.
- **No protocol versioning.** Changing any layout means changing both ends in lockstep; nothing detects a mismatch except the size checks. The golden vectors catch drift between the two codebases, not between deployed builds.
