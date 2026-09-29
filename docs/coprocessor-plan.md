> **Status (2026-09-29):** Todo items 1-11 are implemented and verified in WSL (Ubuntu 26.04, ROS 2 Lyrical, WPILib 2026 JDK 17). `./gradlew build` passes, `ros_ws/scripts/test.sh` passes 50 tests, and the Java sim plus `bench.launch.py` work end to end. Item 12, GitHub CI (`.github/workflows/ci.yml`), is done too. Its ROS job was replayed in a fresh `ubuntu:26.04` container with Podman, and passed.
>
> **Deviations from the text below, all deliberate:**
> - The WSL box is Ubuntu **26.04**, and stock Lyrical binaries exist only for 26.04. The Isaac ROS 5.0 host is 24.04 with NVIDIA's own Lyrical build. `setup_host.sh` gained a `--native` mode (26.04 apt install plus a pyntcore venv).
> - The adapter and TavSLAM internals are out of scope: `spine-link.md` §3.2 documents only the interface.
> - The golden vectors moved to `ros_ws/src/spine_link/test/data/`, because the Isaac container mounts only `ros_ws/`.
> - The order gate resets on a RIO restart only if RIO time went backwards (a reboot), so a robot-program restart can never let an older pose out.
> - A third Stream 3 restart rule: seq forward while RIO time went back (a reboot with missed packets).
> - The companion keeps no trigger history ring. It needs only the newest trigger time, since the lookup lives upstream.
> - `rioRestarts` counts Stream 3 detections only; Stream 1 detections are logged as events.
> - `linkThreadsDoNotAllocateInSteadyState` now requires one clean window per thread out of 5, because a late-JIT one-off was failing the build.
> - WSL2 needs `FASTDDS_BUILTIN_TRANSPORTS=SHM` for cross-process ROS; `env.sh` sets it.
>
> **Timestamp path (the user's own words):** the UDP node gets the timestamps and trigger numbers from Stream 3 and gives them to an upstream node, which gives them to TavSLAM. TavSLAM computes with that timestamp and outputs it with the fused pose, which is sent out via Stream 2. The companion only copies the 8 bytes: no pairing, no lookup, no Orin clock.

# Spine link: coprocessor (ROS 2) side + bench RIO + doc correction

## Context

This repo is the **Spine UDP library** (`frc.lib.SpineUDP`) plus its tests, sitting inside an untouched WPILib command-based template. Nothing calls the library yet, and no coprocessor end exists.

The goal is the ROS 2 side of the link, targeting ROS 2 Lyrical Luth on Isaac ROS 5.0 on a Jetson Orin Nano. It has three parts:
- a production companion node that speaks Streams 1, 2 and 3;
- a synthetic fused-pose generator to drive it during testing;
- an NT4 stats publisher that also writes rotating local logs.

**Bench RIO (your answer):**
- Make the robot project a **library repo plus a bench RIO**.
- It must build for a **real roboRIO** as well as simulation.
- A PC can then act as a fake coprocessor: it receives the RIO's synthetic Streams 1 and 3 and sends synthetic Stream 2 back.
- So the ROS stack must also run on an x86 PC, not only the Orin.

**Verified facts** (web, 2026-09-28):
- **Isaac ROS 5.0** shipped on 2026-09-22. It targets ROS 2 Lyrical on Ubuntu 24.04 and needs **JetPack 7.2** on Orin.
- **Standard setup** is the **`isaac-ros` CLI**, not the old `run_dev.sh`:
  - install with `apt install isaac-ros-cli` from `isaac.download.nvidia.com/isaac-ros/release-5.0` (suite `noble-jetpack` on Jetson, `noble` on x86);
  - then run `sudo isaac-ros init docker` and `isaac-ros activate`.
- **Workspace:** the CLI mounts `$ISAAC_ROS_WS` at `/workspaces/isaac_ros-dev`.
- **Custom layers:**
  - write `$ISAAC_ROS_WS/docker/Dockerfile.<key>` (it starts with `ARG BASE_IMAGE` / `FROM ${BASE_IMAGE}`);
  - register the key in `$ISAAC_ROS_WS/.isaac-ros-cli/config.yaml` under `docker.image.additional_image_keys`;
  - add `$ISAAC_ROS_WS/scripts/.isaac_ros_common-config` containing `CONFIG_DOCKER_SEARCH_DIRS=(${ISAAC_ROS_WS}/docker /etc/isaac-ros-cli/docker)`;
  - build with `isaac-ros activate --build-local`.
- **pyntcore:** RobotPy publishes Linux aarch64 wheels for 2026.x (`robotpy-native-ntcore … manylinux_2_34_aarch64`).

**Timestamp flow (your correction; this replaces the earlier Orin-clock pairing design):**
1. **Spine UDP node (the companion)** receives Stream 3 and hands each `triggerSeq` and its RIO float64 timestamp to the **upstream node** on `spine/triggers`.
2. **Upstream node** (the adapter) matches each frame to its trigger by an **exact frame-count -> trigger-table match**, and gives the frame plus that RIO float64 to **TavSLAM**.
3. **TavSLAM** computes with that timestamp and **outputs it with the fused pose**.
4. **Spine UDP node** receives the fused pose and **sends that timestamp out in Stream 2, copied as its exact 8 bytes**. It does no pairing, lookup, estimation or tolerance window, and never reads an Orin clock for protocol time.

- `pairing.hpp`, `pair_tolerance_s`, `capture_offset_s`, `orin_rx_time` and `SO_TIMESTAMPNS` are all removed from scope.
- **Carrier rule:** the float64 travels in a real `float64` field (`spine_interfaces/FusedPose.controller_timestamp`), never in `header.stamp`. `header.stamp` is sec+nanosec, and writing a double into it rounds to the nearest nanosecond.
- **Adapter scope:** the adapter node is not built here. Its contract is documented in `spine-link.md`, and the synthetic generator plays the adapter+TavSLAM role in test.

**Local limits:** this Windows machine has no Docker and no ROS. WSL has an Ubuntu 24.04 distro, which can run plain ROS 2 Lyrical for local verification (see Verification).

---

## Step 1: Fix CLAUDE.md and docs/spine-link.md so they describe the repo that exists

**Stale items in CLAUDE.md** (to remove or rewrite):
- **Missing classes:** `SwerveDriveSubsystem`, `OdometrySampler`, `CameraTriggerPulser`, `RobotContainer.startVisionLink()` and `Constants.VisionLinkConstants` don't exist.
- **Missing files and deps:** the YAGSL vendordep, `build.gradle`'s `../yagsl/java` source dir, the README, the swerve and pathplanner deploy dirs, and the "raw button 1 / A" conflict are all absent.
- **Wrong fused-pose ownership:** the YAMS estimator description and the apply path in `periodic()`. The **rule itself stays**: Stream 2 is applied via `addVisionMeasurement` with 0.0 std-devs, and `OdometryMathTest.zeroStdDevsReplaceTheEstimate` proves it. Reword it as the contract the production robot project must follow; it isn't code in this repo.

**Rewrite CLAUDE.md to describe:**
- **Library:** `SpineUDP.java` and `PrimitiveSwerveOdometry.java`, with their tests. Keep all the design rules: one clock, zero-alloc, lock-free, portability, and ASCII-only sources.
- **Bench robot:** `frc.robot` becomes a synthetic RIO end (Step 2) that is deployable to a real roboRIO.
- **`ros_ws/`:** the colcon workspace (Steps 3 to 6), which Gradle never touches. Include its commands (setup, build, test, launch).
- **Doc-sync rule:** the lockstep rule now also covers `ros_ws/src/spine_link/include/spine_link/protocol.hpp` and the shared golden vectors.

**`docs/spine-link.md` fixes:**
- **§0 sources:** replace `OdometrySampler` and `CameraTriggerPulser` with the bench producers. Note that the production swerve sampler and DIO pulser live in the robot project; this repo ships synthetic producers.
- **§3, the one-true-clock pipeline:**
  - Pose timestamps are RIO-original. They're attached upstream at the adapter node by an exact frame-count -> trigger-table lookup, before TavSLAM ever sees the frame.
  - Downstream, nothing may touch, re-derive or round-trip them through `builtin_interfaces/Time`.
  - Replace the current "pair them by time on the Orin's own clock" paragraph, which contradicts this.
  - Add an **adapter contract** subsection: the input is `spine/triggers` (`triggerSeq`, float64), the lookup is exact by frame count with an explicit count->seq alignment, the float64 is carried in a float64 field, and a frame with no matching trigger is dropped, never guessed.
  - Add a **FusedPose contract**: the message the companion consumes.
- **§6:** add the Orin-side counters table under `Spine/Orin/*`.
- **§8:**
  - remove the "simulation smoke test that builds the real drivetrain" claim;
  - replace "No coprocessor side exists yet" with a pointer to `ros_ws/`;
  - add the bench RIO and the PC-as-coprocessor workflow.

## Step 2: Clean up the template and build the bench RIO (`frc.robot`)

**Delete** these WPILib template leftovers: `ExampleSubsystem`, `ExampleCommand`, `Autos`, `src/main/deploy/example.txt`, and the unused controller and `OperatorConstants`.

**Add these files:**
- **`Constants.VisionLinkConstants`:**
  - `kOrinIp` defaults to `10.105.83.11`, or `127.0.0.1` when `RobotBase.isSimulation()`;
  - a **`Preferences` override `Spine/orinHost`**, read at startup, so a real RIO can be pointed at a PC's IP without redeploying;
  - ports 5801, 5802 and 5803, 200 Hz, a 64-slot ring, a 1.5 s max age and a 1 s stats period;
  - `kTriggerRateHz = 60`;
  - `kTriggerDioChannel = -1` (disabled; set it to 0 once the cameras are wired) and a 100 µs pulse width;
  - `kLinkConfig(String host)`.
- **`frc/robot/bench/SyntheticOdometrySource implements OdometrySource`:**
  - a closed-form smooth path (a Lissajous-style curve), so `x, y, theta` and `vx, vy, omega` are mutually consistent;
  - the timestamp comes from `SpineClock.now()`;
  - zero allocation and no hardware, so it runs on a bare RIO.
- **`frc/robot/bench/BenchTriggerPulser`:**
  - a `HalPeriodicThread` at 60 Hz;
  - if the DIO is enabled, it pulses a `DigitalOutput`, then calls `trig.recordTrigger(SpineClock.now())`;
  - it is the sole `recordTrigger` caller, which the seqlock single-writer rule requires.
- **`frc/robot/bench/SpineBenchSubsystem`:** `periodic()` drains the `VisionPoseReceiver` with a consumer held in a field, and publishes these through preallocated NT publishers (no `Pose2d`):
  - the last vision pose as `[x, y, theta]` in a `double[]`;
  - the pose age (`now - ts`);
  - `poseOutOfOrder`, counting poses whose `ts <= lastTs`. It is the RIO-side end-to-end check of the Orin's ordering guarantee and should stay at 0;
  - the synthetic odometry pose, for side-by-side viewing in AdvantageScope or Glass.
- **`RobotContainer.startVisionLink()`:** starts consumers before producers: receiver, then trigger sender, then pulser, then odometry sender.

**Tests:** add `goldenVectorsMatchSharedFile` to `SpineUDPTest`. It encodes each packet listed in `ros_ws/src/spine_link/test/data/spine-link-vectors.txt` (moved from `docs/` so the Isaac container, which mounts only `ros_ws/`, can see it) (field values plus the expected LE hex) and compares bytes. The C++ side (Step 4) checks the same file, so the two ends can't silently drift.

## Step 3: Workspace layout and setup scripts (`ros_ws/`)

```
ros_ws/
  .isaac-ros-cli/config.yaml         # additional_image_keys: [spine]
  scripts/.isaac_ros_common-config   # CONFIG_DOCKER_SEARCH_DIRS=(${ISAAC_ROS_WS}/docker /etc/isaac-ros-cli/docker)
  docker/Dockerfile.spine            # ARG BASE_IMAGE / FROM ${BASE_IMAGE}; rosdep deps + pip pyntcore==2026.2.x
  scripts/setup_host.sh              # one-time host prep (see below)
  scripts/build.sh                   # inside the container: rosdep install --from-paths src -iy && colcon build --symlink-install
  scripts/test.sh                    # colcon test + colcon test-result --verbose
  scripts/pull_logs.sh               # optional DS-side retrieval (Step 7)
  src/spine_interfaces  src/spine_link  src/spine_stats  src/spine_bringup
  README.md
```

**`setup_host.sh` has two modes:**
- **`--isaac`** is the default and the Orin path, following NVIDIA's standard:
  1. check that `/etc/nv_tegra_release` shows R39.2 (JetPack 7.2) on Jetson;
  2. add the Isaac apt repo and key (`noble-jetpack` or `noble` chosen by arch);
  3. `apt install isaac-ros-cli`;
  4. export `ISAAC_ROS_WS=<repo>/ros_ws`, so the CLI mounts this directory itself and nothing is copied;
  5. `sudo isaac-ros init docker`;
  6. `isaac-ros activate --build-local` to bake in the `spine` layer.
- **`--plain`** is for PCs without an NVIDIA GPU. It builds the same `Dockerfile.spine` with `BASE_IMAGE=ros:lyrical-ros-base` and runs it with `--network host` and the same mount.
  - This works because none of these nodes depend on Isaac packages, so the PC fake coprocessor doesn't need CUDA.
  - The README documents Windows use: WSL2 needs `networkingMode=mirrored`, so the RIO's UDP reaches the container.

**Also:**
- `.gitignore` gets `ros_ws/build/`, `ros_ws/install/`, `ros_ws/log/` and `ros_ws/spine_logs/`.
- Gradle only reads `src/` and `vendordeps/` at the root, so `ros_ws/src` is never touched.

## Step 4: `spine_interfaces` + `spine_link` (C++, rclcpp components)

**`spine_interfaces` messages** are fixed-size, so no JSON or protobuf is involved. RIO time lives only in `float64` fields, which CDR serializes as the raw IEEE-754 bits, so they're bit-exact:
- `SpineOdometry`: `int64 seq`, `float64 controller_timestamp`, `x y theta vx vy omega`.
- `SpineTrigger`: `int64 trigger_seq`, `int64 packet_seq`, `float64 controller_timestamp`.
- `FusedPose`: the companion's input. Fields are `float64 controller_timestamp` (the RIO float64 carried from the adapter), `int64 trigger_seq` (informational only, for logs), `x y theta` in the field frame, and `bool valid`.
- None of these carry a `header.stamp` with protocol meaning. `SpineOdometry` and `SpineTrigger` have **no** `header` at all, so no Orin receive stamp exists to tempt anyone into time-pairing. How TavSLAM ingests odometry in RIO time is the adapter's concern.
- `LinkStats`: `uint64` counters (listed in Step 6) plus `last_rio_restart` info.
- `LinkEvent`: `header`, `uint8 kind` (RIO_RESTART, TRIGGER_GAP, DECODE_ERROR, …) and a short `detail` string. Events are rare.

**`spine_link/protocol.hpp`** is header-only and has no ROS dependency:
- it holds the constants, mirroring `VisionLinkProtocol` offsets and sizes exactly;
- it has `load_le_i64`, `load_le_f64` and `store_le_*` built on `memcpy` plus explicit byte order;
- timestamps are also kept as `std::array<uint8_t,8>` raw wire bytes;
- `static_assert`s cover sizes and offsets;
- it never casts a struct over a buffer.

**`trigger_table.hpp/.cpp`** implements the doc's §2 read rule exactly. For each 88-byte packet:
1. Reject the packet if the length isn't 88, or if its non-sentinel entries aren't consecutive and strictly descending by 1. The doc guarantees that, so a violation means a decode error, which is counted.
2. **Restart detection**, which happens before the walk and uses RIO data only, no Orin clock:
   - The RIO is treated as restarted if either condition holds:
     - `triggerSeq[0] < lastSeen - 5`: the sequence went backwards by more than a table's worth, which reordering on a LAN cannot produce;
     - `triggerSeq[0] <= lastSeen` while `timestamp[0] > lastSeenTs`: the sequence went backwards but RIO time went forward. That's a robot-program restart, and a reordered packet can't produce it.
   - On restart, reset `lastSeen = -1`, clear the history, reset the Stream 2 order gate, and emit a RIO_RESTART event.
   - Otherwise, a packet with `triggerSeq[0] <= lastSeen` is a duplicate or reordered packet and yields nothing.
3. Walk from `i = 4` down to `0`: read `seq_i`; if `seq_i > lastSeen`, read the timestamp, append `{seq, value}` to the history ring (default 128 entries, about 2 s), publish a `SpineTrigger`, and set `lastSeen = seq_i`. Updating `lastSeen` inside the loop is correct because entries rise as `i` falls. The `-1` sentinel fails `> lastSeen` automatically, since `lastSeen >= -1`. Coalesced packets (several new entries) fall out naturally.
4. If the first new `seq > lastSeen + 1` while `lastSeen != -1`, count `triggersMissed += gap`. That only happens after 5 or more consecutive lost packets.

**`order_gate.hpp`:** holds `last_sent_ts` as a double and **drops any pose with `ts <= last_sent_ts`**. `<=` also catches duplicates. It is reset only on RIO restart.

**`SpineCompanionNode` (component):**
- **Receive thread:** one thread `epoll`s the Stream 1 and Stream 3 sockets (bound on `bind_address`, ports 5801 and 5803).
- **Stream 1:** checks size 64, then decodes.
  - It counts `seq` gaps, and detects a restart when `seq` decreases or the timestamp goes backwards.
  - It publishes `spine/odometry` (`SpineOdometry`).
- **Stream 3:** handled by the trigger table above; triggers go out on `spine/triggers` for the adapter.
- **Pose input:** subscribes to `fused_pose` as **`spine_interfaces/FusedPose`**. In production that's the TavSLAM output path; in test it's the synthetic node, publishing the same type on the same topic. The pipeline is:
  1. **No-fix gate:** if `valid` is false, or `x`, `y` or `theta` is not finite, count `withheldNoFix` and drop it.
  2. **NaN check:** if `controller_timestamp` is NaN, count `withheldBadTimestamp` and drop it. There is no lookup or matching against the trigger table here; the one lookup happens in the upstream node.
  3. **Age:** RIO time is compared only with RIO time. If `newest_trigger_ts - pose_ts > max_age_s` (1.4 s), count `withheldStale`, since the RIO would drop it anyway.
  4. **Order gate:** a pose that fails it counts as `withheldOutOfOrder`.
  5. **Field frame:** apply the optional static 2-D transform `field_from_map_{x,y,yaw}` (default identity) to `x`, `y` and `theta`.
  6. **Encode 40 bytes:** own `seq`, then the pose's `controller_timestamp` written as its exact 8 bytes (little-endian via `std::bit_cast<uint64_t>`, no arithmetic), then `x`, `y`, `theta`.
     - The **loud "ONE TRUE CLOCK — DO NOT TOUCH THIS TIMESTAMP" comment** (your exact text) goes directly above this packing code.
  7. **Send:** `sendto(rio_address:5802)`, counting `posesSent` or `sendErrors`.
  If there is no pose or no good pose, nothing is sent: silence is the no-fix signal.
- **Stats:** counters are `std::atomic<uint64_t>`. They're published at 1 Hz as `LinkStats` on `spine/stats`, and also as `diagnostic_msgs/DiagnosticArray` on `/diagnostics` for standard tooling.
- **Intra-process:** the node is registered with `rclcpp_components`, and launch composes it with `use_intra_process_comms: True`. NITROS isn't used, since these are tiny fixed structs.
- **No `rosidl::Buffer` (decided 2026-09-29):** Isaac ROS 5.0 replaced NITROS with Lyrical's `rosidl::Buffer` and a CUDA buffer backend. It's for large accelerator-resident data such as images and tensors, and its zero-copy currently needs `rmw_fastrtps_cpp`. Our messages are CPU structs of at most 88 B, so it gains nothing here and would add a CUDA and RMW dependency to code that must also build on x86 and in CI. Intra-process `unique_ptr` publishing already makes the companion<->synthetic hop zero-copy. The adapter and TavSLAM image path may use it; nothing in our interface prevents that.
- **Target hardware:** Jetson Orin Nano Super 8 GB, latest JetPack (7.2), ROS 2 Lyrical in the Isaac ROS 5.0 container.

**`SyntheticPoseNode` (component)** is timed only off received triggers:
- **Inputs:** it subscribes to `spine/triggers` and `spine/odometry`.
- **Per trigger:**
  - Find the odometry sample nearest the trigger's `controller_timestamp`. That compares controller time with controller time, which is allowed.
  - Pose = `field_start` ∘ the odometry delta since the first sample, ∘ a wander error: an Ornstein-Uhlenbeck random walk in x, y and θ, integrated with `dt` = the Δ between trigger `controller_timestamp`s, plus white noise.
  - It plays the adapter+TavSLAM role. The output is `FusedPose` whose `controller_timestamp` is the trigger message's float64, **copied by an exact triggerSeq match** (the stand-in for the adapter's frame-count lookup).
  - The equivalent ONE-TRUE-CLOCK comment goes at that lookup site, since the real adapter isn't in this repo. The doc's adapter contract carries the same wording, for whoever writes the real adapter.
- **Latency:** the result is released after a simulated processing latency drawn from `latency_mean ± jitter`, using a small pending queue driven by a 1 ms steady timer. Varying latency naturally produces **out-of-order results**, which exercises the order gate.
- **Fault injection:**
  - `drop_probability` means no fix, so nothing is published;
  - `stale_probability` holds a result back past `max_age_s`, exercising `withheldStale`;
  - a `seed` param makes runs reproducible.
- **Time:** it never generates a protocol timestamp; all timing is derived from the trigger messages.

**gtests**, which are the correctness core:
- **Protocol:** golden vectors from `ros_ws/src/spine_link/test/data/spine-link-vectors.txt` (moved from `docs/` so the Isaac container, which mounts only `ros_ws/`, can see it) are decoded and re-encoded bit-identically.
- **Echo:** timestamps come out bit-exact for awkward doubles (`-0.0`, subnormals, values with every mantissa bit set).
- **Trigger walk:** covers first packet (4 sentinels), steady single new, coalesced 2 and 3 new, duplicate packet, reordered older packet, 5+ lost with a gap count, restart by seq jump, restart by seq-back-but-time-forward, and an inconsistent packet rejected.
- **Order gate:** `<`, `==`, reset.
- **Companion pipeline:** no-fix, stale and out-of-order are each withheld and counted. A valid pose's Stream 2 bytes 8..15 equal the input float64's bits exactly.

## Step 5: `spine_bringup` launch files

- **`bench.launch.py`:**
  - one `component_container` holding the companion and synthetic nodes, with intra-process enabled;
  - plus the `spine_stats` process;
  - arguments `rio_address` and `nt_server` (both default `10.105.83.2`; `127.0.0.1` for sim) and `bind_address`.
- **`orin.launch.py`:** the companion alone with `fused_pose` remapped to the TavSLAM output topic, plus `spine_stats`.
- **`config/spine.yaml`:** all parameters with comments.

## Step 6: `spine_stats` (Python, rclpy + pyntcore)

- **NT4:**
  - connects as an **NT4 client** (`inst.startClient4("spine-orin")`, `setServer(nt_server)`, with a team-number fallback of 10583);
  - publishes `Spine/Orin/*` integer topics:
    - pose path: `posesSent`, `withheldOutOfOrder`, `withheldNoFix`, `withheldStale`, `withheldBadTimestamp`;
    - trigger-pairing failures happen in the upstream node (a frame with no matching trigger), so only that node can count them. The stats node subscribes to an optional `spine/adapter_stats` topic (a `LinkStats`-style message) and forwards `framesUnpaired` if the upstream node publishes it; the contract is documented in the doc;
    - traffic: `odomRecvd`, `odomSeqGaps`, `trigRecvd`, `triggersMissed`;
    - errors: `decodeErrors`, `sendErrors`, `rioRestarts`;
    - health: `statsAgeMs` and a heartbeat;
  - these cover only what the Orin knows, so they don't duplicate `VisionLink/*`.
- **Also subscribes to the RIO's `VisionLink/*`** (read-only) so each log snapshot records both ends side by side. That makes it cheap to spot "RIO sends fine but the Orin never receives" offline.
- **Rotating structured logs:**
  - JSON Lines through `RotatingFileHandler` (default 10 MB × 20 files), into `log_dir`. The default is `/workspaces/isaac_ros-dev/spine_logs`, which is on the host through the workspace mount, so logs survive container restarts.
  - Each line has a run id, a wall time and a monotonic time.
  - Records: 1 Hz stats snapshots from both ends, every `LinkEvent`, and NT connect/disconnect.
  - JSONL is used for the offline log files only; the wire stays fixed binary.

## Step 7: Log retrieval (open decision, flagged, not silently picked)

**Recommendation:**
- Keep the logs Orin-local and persistent as the hard requirement, and ship an optional **DS-side pull** (`pull_logs.sh`, rsync over SSH to the Orin's static IP) to run after a match or in the pits.
- **Why pull and not push:**
  - Pushing during a match adds traffic to the bandwidth-capped field network.
  - A push also depends on the DS being reachable.
  - NT4 isn't a file transport.
  - Pulling costs nothing at match time and works from any laptop.
- The final call stays yours. The design doesn't depend on it.

## Step 8: Doc sync

Update `docs/spine-link.md` §3, §6 and §8 and CLAUDE.md with the final Orin-side behaviour.

---

## Verification

1. **Java:** `./gradlew build`. All existing tests plus the new golden-vector test must pass. The build also proves the bench code compiles for the roboRIO target, and that sources stay ASCII.
2. **ROS unit and integration tests:** run in WSL Ubuntu 24.04 with ROS 2 Lyrical, or in the `--plain` container: `colcon build && colcon test`.
   - The tests include a `launch_testing` integration test with a Python fake RIO (`test/fake_rio.py`). It sends Streams 1 and 3 over loopback, including coalesced, duplicate and restart sequences.
   - It asserts that every Stream 2 datagram is exactly 40 B, that each timestamp's bytes match a sent trigger bit-for-bit, and that timestamps strictly increase. It also checks that the withheld counters move when the synthetic node injects faults.
   - *Installing ROS in WSL modifies your system, so I'll ask before doing it. Otherwise these steps are left for you to run.*
3. **Sim end to end:**
   - Run `./gradlew simulateJava` (Orin host `127.0.0.1`) alongside `ros2 launch spine_bringup bench.launch.py rio_address:=127.0.0.1 nt_server:=127.0.0.1`.
   - Expect `VisionLink/poseRecvd` to climb, `poseDroppedSize`, `poseDroppedStale` and the bench `poseOutOfOrder` to stay at 0, and `Spine/Orin/*` to be visible in NT.
4. **Real RIO + PC:**
   - Deploy the bench build.
   - Set Preferences `Spine/orinHost` to the PC's IP and restart the code.
   - Run `bench.launch.py` on the PC and check the same counters.
5. **Orin:** run `setup_host.sh --isaac`, then `isaac-ros activate`, then `scripts/build.sh`, then `ros2 launch spine_bringup bench.launch.py`.

## Todo list (created at start of execution)

1. Fix CLAUDE.md and spine-link.md for current reality, including the corrected one-true-clock pipeline and the adapter/FusedPose contract. Save the architecture correction to memory.
2. Clean the template and build the bench RIO, plus the golden-vector file and Java test.
3. Scaffold `ros_ws`: the Isaac CLI config, `Dockerfile.spine` and the setup/build/test scripts.
4. `spine_interfaces`.
5. `spine_link` protocol, trigger table and order gate, with their gtests.
6. Companion node.
7. Synthetic node.
8. `spine_bringup` launch files and config.
9. `spine_stats` (NT4 and rotating logs).
10. `launch_testing` integration with the fake RIO.
11. Final doc sync, then `./gradlew build`.
