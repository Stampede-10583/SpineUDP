# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Team 10583's **Spine link**: the UDP transport between a robot controller (roboRIO 2 today, SystemCore later) and an Orin Nano running **TavSLAM** (the team's name for the SLAM stack; keep it). The repo has three parts:

- **Library** (`src/main/java/frc/lib/SpineUDP/`): the controller end of the link. It's meant to be copied into, or depended on by, the real robot project.
- **Bench RIO** (`src/main/java/frc/robot/`): a WPILib 2026 Java robot program that runs the library with synthetic producers and no drivetrain. It deploys to a real roboRIO and runs in simulation, so a PC can act as a fake coprocessor.
- **`ros_ws/`**: the colcon workspace for the coprocessor end (ROS 2 Lyrical on Isaac ROS 5.0). It must build and run on the Orin and on an x86 PC. Gradle never touches it.

**Status:** everything in `docs/coprocessor-plan.md` is implemented and tested in simulation, and GitHub CI (`.github/workflows/ci.yml`) runs both halves. Nothing has run on a real roboRIO or Orin yet.

**CI** has two jobs. `java` runs `./gradlew build` on JDK 17 plus the ASCII check. `ros` runs `setup_host.sh --native` and `scripts/test.sh` in an `ubuntu:26.04` container, on x86_64 and, for public repos, arm64; it also runs `scripts/check_gcc13.sh`. Keep both green: a change that passes locally should pass there with the same scripts.

**Not in this repo:** the production robot code (swerve drivetrain, the real Stream 1 sampler, the DIO camera pulser, the pose estimator that applies Stream 2) and the adapter node that feeds TavSLAM. Their contracts are in `docs/spine-link.md`.

## Commands

### Java (Gradle wrapper with GradleRIO)
Use `./gradlew` (on Windows, `gradlew.bat`). Add `--offline` when the dependency cache is warm.
- **Build and run tests:** `./gradlew build`
- **Compile only:** `./gradlew compileJava`
- **All tests:** `./gradlew test`
- **One test class or method:** `./gradlew test --tests '*OdometryMathTest'` or `--tests '*linkThreadsDoNotAllocateInSteadyState'`. Add `-i` to see test stdout (the allocation and seqlock tests print diagnostics).
- **Simulate (GUI + DS sim):** `./gradlew simulateJava`
- **Deploy the bench build to a roboRIO:** `./gradlew deploy`

**Test setup:**
- JUnit 5 with `wpi.java.configureTestTasks(test)`, so the HAL, NT and sim natives are available in tests. Tests that touch the HAL call `HAL.initialize(500, 0)` first.

**Keep Java sources ASCII-only**, comments included. `build.gradle` sets no source encoding, so javac uses the platform default (Windows-1252 on Windows). A non-ASCII character that compiles fine here can fail the build on Windows.

### ROS 2 (`ros_ws/`)
`ros_ws/README.md` has the details. Stock ROS 2 Lyrical binaries exist only for Ubuntu 26.04; the Orin runs NVIDIA's Lyrical build (Ubuntu 24.04) inside the Isaac ROS 5.0 container. Code must build on both (C++20, GCC 13 and 15, Python 3.12 and 3.14).
- **Setup (once):** `ros_ws/scripts/setup_host.sh --native` (Ubuntu 26.04 PC/WSL/CI), `--isaac` (Orin; uses the `isaac-ros` CLI, not `run_dev.sh`) or `--plain` (Docker on `ros:lyrical-ros-base`).
- **Build:** `ros_ws/scripts/build.sh`, which runs `rosdep install` for missing deps, then `colcon build --symlink-install`.
- **Build and test everything:** `ros_ws/scripts/test.sh` (gtest, pytest and the `launch_testing` fake-RIO integration test). One package: `ros_ws/scripts/test.sh --packages-select spine_link`.
- **Shell for ros2 commands:** `source ros_ws/scripts/env.sh`. It loads ROS, the overlay, and the venv's `pyntcore` via `PYTHONPATH` (ROS Python nodes get a `/usr/bin/python3` shebang). Under WSL2 it sets `FASTDDS_BUILTIN_TRANSPORTS=SHM`, because WSL2 drops DDS discovery multicast.
- **Launch:**
  - bench (companion, synthetic pose node, stats): `ros2 launch spine_bringup bench.launch.py sim:=true` for the robot sim on this machine, or no args for a real RIO at 10.105.83.2;
  - production (companion with `fused_pose` remapped to TavSLAM's output, plus stats): `ros2 launch spine_bringup orin.launch.py fused_pose_topic:=<topic>`.
- **PC as fake coprocessor under WSL2:** WSL must use `networkingMode=mirrored` so the RIO's UDP reaches it. Point the RIO at the PC with the `Spine/orinHost` Preference; in sim the host is `127.0.0.1`.
- **Java on this box:** the WPILib 2026 JDK is at `~/wpilib/2026/jdk`; run Gradle as `JAVA_HOME=~/wpilib/2026/jdk ./gradlew build` (the system JDK 25 can't run Gradle 8.11).

## Architecture

### The one-clock rule
- **One clock.** The controller's clock is the **only** timebase, and every timestamp is `MathSharedStore.getTimestamp()`, reached through `SpineClock`. The Orin never stamps anything; it echoes controller timestamps back verbatim.
  - Don't introduce `System.nanoTime`, `Timer`, FPGA reads, Phoenix signal timestamps, or any Orin-side clock sync into the link. That is treated as an architectural regression.
  - The `libraryUsesOnlyTheSharedClockAndNoRobotCode` test scans the library source to enforce this.
- **The timestamp path on the Orin** (this replaced an earlier design that paired frames with triggers on the Orin's clock; don't bring that back):
  1. The companion node receives Stream 3 and publishes each new `triggerSeq` with its RIO float64 timestamp on `spine/triggers`.
  2. The upstream **adapter** node matches each camera frame to its trigger by an **exact frame-count -> trigger-table match** and hands TavSLAM the frame plus that float64.
  3. TavSLAM outputs the fused pose carrying the same float64, as `spine_interfaces/FusedPose.controller_timestamp`.
  4. The companion copies those exact 8 bytes into Stream 2.
  - The companion does **no** pairing, lookup, arrival-time estimation or tolerance window, and never reads an Orin clock for protocol time.
  - The float64 always travels in a real `float64` field, **never** in `header.stamp` or `builtin_interfaces/Time`. Those are sec+nanosec, so a double doesn't round-trip through them.
  - A `// ONE TRUE CLOCK - DO NOT TOUCH THIS TIMESTAMP` comment sits above the companion's Stream 2 packing code and at the synthetic node's `triggerSeq` lookup. Keep both.
- **Stream 2 ordering:** the companion never sends a pose whose timestamp is `<=` one it already sent, and sends nothing when it has no good pose. On a detected RIO restart the order gate is reset only if RIO time went backwards (a reboot); a robot-program restart keeps the RIO clock running, so the gate is kept.

### Contract for the robot project that applies Stream 2
This code lives in the production robot project, not here; `OdometryMathTest` proves the math.
- Apply each Stream 2 pose with `addVisionMeasurement(pose, ts, stdDevs)` using std-devs of **0.0**. That makes the Kalman gain exactly 1, so the Orin pose *replaces* the estimate at its timestamp, and the estimator's 1.5 s buffer replays the odometry since then (`zeroStdDevsReplaceTheEstimate`, `pastPoseIsReplayedForward`). Use a single estimator; don't add a second one.
- WPILib's `addVisionMeasurement` deletes vision updates newer than the one being added, so an out-of-order pose rolls back newer corrections. The RIO does not reorder; the Orin's order gate is what prevents this.

### The UDP link (`frc.lib.SpineUDP`, formerly KatanaUDP)
**`docs/spine-link.md` is the protocol reference.** It covers byte layouts, time semantics, delivery behaviour, what the RIO does with each pose, the adapter and FusedPose contracts, and what's still TODO. It is written for engineers building the coprocessor end.
- **Keep these in lockstep** (there is no version field): `VisionLinkProtocol` in `SpineUDP.java`, `ros_ws/src/spine_link/include/spine_link/protocol.hpp`, the shared golden vectors in `ros_ws/src/spine_link/test/data/spine-link-vectors.txt`, and `docs/spine-link.md`. Any change to the layouts, the ports, the validity checks or the apply contract updates all of them.
- **Things easy to get wrong:**
  - Timestamps are **float64 seconds**; sequence numbers are int64.
  - Stream 3 entries are most-recent-first; walk them from index 4 down to 0.

`SpineUDP.java` is a single namespace file holding the whole controller side of a three-stream protocol (little-endian; offsets defined only in `VisionLinkProtocol`):

| Stream | Direction | Rate | Thread | Bench producer/consumer |
|---|---|---|---|---|
| 1: odometry (64 B) | RIO -> Orin | 200 Hz | `OdometrySender` on a `HalPeriodicThread` | `SyntheticOdometrySource` (an `OdometrySource`) |
| 2: fused pose (40 B) | Orin -> RIO | event | `VisionPoseReceiver` blocking-receive thread -> SPSC ring | drained in `SpineBenchSubsystem.periodic()` |
| 3: trigger table (88 B) | RIO -> Orin | ~60 Hz | `TriggerTableSender` park/unpark thread | `BenchTriggerPulser` (optional DIO pulse, then `recordTrigger`) |

- The bench classes live in `frc.robot.bench`. `RobotContainer.startVisionLink()` wires and starts everything, consumers before producers: receiver, trigger sender, pulser, odometry sender.
- The config is `Constants.VisionLinkConstants.kLinkConfig(host)`. The Orin host defaults to `10.105.83.11` (`127.0.0.1` in sim) and can be overridden with the `Spine/orinHost` Preference.
- The RIO publishes counters under `VisionLink/*`, and the Orin under `Spine/Orin/*`; both lists are in the doc (§6).

**Design rules the library holds to** (keep them when editing):
- **Zero allocation in steady state** on every link thread:
  - Use preallocated direct `ByteBuffer`s, primitive samples and `double[]` scratch.
  - No varargs, capturing lambdas, boxing, or WPILib geometry objects on hot paths.
  - `OdometrySource` implementations run on the odometry thread and must not allocate either. In the robot project, Phoenix `BaseStatusSignal.refreshAll` allocates, so refresh signals individually.
  - The only accepted allocation is the per-pose `Pose2d` passed to the WPILib estimator on the main thread.
  - `linkThreadsDoNotAllocateInSteadyState` measures this per thread.
- **Lock-free:**
  - Periodic threads call `NotifierJNI` directly through `HalPeriodicThread`, because the WPILib `Notifier` class locks on every callback.
  - Stream 3 hands off through a single-writer seqlock (`TriggerRing`); `recordTrigger` must be called from one thread only.
  - Stream 2 uses an SPSC ring.
  - Counters are single-writer opaque longs (`Counter`).
- **Stream 1 content:** pure wheel+gyro dead reckoning (or the bench's synthetic path), never the vision-corrected pose, because that would feed the Orin its own output. An `OdometrySource` must never touch the vision-fused estimator.
- **Portability:**
  - The library is reused on SystemCore, so it has no `frc.robot` imports; the robot passes a `Config` in.
  - WPILib touchpoints are confined to `SpineClock`, `HalPeriodicThread` and `StatsPublisher` (plus `PoseView.getPose2d()`).
  - Hardware-specific code (DIO pulsing, sensor sampling) stays in `frc.robot` or the production robot project.

`PrimitiveSwerveOdometry` is an allocation-free port of `SwerveDriveOdometry.update` (closed-form forward-kinematics pseudo-inverse and the `Pose2d.exp` twist), for the production Stream 1 sampler. `OdometryMathTest` holds it to 1e-9 against WPILib, so any math change must keep that test passing.

### Coprocessor side (`ros_ws/`)
- `spine_interfaces`: `SpineOdometry`, `SpineTrigger`, `FusedPose`, `LinkStats`, `LinkEvent`, `AdapterStats`. RIO time lives only in `float64` fields; `SpineOdometry` and `SpineTrigger` have no `header`.
- `spine_link` (C++20):
  - **ROS-free core** (`spine_link_core`, unit-tested without ROS): `protocol.hpp` does explicit byte-by-byte little-endian loads and stores plus `std::bit_cast`, never a struct cast. `TriggerTable` holds the Stream 3 read rule and RIO-restart detection. `OdometryTracker` holds Stream 1 gaps and restarts. `OrderGate` drops `ts <= last`. `PosePipeline` is the FusedPose -> Stream 2 withhold/encode path.
  - **Components:** `spine::SpineCompanionNode` runs one epoll receive thread for Streams 1 and 3; the executor handles `fused_pose`, and the two share state under a mutex. `spine::SyntheticPoseNode` is timed only off received triggers, with latency, drop, invalid and stale injection.
- `spine_bringup`: `bench.launch.py` composes the companion and synthetic nodes with intra-process comms, plus `spine_stats`; `orin.launch.py`; `config/spine.yaml`; `test/fake_rio.py` and the integration test.
- `spine_stats` (Python, rclpy + pyntcore): publishes `Spine/Orin/*` as an NT4 client, mirrors the RIO's `VisionLink/*` and `SpineBench/*` read-only, and writes rotating JSONL logs to `$SPINE_LOG_DIR` (`ros_ws/spine_logs/`). Whether logs are also pulled to the DS (`scripts/pull_logs.sh`) is an **open decision**; don't settle it silently.
- **No `rosidl::Buffer` or NITROS:** the messages are tiny CPU structs, and intra-process `unique_ptr` publishing is already zero-copy. The rationale is in `docs/coprocessor-plan.md`.
