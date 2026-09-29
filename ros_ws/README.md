# Spine coprocessor workspace (`ros_ws/`)

The Orin end of the Spine RIO-Orin UDP link, as a ROS 2 Lyrical colcon workspace. The protocol and every rule these nodes follow are in [`../docs/spine-link.md`](../docs/spine-link.md). Gradle never touches this directory.

| Package | What it is |
|---|---|
| `spine_interfaces` | Messages: `SpineOdometry`, `SpineTrigger`, `FusedPose`, `LinkStats`, `LinkEvent`, `AdapterStats`. RIO time only ever lives in `float64` fields. |
| `spine_link` | C++. `protocol.hpp` (wire format, mirrors `VisionLinkProtocol`), the Stream 3 trigger table, the Stream 2 order gate and withhold pipeline, plus two components: `spine::SpineCompanionNode` (speaks Streams 1, 2 and 3) and `spine::SyntheticPoseNode` (stands in for the adapter + TavSLAM). The golden vectors shared with the Java tests are in `test/data/`. |
| `spine_stats` | Python. Publishes `Spine/Orin/*` to NetworkTables as an NT4 client and writes rotating JSONL logs of both ends. |
| `spine_bringup` | `bench.launch.py`, `orin.launch.py`, `config/spine.yaml`, and the fake-RIO integration test. |

## Platforms

| Where | OS | ROS | Setup |
|---|---|---|---|
| Jetson Orin Nano Super 8 GB | JetPack 7.2 (Ubuntu 24.04) | Lyrical, NVIDIA's build, in the Isaac ROS 5.0 container | `scripts/setup_host.sh --isaac` |
| x86 PC or WSL2, Ubuntu 26.04 | 26.04 | stock Lyrical from packages.ros.org | `scripts/setup_host.sh --native` |
| any Linux PC with Docker, no GPU | any | `ros:lyrical-ros-base` container | `scripts/setup_host.sh --plain` |

Stock Lyrical binaries exist only for Ubuntu 26.04, which is why the Orin uses NVIDIA's container and a PC uses 26.04 or Docker. None of these nodes need CUDA or Isaac packages.

## Scripts

All scripts can be run from anywhere.

| Script | What it does |
|---|---|
| `scripts/setup_host.sh --native\|--isaac\|--plain` | One-time host setup (see Platforms). Safe to re-run. |
| `scripts/build.sh [colcon args]` | Installs any missing package dependencies with rosdep, then `colcon build --symlink-install`. `SPINE_SKIP_ROSDEP=1` skips rosdep. |
| `scripts/test.sh [colcon test args]` | Builds, then runs every test (gtest, pytest, launch_testing) and prints `colcon test-result`. Exits non-zero on any failure. |
| `source scripts/env.sh` | Sets up a shell for `ros2 launch` / `ros2 run`: ROS, this workspace, the pyntcore venv, `SPINE_LOG_DIR`, and the WSL2 DDS fix. |
| `scripts/pull_logs.sh [user@host] [dest]` | Copies the Orin's logs over SSH (see Log retrieval). |
| `scripts/check_gcc13.sh` | Compile-checks `spine_link` with GCC 13, the Orin's compiler (run after `build.sh`; needs `g++-13`). CI runs it. |

Typical PC session:

```bash
ros_ws/scripts/setup_host.sh --native   # once
ros_ws/scripts/test.sh                  # build + all tests
source ros_ws/scripts/env.sh
ros2 launch spine_bringup bench.launch.py sim:=true
```

## Running

- **Bench, fake coprocessor** (companion + synthetic pose node in one process, plus `spine_stats`):
  - against the robot sim on this machine: `ros2 launch spine_bringup bench.launch.py sim:=true`, alongside `./gradlew simulateJava`;
  - against a real RIO: `ros2 launch spine_bringup bench.launch.py` (the RIO is at 10.105.83.2), with the RIO's `Spine/orinHost` Preference set to this PC's IP.
- **Production on the Orin:** `ros2 launch spine_bringup orin.launch.py fused_pose_topic:=<TavSLAM's FusedPose topic>`.
- All parameters, with comments, are in `src/spine_bringup/config/spine.yaml`. The synthetic node's fault injection (`drop_probability`, `invalid_probability`, `stale_probability`) is there too.

## WSL2 notes

- WSL must use `networkingMode=mirrored` so the RIO's UDP reaches the PC's IP.
- WSL2 doesn't deliver the UDP multicast that DDS discovery uses, so ROS nodes in different processes can't find each other. `env.sh` detects WSL and sets `FASTDDS_BUILTIN_TRANSPORTS=SHM`, which works for everything on one machine. The Spine link itself is unicast UDP and isn't affected.

## Log retrieval (open decision)

`spine_stats` always writes rotating JSON Lines logs on the machine it runs on: `spine_logs/spine-orin.jsonl`, 10 MB x 20 files. Each record carries a run id, the wall time and the monotonic time. The records are 1 Hz snapshots of both ends' counters, every `LinkEvent`, and NT connects and disconnects. On the Orin the default location is inside the workspace mount, so the logs survive container restarts.

Whether those logs should also be pulled to the driver-station laptop as part of the match routine is **not decided**. The recommendation is to keep them Orin-local and run `scripts/pull_logs.sh` after a match or in the pits:
- it costs nothing at match time;
- it doesn't depend on the DS being reachable;
- it adds no traffic to the bandwidth-capped field network.
