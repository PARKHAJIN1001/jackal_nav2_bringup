# Staged startup patch — 2026-09-17

## Scope and evidence

The requested sequence is `nav_bringup.launch.py` → manual initial pose →
separately launched `mid360_bringup/perception.launch.py`.

The previous gate checked publisher discovery, so constructor-created endpoints
could advance launch without producing measurements. In the saved 18:53 and
18:55 launch sessions, FAST-LIVO2 exited with `IMU application buffer limit
reached` before AMCL received the manual initial pose. Relay logs also reported
stale LiDAR measurements and receipt timeouts. This establishes a broken input/
odometry chain in those sessions; it does not establish why upstream LiDAR
delivery stopped or demonstrate an AMCL parameter defect.

## Implementation

- `scripts/topic_ready_gate.py` now subscribes to messages and requires fresh,
  increasing timestamps and continuous receipt. Bad frames, malformed/empty
  inputs, invalid odometry and startup pose discontinuities cannot open a gate.
  Cancellation returns a nonzero exit code. Deadlines use a steady clock.
- `launch/nav_bringup.launch.py` waits for cloud+IMU, then FAST-LIVO2 odometry,
  then active AMCL/map_server, a map, scan+adapted odometry, and scan-time odom
  TF. A short bounded scan-header history allows TF computation to lag scan
  projection without accepting stale transforms. No map→odom is required
  before the operator's initial pose.
- FAST-LIVO2/relay process exits stop the launch, including child processes
  created inside the upstream launch's `OpaqueFunction`. Gate failure or
  shutdown cannot intentionally advance to the next phase.
- The initial-pose prompt is followed by generated perception profiles and the
  full command for a separate terminal. The helper propagates `use_sim_time`.
  Overrides select the local relay, odom tracking, timestamped TF, and avoid
  duplicate base/LiDAR and LiDAR/IMU static transforms.
- The raw camera Image display is disabled in a temporary RViz profile by
  default; `use_camera_image:=true` retains the supplied profile's settings.
  Source RViz and upstream perception YAML files are preserved.

Default readiness interval: 5 s. Maximum message age/gap: 0.3 s; future
tolerance: 0.05 s. Sensor and odometry gate deadlines: 60 s; localization input
deadline: 90 s. Keep the robot stationary during startup. These thresholds and
their limits are documented in the README.

## Verification

- All Python regression tests: **192 passed**.
- CMake build succeeded in `/tmp/jackal_amcl_rotation_patch_20260917/build`.
- `ros2 launch ... nav_bringup.launch.py --show-args` succeeded without starting
  launch processes; the installed profile helper generated valid overrides.
- Changed Python files passed `ament_flake8` and `ament_pep257`.
- `test/integration_topic_ready_gate.py` passed in localhost-only ROS domain
  187 with synthetic inputs: publisher-only timeout; cloud+IMU freshness;
  lifecycle/map/scan-time TF readiness with 50 ms TF lag and no map→odom.
  Clean-run logs: `/tmp/nav_gate_integration_wxmqo13i/`.
- The existing installation symlinks point to the edited launch and scripts.

The DDS test used mock lifecycle services, not real AMCL or FAST-LIVO2. No real
robot stack was launched, and no initial poses, goals or velocity commands were
sent. This patch addresses startup sequencing and exit propagation. LiDAR
transport continuity, actual scan-map alignment and motion localization still
need hardware verification. No CPU/RAM protection policy was added.

Subsequent hardware testing reproduced the LiDAR interruption and identified
IP reassembly pressure as a major contributor. See the
[live pilot comparison](Staged_Network_Pilot_2026-09-17.md); startup gate success
in synthetic tests does not establish network continuity on the robot link.
