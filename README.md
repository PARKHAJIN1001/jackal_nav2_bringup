# jackal_nav2_bringup

AMCL and Nav2 integration for a Jackal whose smooth local pose comes from an
external FAST-LIVO2 process.

Current baseline and agreed next steps: [2026-09-18 session handoff](docs/Session_Handoff_2026-09-18.md).
With the laptop IP reassembly limit temporarily set to 128MiB, Nav2 plus separate
perception maintained scan/map alignment during manual spot turns. The user
accepted current localization as the development baseline. RViz now uses the
native Nav2 Goal tool and Navigation 2 panel for goals, feedback and cancellation.
The default single-goal tree replans at 1 Hz without automatic recovery spins,
backups or costmap clearing. Additional localization/network tuning is deferred.
Hardware autonomous driving still requires the attended acceptance procedure;
motion remains disabled by default.
See the [measured results](docs/validation/2026-09-18/z_axis_test_1351/REPORT.md).

Recommended entry point: `nav_session.py nav` for stationary and motion-enabled sessions.
It runs the single staged definition in `nav_bringup.launch.py`; `bringup.launch.py`
is now only an alias. See [startup stability and operator-run validation](docs/Startup_Stability.md),
[operations](docs/Operations.md), and [goal navigation](docs/Goal_Navigation.md).
The full-stack monitor requires 180 continuous healthy seconds within a 600-second
acquisition/recovery window. Readiness is revoked on faults; motion remains opt-in.
This change has offline verification only; hardware acceptance is pending.

Historical investigations remain available in the
[September 17 network pilot](docs/Staged_Network_Pilot_2026-09-17.md),
[staged startup patch](docs/Staged_Startup_Patch_2026-09-17.md),
[September 16 validation](docs/Connected_Validation_2026-09-16.md),
[restart trials](docs/Relaunch_Reproduction_2026-09-16.md),
[upstream review](docs/Upstream_Review_2026-09-14.md),
[FAST follow-up](docs/FAST_LIVO_Followup_2026-09-14.md),
[stationary pilot](docs/Stationary_Pilot_2026-09-14.md),
[September 11 implementation](docs/Offline_Implementation_2026-09-11.md) and
[September 10 handoff](docs/Session_Handoff_2026-09-10.md).
Their acceptance status and next-work recommendations describe those sessions,
not the current baseline.

## Frame and topic ownership

The navigation stack uses the global `/tf` and `/tf_static` topics:

```text
map                   AMCL owns map -> odom
 `-- odom             FAST-LIVO2 owns odom -> base_link
      `-- base_link   an external static publisher owns sensor transforms
           `-- livox_frame
```

The existing Clearpath wheel/EKF tree on `/j100_0519/tf` is intentionally kept
separate and is not consumed by this package. Do not place FAST-LIVO2 or this
Nav2 stack in the `j100_0519` TF namespace unless the Clearpath
`odom -> base_link` broadcasters have first been disabled.

Required external inputs are:

- `/aft_mapped_to_init` (`nav_msgs/msg/Odometry`) with frames
  `odom -> base_link`; FAST-LIVO2 must also publish this transform on `/tf`.
- `/livox/lidar` (`sensor_msgs/msg/PointCloud2`) with a valid global TF path
  to `base_link`, used by the independent stop monitor and the default AMCL
  scan projection.
- A real occupancy-grid map YAML and the image referenced by it.

The staged launch starts the relay, FAST-LIVO2, scan projection and Nav2 in that order.
The MID-360 driver and Clearpath platform remain external robot services.
The relay input defaults to `/livox/lidar`, output to `/livox/lidar_local`; it preserves
measurement stamps and payload. Input/output aliases must differ. Diagnostic ages
are reported on `/nav2/lidar_relay_diagnostics`.

`bringup.launch.py` shares the staged implementation and starts FAST-LIVO2 too.
Its former external-FAST/composition flags are rejected. Custom experiments can
still use `localization.launch.py` and `navigation.launch.py`; those lower launches
also run the network preflight and disable automatic respawn. They do not create
a composition container. Direct launches have no session ownership lock.

## Static/dynamic costmap publication policy

The package keeps physical context, social context, and reactive safety on
separate topic paths:

```text
/map -> StaticLayer + InflationLayer -> /static_costmap/costmap
                                      -> /map_encoder/input

/map -> StaticLayer + InflationLayer -> /global_costmap/costmap

/map -> StaticLayer + InflationLayer -> /local_costmap/costmap (odom, rolling)

/livox/lidar_local -> safety guard preprocessing -> /nav2/safety_points
                                               -> Collision Monitor + Guard
/ped_tracking -> pedestrian figures -> /nav2/pedestrian_figures (RViz only)
/ped_traces + /ped_tracking -> current human traces -> /nav2/pedestrian_traces
```

All three costmaps contain only prior-map geometry and inflation. They do not
subscribe to LiDAR, scans, detections or tracks. Even newly placed stationary
objects are excluded. The local window remains 4 x 4 m in `odom`, resolution
0.05 m, and projects `/map` through TF. People are visualized separately and
the raw-cloud stop pipeline handles reactive stopping, not detour planning.
AMCL still consumes `/scan` without person removal. Traces are also RViz-only;
they do not feed the costmaps or the safety path.

### Pedestrian figures and traces

Nav2 starts dedicated `moai_nav_viz` figure and trace nodes, not perception.
`use_pedestrian_figures` and `use_pedestrian_traces` default to `false`.
Use the session flag `--pedestrian-viz` when perception visualization is needed. Inputs default to `tracks_topic:=/ped_tracking`
and `traces_topic:=/ped_traces`. RViz enables both Nav2 displays; legacy
detection boxes and bbox traces remain optional, disabled debug displays.

RViz already uses `map` as its Fixed Frame; the figure/trace nodes currently
publish in `odom`. Changing a frame label alone cannot fix physical alignment.
The [next validation plan](docs/Connected_Validation_2026-09-16.md) covers
measurement-time map output, the current frame-locked display behavior, sensor
calibration, and alignment during robot motion. Tracking remains in `odom`;
an explicit map-output visualization option is planned, not implemented here.

Edit [pedestrian_viz.yaml](config/pedestrian_viz.yaml), or supply an alternate
file with `pedestrian_viz_params_file:=/absolute/path/to/pedestrian_viz.yaml`.
Keep the `pedestrian_figures` and `pedestrian_traces` node keys in that file.
Parameters are startup-only; restart the affected visualization node after editing.

- `figure_scale: 0.5`: linear visual scale only. Head plus body height is
  0.85 m, body height 0.69 m, head radius 0.08 m, shoulder radius 0.125 m.
  XY position, ground contact, physical detection size and safety geometry do not change.
  `figure_scale: 1.0` restores the previous visual size.
- `label_height: 0.18`: independent readable text height in metres.
- Trace defaults: `history_seconds: 3.0`, `max_points: 100`,
  `line_width: 0.03`, `ground_z: 0.02` in `odom`.
- Only IDs currently present as people in fresh `/ped_tracking` are shown.
  A trace needs at least two valid history points. The int64 ID is placed in
  its marker namespace, and its color matches the figure.
- Each historical pose uses its own frame and measurement timestamp for TF.
  No latest-TF/embedded-odometry fallback or extra prediction is performed.
  Both input streams expire after 1 s using measurement and monotonic receipt
  ages. Empty/missing IDs, invalid history or unavailable TF remove the trace;
  10 Hz refresh and 0.3 s marker lifetime bound residual display time.
  ROS clock regression clears both inputs. FAST-LIVO2 resets are not reliably
  identifiable from trace messages alone: restart perception and visualization
  with a new odom session, and re-enter the AMCL initial pose.

The trace window is an **upper limit**, not a promise of three seconds of history.
The existing tracker retains 10 samples by default. Nav2 neither synthesizes
missing history nor changes tracker defaults. For a longer history, the separate
perception launch already offers `trace_window` (sample count) and
`trace_down_sample`; select them after measuring the actual update rate.
`launch_trace_markers:=false` disables the legacy bbox renderer, not `/ped_traces`.

`map_patch_node` samples `/static_costmap/costmap` through the current
`map -> base_link` transform and publishes `/map_encoder/input` as a
`nav_msgs/msg/OccupancyGrid`. Its default contract is:

- frame: `base_link`, with the robot at the grid center and forward along `+x`
- size: `10 m x 10 m`, `0.05 m/cell`, `200 x 200` cells
- rate: `10 Hz`
- values: standard OccupancyGrid values `-1..100`, including static inflation

The patch node can be omitted with `use_map_patch:=false`. This does not disable
the separate static costmap or the independent safety monitor.

## Recommended bringup procedure

Use the accepted September 18 localization baseline and the current
[goal-navigation procedure](docs/Goal_Navigation.md). Older diagnostic blockers
are historical findings unless reproduced in the current run.
The default startup policy deliberately does not inject a saved or hard-coded
AMCL pose. Every AMCL reset requires the operator to publish a fresh initial
pose with RViz **2D Pose Estimate**. Until that happens, messages about a
missing `map -> odom` transform are expected and navigation goals must not be
sent.
`amcl_quality_monitor` only publishes `/nav2/localization_diagnostics`; large
covariance prompts manual inspection, not automatic `/initialpose` injection.
Position and yaw variance thresholds are separate, and low covariance does not
prove pose accuracy. Disable it with `use_amcl_quality_monitor:=false` if needed.
The old `amcl_recovery_monitor.py` executable is a diagnostics-only compatibility
entry point, scheduled for removal on 2026-10-31 after caller migration. `use_amcl_recovery` has been retired; it cannot enable automatic resets.

### 1. Build and prepare the laptop

```bash
cd ~/moai_navigation_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to fast_livo mid360_bringup jackal_nav2_bringup
source ~/moai_navigation_ws/install/setup.bash
source ~/moai_navigation_ws/install/jackal_network_bringup/share/jackal_network_bringup/config/network_env.sh laptop
```

Verify system-clock synchronization **before starting ROS nodes**. Since
2026-09-11 the configured path is external NTP → laptop chrony (`192.168.50.1`)
→ NUC systemd-timesyncd. Operator-only setup and rollback instructions are in
the sibling package's `jackal_network_bringup/docs/time_sync.md`.
Do not run the old `init_time`/`ntpdate` procedure alongside this time service.

```bash
chronyc tracking
chronyc sources -v
ssh jackal 'timedatectl timesync-status'
```

The configured NUC server must be `192.168.50.1`, with increasing packet count
and no repeated root-distance rejection. `Normal`/`NTPSynchronized=yes` alone
is insufficient: inspect reference age and compare both clocks with a
round-trip-aware measurement. Initial pre-ROS targets are absolute NUC−laptop
offset ≤20 ms and remaining correction ≤5 ms over repeated samples. Check
chrony's `System time` separately; `adjtime` pending=0 does not imply that
chrony's correction is complete. These are measured startup criteria, not a
guarantee of long-term UTC accuracy or reboot/reconnection recovery.
Chrony's served NTP estimate can differ from its host's OS clock during slew;
use direct OS-to-OS comparison for ROS timestamp alignment, not NTP reply offset alone.

This stack uses wall clock (`use_sim_time:=false`); `/clock` cannot replace
machine synchronization. Request operator sudo assistance for system changes.
Do not restart chrony, step clocks or copy time manually during ROS operation.
Stop time-sensitive nodes before any necessary step and reinitialize afterward;
never hide clock mismatch by increasing TF tolerances.

### 2. Recommended: managed staged launch and separate perception

Use the [complete operating procedure](docs/Operations.md), including the first
package rebuild, network-owned temporary IP-reassembly setup, common environment and shutdown steps.
The September 18 test is the accepted localization baseline; additional network
tuning is deferred. Network tuning is runtime-only and owned by jackal_network_bringup (16MiB minimum, ipfrag_time=3).

With the robot services running, the laptop environment sourced and the kernel
prepared, keep the robot stationary and run in terminal A:

```bash
ros2 run jackal_nav2_bringup nav_session.py nav \
  --map "$HOME/moai_navigation_ws/src/jackal_nav2_bringup/maps/frontier_10F/frontier_10F.yaml"
```

The wrapper rejects duplicate stack processes, preserves logs under
`~/.ros/nav_sessions`, and runs `nav_bringup.launch.py` with motion disabled by default.
`--profile-dir` passes the prepared nav2/safety/operator files; `--enable-motion` is explicit.
Its `status` reports process ownership, not localization accuracy or message readiness.

This launch owns the local relay, FAST-LIVO2, Nav2 and RViz. Do not also run
`bringup.launch.py` or a second FAST-LIVO2 instance. The gates require actual
measurements, rather than publisher discovery:

1. Fresh cloud and IMU samples continuously for `ready_settle` seconds
   (default 5), before FAST-LIVO2 starts.
2. Fresh, valid FAST-LIVO2 odometry for the same interval before Nav2 starts.
   During this stationary startup, position must stay within 1 m of the odom
   origin, with no consecutive jumps above 2 m/s or 3 rad/s.
3. Fresh scan and adapted odometry, active AMCL/map_server, a received map,
   and `odom -> base_link` TF at the scan timestamp before the initial-pose
   prompt is printed. `map -> odom` is intentionally not required yet.

Every gate rejects samples older than 0.3 s or more than 0.05 s in the future;
receipt or timestamp gaps over 0.3 s restart the continuity window. Timeouts
are 60 s for sensor/FAST-LIVO2 stages and 90 s for localization inputs. These
are configurable with `input_timeout` and `localization_timeout`, independently of
`ready_settle`. After all processes start, `stack_stability.py` continuously checks
cloud/IMU/FAST odometry/scan/adapted odometry, TF, lifecycle and output ownership.
Its independent defaults are `stability_timeout=600` and `stability_settle=180`.
Faults and perception/lifecycle transitions restart continuity; this is not a pose-accuracy proof.

Wait for the stability state `INITIAL_POSE_REQUIRED`, then use RViz **2D Pose Estimate**
and check that the scan matches the correct map walls. Then open a separate
terminal, source the same ROS/workspace/network profile from step 1, and run:

```bash
ros2 run jackal_nav2_bringup nav_session.py perception --initial-pose-confirmed
```

This second session checks fresh scan/odom, active AMCL/map_server and scan-time
`map -> odom` before launching perception. The flag records the operator's visual
alignment check; TF presence alone cannot establish scan/map alignment.
Generated profiles select the local relay and `tracking_frame: odom`, and avoid
duplicate base/LiDAR and LiDAR/IMU static transforms. The upstream endpoint
connection check now allows 60 seconds for cold YOLO startup; it does not verify
inference or fresh detection messages. See Operations for the message-flow check.

The raw camera Image display is disabled in a temporary RViz profile by
default. Set `use_camera_image:=true` to retain the supplied profile's image
settings. The source RViz profile is not rewritten. This avoids that extra
raw-image subscription; it does not establish why earlier LiDAR input stopped.

If a critical process exits (including FAST, relay, Guard, operator stop, odom
adapter or stability monitor), the launch shuts down its remaining processes. Gate failure also stops startup. Managed perception stops when its
owning Nav2 session ends. Ctrl+C in terminal A stops managed perception first,
then Nav2. To stop both from another prepared terminal:

```bash
ros2 run jackal_nav2_bringup nav_session.py stop
sudo python3 "$(ros2 pkg prefix jackal_network_bringup)/lib/jackal_network_bringup/ipfrag_session.py" restore
```

Kernel restoration is explicit and refuses while stack processes remain. The
session tool never kills unowned processes or changes the kernel itself. Direct
launch processes must be stopped in their original terminals. After a failure,
inspect the input error, restart with a new initial pose, then restart perception.

The former external-FAST manual startup sequence has been retired. Use the
managed session above and the current [initial-pose/goal procedure](docs/Goal_Navigation.md).
Battery and speed displays remain independently configurable with
`use_battery_gauge` and `use_speed_display` (both default true).

## Launch overrides

Direct staged launch exposes additional topic/display settings and performs the same
network preflight and continuous stability checks:

```bash
ros2 launch jackal_nav2_bringup nav_bringup.launch.py \
  map:=/absolute/path/to/map.yaml enable_motion:=false \
  raw_lidar_topic:=/livox/lidar lidar_pointcloud_topic:=/livox/lidar_local \
  ready_settle:=5.0 input_timeout:=60.0 localization_timeout:=90.0 \
  stability_settle:=180.0 stability_timeout:=600.0 \
  use_map_patch:=true use_rviz:=false
```

The AMCL motion thresholds are 0.05 m translation or 0.05 rad rotation, with
120 scan beams. These make small-motion corrections more frequent; they do not
prove absolute accuracy. The default motion model is `nav2_amcl::OmniMotionModel`
for LIO pose increments that include small lateral motion during rotation.
This avoids DifferentialMotionModel's large bearing-noise jump at 1 cm of
translation. Alpha values remain 0.2 to isolate the model change; controller
lateral velocity limits remain zero. The [recorded-scan AMCL core comparison](docs/AMCL_Rotation_Patch_2026-09-17.md)
supports reduced particle spread; the September 18 manual rotation run is the
accepted development baseline.
A stationary filter does not normally update on every
scan; a single localization-only update can be requested for inspection:

```bash
ros2 service call /request_nomotion_update std_srvs/srv/Empty '{}'
```

This updates localization only and does not command robot motion. Repeatedly
forcing updates can make covariance look small without resolving an ambiguous
corridor; covariance is not ground-truth error.

## Command safety

The only launched final-output path is:

```text
Controller/behaviors -> Velocity Smoother -> Collision Monitor -> Safety Guard
  cmd_vel_nav         nav2_cmd_vel_unstamped  nav2/collision_checked_cmd_vel
                                                            |
                                    /j100_0519/nav2_cmd_vel (TwistStamped)
```

`enable_motion:=false` is the default and makes the Guard publish zeros. There
is no direct stamper bypass and no smoother after Collision Monitor. The
stamper is now an import-only message-copy helper with no publisher or executable node. This package never publishes directly to
`/j100_0519/cmd_vel` or changes the NUC's existing `forward_cmd_vel=false`.

The `Twist` to `TwistStamped` conversion is owned by this package; navigation
consumes read-only preflight results from `jackal_network_bringup`. Forwarding the dedicated Nav2 topic
to the actual platform command remains an external platform-integration
decision. Enable motion only after checking the TF tree, localization, costmap,
emergency stop, and a clear test area.

`config/nav2_safety.yaml` is the single source of polygon dimensions for both
the guard and monitor. Stop: x ±0.60 m/y ±0.50 m, slow: x ±1.00 m/y ±0.80 m;
three points trigger stop or 0.3 speed ratio. These are provisional test values,
not validated stopping distances. The guard independently checks the stop
rectangle, raw-cloud timestamp and monotonic receipt age (0.30 s), monitor
command age (0.25 s), mandatory dynamic TF health, finite commands, and clock
regressions at 20 Hz. It clamps output to the prepared profile, bounded by the observed NUC limits and 0.50 m/s / 1.0 rad/s.
Raw points are only transformed, finite-filtered, self-masked, and height-cropped
to 0.10–1.80 m. Empty **filtered** observations are valid; empty/malformed/all-NaN
raw observations are faults. No transform failure is replaced by identity.

```bash
ros2 topic echo /nav2/safety_diagnostics
ros2 lifecycle get /collision_monitor
```

The platform's independently configured 0.5 s watchdog and manual/E-stop
priority must be checked before attended low-speed motion. This is software
stopping, not a certified safety system. It cannot plan around a pedestrian.
See [implementation and acceptance checks](docs/Static_Safety_Figures_Validation.md).

## Verification

For a short, attended memory/network baseline, identify the actual FAST-LIVO2
PID (and optionally YOLO PID), then record without extra ROS subscriptions:

```bash
ros2 run jackal_nav2_bringup runtime_resource_audit.py \
  --pid <FAST_LIVO_PID> --duration 60 --output /tmp/nav2_resources_new.jsonl
```

Repeat `--pid` for additional processes. The output must be a new file. RSS,
anonymous/file/shared RSS, per-process swap, available RAM and UDP error deltas
are sampled with monotonic time; process exit/PID reuse stops the recording.
The tool does not kill processes, adjust network buffers, step clocks or assert
that a stable RSS sample establishes safety. Stop the launch manually if memory
continues growing. Do not run a long audit until the short baseline is stable.

After setting the initial pose, collect a read-only audit (exit 0 means the
observed ownership/timing/input checks passed, not that the robot is localized
in the correct corridor):

```bash
ros2 run jackal_nav2_bringup tf_localization_audit.py \
  --duration 12 --output /tmp/jackal_tf_audit.json
```

The audit resolves actual DDS writers to node names per TF edge, checks scan
timestamps against buffered TF, and reports scan endpoint distances to map
walls. It accepts AMCL's intentional future dating and old static TF stamps.
See [TF/localization validation notes](docs/TF_Localization_Validation.md) for
the measured findings, FAST-LIVO2 frame fix, and remaining validation limits.

For the next **connected** measurement, the evidence recorder now supports
a localization-only baseline without querying/subscribing to perception inputs:

```bash
ros2 run jackal_nav2_bringup record_navigation_validation.py \
  --localization-only --duration 603 --output-dir /tmp/nav2_baseline_next

# Separate run after starting perception and restoring the initial pose if needed.
ros2 run jackal_nav2_bringup record_navigation_validation.py \
  --duration 603 --output-dir /tmp/nav2_full_perception_next
```

Each output directory must be new. Default mode remains full perception.
`recording_status.json` records the mode, each command's progress/return code,
metadata errors and cancellation. A timeout, missing report, short capture or
failed parameter dump cannot produce `recorded`. `recorded_with_findings`
means the audit reported problems; `incomplete` and `interrupted` are not passes.
Even `recorded` is evidence acquisition status, not pilot approval or proof of
absolute pose accuracy. The three-second warmup and any restart-candidate phase
must be excluded when assessing a continuous ten-minute steady interval.
These commands were **not run** during the disconnected September 11 work.

```bash
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /scan
ros2 topic hz /livox/lidar
ros2 topic echo --once /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /static_costmap/static_costmap
ros2 param get /static_costmap/static_costmap plugins
ros2 param get /global_costmap/global_costmap plugins
ros2 param get /local_costmap/local_costmap plugins
ros2 topic echo --once /static_costmap/costmap --field info
ros2 topic echo --once /map_encoder/input --field info
```

All three plugin queries must report only `static_layer` and `inflation_layer`.
At fixed map and TF, a person crossing the LiDAR field must not change any of
their cells. Figure positions and safety output may change. With a moving
robot/AMCL correction the rolling map/patch can change because the sampling
pose changes, not because live points entered a costmap.

There must be exactly one global `/tf` publisher for each of
`odom -> base_link` and `map -> odom`. FAST-LIVO2 pose resets are treated as
odometry discontinuities; the adapter publishes zero twist for that sample and
starts a new differentiation baseline.

### Local ROS installation

This development PC previously had a partial ROS package update and failed with:

```text
libdiagnostic_updater.so: cannot open shared object file
```

The installation has since been aligned to
`ros-humble-diagnostic-updater 4.0.7-1jammy.20260724.032411`; `dpkg --verify`
and dynamic-library resolution now pass. If the error returns after another
partial upgrade, realign the package with:

```bash
sudo apt update
sudo apt install --only-upgrade ros-humble-diagnostic-updater
```

All Nav2 binary packages must also use the same release. In particular,
`ros-humble-nav2-msgs` must not remain on 1.1.18 while the Nav2 nodes are on
1.1.20. Realign it with:

```bash
sudo apt update
sudo apt install --only-upgrade ros-humble-nav2-msgs
```

The repository does not modify system packages automatically.

## Operator stop and current trial profile

Implementation and test results: [2026-09-21 patch report](docs/Navigation_Patch_2026-09-21.md).

See [goal navigation](docs/Goal_Navigation.md) for Circle stop, Triangle re-arm,
stationary button calibration, bridge-bounded profiles and heading-aligned map data.
The operator gate stops **Nav2 output**, not higher-priority manual/RC inputs.
Physical button-to-stop latency is not certified by unit or kinematic tests.
The default remains `enable_motion:=false`; unverified button mapping and footprint
prevent arming. `launch_operator_stop:=false` only permits an externally launched
supervisor; it does not disable the Guard's mandatory stop heartbeat.
