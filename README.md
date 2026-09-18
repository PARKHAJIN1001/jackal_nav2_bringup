# jackal_nav2_bringup

AMCL and Nav2 integration for a Jackal whose smooth local pose comes from an
external FAST-LIVO2 process.

Current baseline and agreed next steps: [2026-09-18 session handoff](docs/Session_Handoff_2026-09-18.md).
With the laptop IP reassembly limit temporarily set to 128MiB, Nav2 plus separate
perception maintained scan/map alignment during manual spot turns. The user
accepted current localization as the development baseline. Session tools now
organize startup/shutdown while preserving that configuration; their first
hardware restart is pending. Next: validate that workflow, then complete RViz
2D Goal Pose autonomous navigation. Additional localization/network tuning is
deferred. Autonomous driving has not yet been validated; motion remains disabled.
See the [measured results](docs/validation/2026-09-18/z_axis_test_1351/REPORT.md).

Recommended operation: [startup, initial pose, separate perception, shutdown and kernel restore](docs/Operations.md).
The session tools preserve the validated launch settings and keep motion disabled.
They are additive: direct launch remains available for advanced configurations.

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

This package starts `pointcloud_to_laserscan` and publishes `/scan` for AMCL by
default. The legacy `bringup.launch.py` expects an external FAST-LIVO2 process;
`nav_bringup.launch.py` starts FAST-LIVO2 after sensor readiness. Both expect
the MID-360 driver and Clearpath platform stack to be running on the robot.
Set `use_scan_projection:=false` only when another node already owns `/scan`.

For a networked MID-360, several laptop-side PointCloud2 subscribers can create
multiple large DDS streams over the robot link, depending on DDS transport and
multicast settings. The relay is enabled by default and provides one bounded
remote subscription; it does not bound downstream queues, enforce host-local
transport, or re-date/transform the cloud. FAST-LIVO2 and perception must be
configured separately to use its output; Nav2 cannot override external nodes:

```bash
# Start Nav2 first so the relay output exists.
ros2 launch jackal_nav2_bringup bringup.launch.py \
  map:=/absolute/path/to/map.yaml \
  use_composition:=true \
  use_lidar_relay:=true \
  raw_lidar_topic:=/livox/lidar \
  lidar_pointcloud_topic:=/livox/lidar_local enable_motion:=false

# Then start FAST-LIVO2 against the local relay output.
ros2 launch fast_livo mapping_mid360.launch.py \
  lidar_topic:=/livox/lidar_local image_enable:=false rviz:=false
```

Do not set the relay input and output to the same topic.
When `lidar_pointcloud_topic` is omitted, it resolves to `/livox/lidar_local`
with the relay enabled, or to `raw_lidar_topic` with it disabled. Explicit custom
outputs are preserved. Resolved input/output aliases are checked for relay loops.
`/nav2/lidar_relay_diagnostics` separates original measurement age from monotonic
receipt silence and reports stamp/clock regression counts. It is diagnostic only:
payload and stamp are unchanged, and the safety Guard remains responsible for stopping.

`bringup.launch.py` uses Nav2 component composition by default. Keeping the
Nav2 C++ nodes in one DDS participant materially reduces discovery churn while
the remote MID-360 stream is active. The standalone launch files retain
`use_composition:=false` as their default because they do not create a component
container by themselves.

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
`use_pedestrian_figures` and `use_pedestrian_traces` default to `true` and can
be disabled independently. Inputs default to `tracks_topic:=/ped_tracking`
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

Resolve the FAST-LIVO2 blockers in the latest review before using this procedure.
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
entry point. `use_amcl_recovery` has been retired; it cannot enable automatic resets.

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
package rebuild, temporary 128MiB setup, common environment and shutdown steps.
The September 18 test is the accepted localization baseline; additional network
tuning is deferred. The 128MiB setting is runtime-only, not installed persistently.

With the robot services running, the laptop environment sourced and the kernel
prepared, keep the robot stationary and run in terminal A:

```bash
ros2 run jackal_nav2_bringup nav_session.py nav \
  --map "$HOME/moai_navigation_ws/src/jackal_nav2_bringup/maps/frontier_10F/frontier_10F.yaml"
```

The wrapper rejects duplicate stack processes, preserves logs under
`~/.ros/nav_sessions`, and runs `nav_bringup.launch.py` with `enable_motion:=false`.
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
are startup checks, not a continuous localization-accuracy monitor.

When the terminal prints `AMCL/map_server active`, use RViz **2D Pose Estimate**
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

If FAST-LIVO2 or the relay exits, the staged launch shuts down its remaining
processes. Gate failure also stops startup. Managed perception stops when its
owning Nav2 session ends. Ctrl+C in terminal A stops managed perception first,
then Nav2. To stop both from another prepared terminal:

```bash
ros2 run jackal_nav2_bringup nav_session.py stop
sudo python3 "$(ros2 pkg prefix jackal_nav2_bringup)/lib/jackal_nav2_bringup/ipfrag_session.py" restore
```

Kernel restoration is explicit and refuses while stack processes remain. The
session tool never kills unowned processes or changes the kernel itself. Direct
launch processes must be stopped in their original terminals. After a failure,
inspect the input error, restart with a new initial pose, then restart perception.

### Alternative manual procedure: start Nav2, the local LiDAR relay, and RViz

The following legacy steps are for independently managed components. They do
not use the session ownership/ordered-stop wrapper; do not mix them with a
managed session. They also require the prepared 128MiB host setting.

With the Clearpath platform and MID-360 driver already running, use terminal A:

```bash
source /opt/ros/humble/setup.bash
source ~/moai_navigation_ws/install/setup.bash
source ~/moai_navigation_ws/install/jackal_network_bringup/share/jackal_network_bringup/config/network_env.sh laptop

ros2 launch jackal_nav2_bringup bringup.launch.py \
  map:=$HOME/moai_navigation_ws/src/jackal_nav2_bringup/maps/frontier_10F/frontier_10F.yaml \
  use_composition:=true \
  use_lidar_relay:=true \
  raw_lidar_topic:=/livox/lidar \
  lidar_pointcloud_topic:=/livox/lidar_local \
  use_rviz:=true \
  use_pedestrian_figures:=true \
  enable_motion:=false
```

Starting this launch first establishes the bounded local relay before
FAST-LIVO2 subscribes. The launch terminal prints an explicit reminder that an
initial pose is required.

The default RViz configuration also shows a **Battery (%)** circular gauge at
the upper left. `battery_percentage_bridge.py` subscribes to
`/j100_0519/platform/bms/state` (`sensor_msgs/msg/BatteryState`) and publishes
`percentage * 100` on `/nav2/battery_percentage` (`std_msgs/msg/Float32`).
The input uses sensor-data QoS to accept best-effort and reliable publishers;
the gauge uses a reliable subscription and a maximum of 100.
The required `rviz_2d_overlay_plugins` package is a runtime dependency
(`sudo apt install ros-humble-rviz-2d-overlay-plugins` on Humble).

Override the input with `battery_state_topic:=/your/battery_state` if needed.
`use_battery_gauge:=false` stops the bridge; uncheck **Battery (%)** in RViz to
hide the display. Custom `rviz_config` files need their own
`rviz_2d_overlay_plugins/PieChartOverlay` display on `/nav2/battery_percentage`
with `max value` set to 100. The bridge also works with `use_rviz:=false` for
a separately started RViz.

Only finite input percentages in [0, 1] are accepted. Unknown (`NaN`) or
out-of-range values are skipped with a warning, not presented as an empty or
full battery. The stock overlay retains its last reading when messages stop
or become invalid; it does not indicate freshness. Before the first message,
any initial zero shown by the overlay is not a measured battery value.
Check the source and converted values with:

```bash
ros2 topic echo /j100_0519/platform/bms/state --once
ros2 topic echo /nav2/battery_percentage --once
```

The default RViz also displays `Speed: 0.00 m/s` below the battery gauge.
`speed_overlay.py` computes planar speed as `hypot(vx, vy)` from the odometry
twist on `nav_odom_topic` (default `/odom`, the FAST-LIVO2 adapter's estimated
motion). Reverse motion remains positive; angular and vertical velocity do
not contribute. This is an odometry estimate, not a commanded speed.
Use `speed_odom_topic:=/your/odom` to select another `nav_msgs/msg/Odometry`
source, or `use_speed_display:=false` to disable the publisher.
The `rviz_2d_overlay_plugins/TextOverlay` display subscribes to
`/nav2/speed_overlay`, with message-controlled position and appearance.
Before odometry arrives it shows `waiting`; after two seconds without an
update it shows `stale` instead of retaining a current-speed reading.

### 3. Start FAST-LIVO2

After terminal A has created `/livox/lidar_local`, use terminal B:

```bash
source /opt/ros/humble/setup.bash
source ~/moai_navigation_ws/install/setup.bash
source ~/moai_navigation_ws/install/jackal_network_bringup/share/jackal_network_bringup/config/network_env.sh laptop

ros2 launch fast_livo mapping_mid360.launch.py \
  lidar_topic:=/livox/lidar_local \
  image_enable:=false \
  rviz:=false \
  publish_sensor_static_tf:=false \
  publish_lidar_to_imu_tf:=true
```

Wait until the occupancy map is visible and `/scan` and `/odom` are arriving
(use `ros2 topic hz /scan` and `ros2 topic hz /odom`). With RViz fixed to `map`,
the scan cannot appear until the next step creates `map -> odom`.

### 4. Set the Jackal initial pose in RViz

1. Select **2D Pose Estimate** in the RViz toolbar.
2. Click the Jackal's approximate physical position on the map.
3. Keep the mouse button pressed and drag the arrow in the Jackal's forward
   direction, then release it.
4. Check that **AMCL Pose** appears and that the MID-360 scan overlaps the map
   walls. Re-enter the pose if it aligns with the wrong repeated corridor.

The included RViz configuration publishes the estimate to `/initialpose` with
standard deviations of approximately `0.5 m` in x/y and `15 degrees` in yaw.
For the long, repeated corridors in `frontier_10F`, identify the correct
corridor section and start within roughly `0.5--1.0 m` and `10--20 degrees`
when possible. Along-corridor position is generally less observable than
distance and angle relative to the side walls.

Confirm the resulting global transform before setting a goal:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo --once /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped --qos-durability transient_local
```

Only after the scan-map alignment is plausible should **2D Goal Pose** be
used. A successful transform alone does not prove that AMCL selected the
correct repeated corridor.

### 5. Start perception separately (terminal C)

Nav2 never launches perception. Source the same ROS/workspace/network profile
in terminal C. Current upstream no longer exposes the old `lidar_input_topic`
and `tracking_frame` convenience arguments; do not pass arguments that it ignores.
Generate profiles from the installed upstream YAML instead:

```bash
ros2 run jackal_nav2_bringup prepare_perception_config.py
```

Run the separate `ros2 launch mid360_bringup perception.launch.py ...` command
printed by that utility after the upstream blockers are fixed. The tool starts
no nodes and preserves installed tuning, changing only the accumulator input
to `/livox/lidar_local`, extractor `tracking_frame` to `odom`, and
`use_latest_tf_fallback` to `false`. It records source hashes and refuses to
overwrite an existing output directory. Use `--lidar-topic /custom/local` for a
custom relay. Temporary profiles must be regenerated after upstream changes.

The NUC owns `base_link -> livox_frame`; FAST-LIVO2 owns
`livox_frame -> livox_imu_frame`. Perception supplies only the calibrated
`livox_frame -> camera_link` bridge; RealSense retains its optical child tree.
Standalone perception defaults are unchanged when overrides are omitted.
The generated `tracking_frame: odom` changes the extractor output/tracker input
coordinates; it does not change the tracking algorithm.
Do not use `full_stack.launch.py` as a shortcut here: its current default does
not start FAST-LIVO2, and opting in also enables an identity-odometry fallback
unless explicitly disabled. A fabricated identity pose is not valid odometry
for Nav2. Separate FAST-LIVO2 and perception remain the supported procedure.

### 6. Stop or restart the stack

Stop Nav2/RViz with `Ctrl-C` in terminal A first so that navigation command
generation ends while odometry is still available. Then stop perception in C
and FAST-LIVO2 in B. A FAST-LIVO2 restart resets the odom origin: **restart
perception too, then re-enter AMCL's 2D Pose Estimate**. Never carry old tracks
or an old initial pose across that reset. Verify no test processes remain before
restarting:

```bash
pgrep -af '[j]ackal_nav2_bringup|[f]astlivo_mapping|[r]viz2|[c]omponent_container_isolated'
```

## Launch overrides

Useful overrides are:

```bash
ros2 launch jackal_nav2_bringup bringup.launch.py \
  map:=/absolute/path/to/map.yaml \
  fast_livo_odom_topic:=/aft_mapped_to_init \
  nav_odom_topic:=/odom \
  nav_cmd_vel_topic:=/j100_0519/nav2_cmd_vel \
  use_map_patch:=true \
  use_scan_projection:=true \
  use_lidar_relay:=false \
  raw_lidar_topic:=/livox/lidar \
  lidar_pointcloud_topic:=/livox/lidar \
  scan_topic:=/scan \
  use_composition:=true \
  enable_motion:=false \
  use_pedestrian_figures:=true tracks_topic:=/ped_tracking \
  use_pedestrian_traces:=true traces_topic:=/ped_traces \
  use_rviz:=false
```

The AMCL motion thresholds are 0.05 m translation or 0.05 rad rotation, with
120 scan beams. These make small-motion corrections more frequent; they do not
prove absolute accuracy. The default motion model is `nav2_amcl::OmniMotionModel`
for LIO pose increments that include small lateral motion during rotation.
This avoids DifferentialMotionModel's large bearing-noise jump at 1 cm of
translation. Alpha values remain 0.2 to isolate the model change; controller
lateral velocity limits remain zero. The [recorded-scan AMCL core comparison](docs/AMCL_Rotation_Patch_2026-09-17.md)
supports reduced particle spread, with live rotation validation still pending.
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
standalone stamper remains only as a compatibility utility/helper; do not run
it alongside the guard. This package never publishes directly to
`/j100_0519/cmd_vel` or changes the NUC's existing `forward_cmd_vel=false`.

The `Twist` to `TwistStamped` conversion is owned by this package; navigation
does not depend on `jackal_network_bringup`. Forwarding the dedicated Nav2 topic
to the actual platform command remains an external platform-integration
decision. Enable motion only after checking the TF tree, localization, costmap,
emergency stop, and a clear test area.

`config/nav2_safety.yaml` is the single source of polygon dimensions for both
the guard and monitor. Stop: x ±0.60 m/y ±0.50 m, slow: x ±1.00 m/y ±0.80 m;
three points trigger stop or 0.3 speed ratio. These are provisional test values,
not validated stopping distances. The guard independently checks the stop
rectangle, raw-cloud timestamp and monotonic receipt age (0.30 s), monitor
command age (0.25 s), mandatory dynamic TF health, finite commands, and clock
regressions at 20 Hz. It clamps output to 0.20 m/s and 0.35 rad/s.
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
