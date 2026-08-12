# autonomy_ws

Formula Student autonomy workspace. Holds our own code only — the eufs_sim2
simulator lives in its own workspace (`~/eufs`) and is **never** a build
dependency of anything here. The two workspaces meet only over ROS topics at
runtime, plus one pure message package (`eufs_msgs`).

```
~/eufs/          sim workspace   (eufs_sim2, vehicle_models, ... - x86 dev VM only)
~/autonomy_ws/   this workspace  (fs_autonomy - deploys to the Jetson)
```

That boundary is the containment strategy: the sim can be updated, broken, or
absent (as it will be on the car) without touching this workspace, and this
workspace can be copied to an arm64 target and built there unchanged.

## Package: fs_autonomy

| Node | Does | Depends on |
|---|---|---|
| `accel_driver` | ACCELERATION mission: closed-loop speed control, brakes past 75 m | std/nav/ackermann msgs only |
| `skidpad_driver` | SKIDPAD mission: odom-based figure-8 (pure pursuit), geometry from the rules/map | std/nav/ackermann msgs only |
| `trackdrive_driver` | TRACK_DRIVE + AUTOCROSS: steers from `/cones` (nearest blue/yellow pair midpoint), laps counted from odom | `eufs_msgs` |
| `cone_viz` | republishes `ConeWithColorProbabilityArray` cone topics as MarkerArrays for Foxglove/RViz | `eufs_msgs` |
| `sil_accel` / `sil_skidpad` / `sil_trackdrive` | closed-loop test harnesses replacing the sim (1D / bicycle / bicycle + FoV-limited cone sensor) | std/nav/ackermann + `eufs_msgs` |

`trackdrive_driver` is the perception-socket proof: it never reads the map or
uses odometry for geometry -- steering comes entirely from car-relative cone
detections, so swapping the sim's cone_fusion for a real perception stack is
a pure substitution. Autocross runs the same executable with `laps: 1` and
its own mission gate (see `config/autocross_params.yaml`).

Upstream bug (eufs_sim2): the fused `/cones` topic is advertised but never
published -- `perception_cones_pub_` in `cone_fusion.cpp` is created and then
never used. The FoV-filtered car-relative feed actually flows on
`/cones/lenient`, so the trackdrive/autocross configs point `cones_topic`
there. (Candidate upstream MR: publish the fused cones, ideally with noise.)

Hard-won detail, do not regress it: the sim reads **only**
`drive.acceleration` and `drive.steering_angle` from `/cmd`
(`eufs_sim2/src/control.cpp`). `drive.speed` is silently ignored, so speed
must be regulated by the controller (P loop on `/odom` speed error → an
acceleration command). Negative acceleration brakes; the vehicle model zeroes
it once stopped, so a held brake command cannot reverse the car.

## Build (dev VM)

```bash
source /opt/ros/humble/setup.bash
source ~/eufs/install/setup.bash        # underlay: provides eufs_msgs
cd ~/autonomy_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means edits to the Python sources take effect without
rebuilding.

## Run (against the sim)

```bash
eufs sim run                              # terminal 1
ros2 launch fs_autonomy autonomy.launch.py # terminal 2, AFTER the sim is up
```

`autonomy.launch.py` starts foxglove_bridge, cone_viz, and every mission
controller. Controllers idle until their own mission is selected: each one
gates on its `AMIState:` in `/sim/ros_can/state_str`, so the Foxglove mission
dropdown decides which controller engages at DRIVING. Restart terminal 2
whenever the sim restarts (the bridge snapshots topics/services at startup).

Then in Foxglove: Set Mission → ACCELERATION, then GO. Parameters live in
`src/fs_autonomy/config/accel_params.yaml` (`mission:` included).

`accel.launch.py` still exists to run the accel controller alone. Quick
no-sim smoke tests of the control loops (work on the Jetson too):

```bash
export ROS_DOMAIN_ID=42             # in BOTH terminals -- see below

ros2 run fs_autonomy accel_driver   # terminal 1
ros2 run fs_autonomy sil_accel       # terminal 2 -- RESULT line after 40 s

ros2 run fs_autonomy skidpad_driver # terminal 1
ros2 run fs_autonomy sil_skidpad   # terminal 2 -- RESULT line after 100 s

ros2 run fs_autonomy trackdrive_driver --ros-args -p laps:=3   # terminal 1
ros2 run fs_autonomy sil_trackdrive                           # terminal 2 -- RESULT when stopped (~95 s)
```

**Always run harness tests in their own `ROS_DOMAIN_ID`.** DDS is a shared
bus: if the real sim is running anywhere on the machine, its /odom and state
topics interleave with the harness's and the driver under test receives two
contradictory realities. The symptom is the driver relatching "DRIVING -- go"
over and over. Isolating the domain makes collisions impossible.

## Deploy to a Jetson

Target assumption: JetPack 6 (Ubuntu 22.04) → ROS 2 Humble, same as the VM.
Two options, both isolated from the sim:

### Option A — native build

```bash
# on the Jetson
sudo apt install ros-humble-ros-base ros-humble-ackermann-msgs python3-colcon-common-extensions
mkdir -p ~/autonomy_ws/src && cd ~/autonomy_ws/src
git clone https://gitlab.com/eufs/eufs_msgs.git          # pure msgs, builds anywhere
# copy or clone this repo's src/fs_autonomy here
cd ~/autonomy_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build
```

Note the difference from the VM: on the Jetson `eufs_msgs` is cloned into
*this* workspace's `src/` (there is no sim workspace to underlay). Never clone
it in both places on the same machine — pick underlay (VM) or in-tree
(Jetson), or the overlay ordering gets confusing.

### Option B — Docker (strongest containment)

`docker/Dockerfile` builds from `ros:humble-ros-base`, which is multi-arch —
the same file produces an x86_64 image on the VM and an arm64 image on the
Jetson. Nothing from the host except the ROS network can affect it.

```bash
cd ~/autonomy_ws
docker build -t fs_autonomy -f docker/Dockerfile .
docker run --rm -it --net=host --ipc=host fs_autonomy \
    ros2 launch fs_autonomy accel.launch.py
```

`--net=host --ipc=host` lets DDS discover nodes on the host / LAN. If
discovery misbehaves across machines, set the same `ROS_DOMAIN_ID` everywhere
first — it defaults to 0.

### Troubleshooting: "Set Map dropdown is empty / get_map not advertised"

Run `python3 scripts/check_bridge.py` (dev VM tool; `pip install --user
websockets` once). It connects to the bridge exactly like Foxglove Studio
does and tells you which side is broken:

- **cannot connect** -> bridge isn't up yet (stack not launched, or
  wait_for_sim still waiting for the sim).
- **get_map NOT advertised** -> stale bridge snapshot: restart the stack
  launch with the sim running.
- **get_map advertised** -> server side is fine; the stale state is inside
  Foxglove Studio. Hit View -> Reload (Ctrl+R) in Foxglove -- panels like
  Set Map call get_map once when they mount and never re-query after a
  reconnect, so they must be re-mounted. Reload re-mounts all panels and
  keeps the layout.

The client-side case happens after every stack restart that Foxglove lives
through: the bridge goes away (and stays down while wait_for_sim holds it
back), the panel's one-shot query is left holding a dead result, and
reconnecting alone doesn't re-run it. Rule of thumb: after restarting the
stack, wait for "sim is up -- starting foxglove_bridge" in the launch
output, then Ctrl+R in Foxglove.

### What to keep out of this workspace

- Anything that imports from `eufs_sim2`, `vehicle_models`, `state_lib`,
  `map_lib` — sim internals, not available on the car.
- Absolute paths into `~/eufs`.
- New message dependencies that only the sim publishes, unless they are pure
  `_msgs` packages that can be cloned onto the target (like `eufs_msgs`).

## Monte-Carlo sweeps (perception robustness)

`sil_trackdrive` has a parameterized sensor error model; `scripts/run_sweeps.py`
batch-runs seeded episodes against `trackdrive_driver` and writes one CSV row
per episode. Always from a sourced shell:

```bash
cd ~/autonomy_ws
source /opt/ros/humble/setup.bash
source ~/eufs/install/setup.bash
source install/setup.bash

# sanity check: one clean episode, ~35 s
python3 scripts/run_sweeps.py --episodes 1 --speed 3

# a real sweep: 5 points x 20 seeded episodes (~1 h at 3x realtime)
python3 scripts/run_sweeps.py --episodes 20 --speed 3 \
    --sweep p_detect_scale=1.0,0.8,0.6,0.4,0.2 --out dropout.csv

# turn the CSV into the report graph (success rate + path deviation)
python3 scripts/plot_sweeps.py dropout.csv --x p_detect_scale
```

All three missions sweep via `--mission accel|skidpad|trackdrive` (each pairs
its driver with its SIL harness and applies its own pass criteria):

```bash
# the state-estimation experiment: skidpad is odom-geometry, watch it degrade
python3 scripts/run_sweeps.py --mission skidpad --episodes 20 --speed 3 \
    --sweep odom_yaw_drift_deg_per_sqrt_min=0,1,2,4,8 --out skid_drift.csv
```

Sweepable error-model knobs (harness params). Perception (sil_trackdrive
only): `p_detect_scale`, `bearing_noise_deg`, `range_noise_frac`,
`color_flip_prob`, `false_positives_per_frame`, `latency_frames`,
`sensor_range`, `sensor_fov_deg`. Odometry (all harnesses — the harness
scores against truth but publishes a corrupted estimate): `odom_pos_noise`,
`odom_yaw_noise_deg`, `odom_vel_noise`, `odom_drift_m_per_sqrt_min`,
`odom_yaw_drift_deg_per_sqrt_min`. Driver params (e.g. `target_speed`,
`laps`) can be swept the same way. Repeat `--sweep` for a grid; `--fixed
k=v` pins a value.

## State estimation (EKF)

`estimation.launch.py` runs a dead-reckoning robot_localization EKF against
the sim's raw sensors (needs `apt install ros-humble-robot-localization`):
`wheel_odometry` converts `/ros_can/wheel_speeds` (rev/s in the sim despite
the RPM comment -- upstream quirk) into a covariance-stamped twist,
`imu_frontend` fills the covariance fields the sim leaves zeroed, and the
EKF fuses them into `/odometry/filtered`. Point any controller at it with
`-p odom_topic:=/odometry/filtered` to drive on estimated state instead of
ground truth, or flip the whole stack at once with the launch argument
`realistic:=true` on the main launch. Measure the estimator against truth while a mission runs with
`scripts/extract_odom_profile.py` -- its `knobs` output block calibrates the
harness odometry knobs above from the real filter's behaviour.

Do NOT fuse `/ros_can/twist`: its angular axes are scrambled upstream
(x=pitch, y=yaw, z=roll -- see twist_publisher.cpp) and its covariance is
hard-zeroed. MR candidate.

With a measured detector profile (from `scripts/extract_error_profile.py`,
run wherever the model + dataset live):

```bash
python3 scripts/run_sweeps.py --episodes 20 --speed 3 \
    --profile perception/error_profile.json \
    --sweep p_detect_scale=1.0,0.8,0.6,0.4 --out yolo.csv
```

Notes: episodes are sequential on ROS_DOMAIN_ID 60, so a running sim doesn't
interfere -- but close it anyway to keep CPU free; `--speed 3` is validated
against realtime on this VM, don't push it higher here. An episode PASSES
when laps >= `--laps` (default 3), `cones_hit == 0`, and the car ends stopped.
