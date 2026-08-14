# PROJECT HANDOFF — FS Driverless Sim-First Platform (v2, 2026-08-12)

Final-year project (K.J. Somaiya School of Engineering, Sem 7). Team: Rishi
Satish, Harshvardhan Singh, Ayon Majumdar. Guide: Dr. Makarand Govind Kulkarni.

**How to use this doc:** paste it (plus `README.md`) into any AI session as
opening context, or read it yourself to resume work. It records strategy,
results, hard-won facts, and future ideas — everything NOT obvious from the
code. Repo: `github.com/red-lance/FSDV-Sim` (private). Dev machine: Ubuntu
22.04 VirtualBox VM, ROS 2 Humble, no GPU; sim workspace `~/eufs` (eufs_sim2,
cloned from gitlab.com/eufs), our workspace `~/autonomy_ws` (this repo).

---

## 1. The thesis

Not "four controllers that drive a sim car." A **development platform** for
Formula Student driverless: *develop* against a simulator, *verify*
automatically and cheaply, *deploy the identical software* to the car's
computer (Jetson) unchanged. **Develop → Verify → Deploy.**

- Controllers = proof of use. SIL harnesses = proof of correctness.
- Containment rules + multi-arch Docker = proof of portability.
- Interface contracts (two layers: raw-sensor ↔ object) = the product.

One-liner: *"Simulators, stacks, and datasets all exist online — what doesn't
exist is a verified path between them: documented interfaces, repeatable
closed-loop testing, and identical-code deployment to embedded hardware. That
path is the platform."*

Scope fence (state it, don't apologize): no perception training in-loop, no
SLAM, no learned control — the platform defines their interfaces and DERIVES
THEIR REQUIREMENTS (sweep curves read backwards); implementing them is Sem 8.

## 2. What exists (repo map)

Nodes (`src/fs_autonomy/fs_autonomy/`):
- `accel_driver`, `skidpad_driver`, `trackdrive_driver` (also runs AUTOCROSS
  via `laps:=1` config) — mission controllers, gate on `AMIState` + `ASState:
  DRIVING` in `/sim/ros_can/state_str`; all take `odom_topic` (default
  `/odom`, set `/odometry/filtered` for EKF).
- `sil_accel` / `sil_skidpad` / `sil_trackdrive` — SIL harnesses (1D /
  kinematic bicycle / bicycle + FoV cone sensor). Parameterized error models;
  print one `RESULT k=v...` line and exit. sil_trackdrive: perception error
  model (P(detect|range) from profile JSON, bearing/range noise, color flips,
  false positives, latency frames). All three: odometry corruption via
  `odom_corruptor.py` (white noise + seeded random-walk drift; scores vs
  TRUTH, publishes the corrupted estimate).
- `wheel_odometry` (wheel speeds → covariance-stamped twist), `imu_frontend`
  (fills IMU covariances the sim zeroes) — EKF input adapters.
- `cone_viz` — cone arrays → MarkerArrays for Foxglove.

Launches: `autonomy.launch.py` (everything; includes EKF stack when
robot_localization installed; args: `estimation:=false`, `realistic:=true` =
all controllers on `/odometry/filtered`), `estimation.launch.py` (EKF stack
alone), `accel.launch.py` (legacy single).

Scripts (`scripts/`):
- `run_sweeps.py` — Monte-Carlo runner: `--mission accel|skidpad|trackdrive`,
  `--sweep k=v1,v2` (grid via repeats), `--fixed`, `--profile <json>`,
  `--episodes N`, `--speed 3` (3x realtime, validated on this VM; scales
  harness realtime_factor + driver tick/publish rate). CSV out, per-mission
  pass criteria.
- `plot_sweeps.py` — success-rate + deviation curves from sweep CSVs.
- `extract_error_profile.py` — offline YOLO eval on labeled data →
  `error_profile.json` (P(detect|range) via box-height range proxy, color
  confusion, bearing/range err, FP/frame, latency). Run WHERE THE DATASET IS.
- `extract_odom_profile.py` — EKF vs ground truth while a mission runs →
  calibrated values for the odom knobs.
- `measure_skidpad.py` — TRUE-geometry skidpad circle error in the full sim
  (same metric as sil_skidpad; used for the prediction test).
- `check_bridge.py` — Foxglove bridge diagnostics.

Perception (`perception/`): YOLOv5n ("yolov5nu", ultralytics-loadable)
trained on FSOCO AMZ subset, 50 epochs: P 0.90 / R 0.64 / mAP50 0.736,
~29 ms/frame CPU. Weights + full training artifacts + FSOCO converter
(75/15/10 train/val/test split — measure on TEST). README's 55.9% table is
from an older run; results.csv is authoritative.

Docker: `docker/Dockerfile`, multi-arch (x86 VM / arm64 Jetson).

## 3. Verified results (quote these)

| What | Result |
|---|---|
| accel, SIL + full sim | holds 8 m/s, brakes at 75 m, stops ~84 m (= v²/2a physics) |
| skidpad, SIL | 5 cm max radial err, exact 8π, clean stop |
| skidpad, FULL SIM ground truth | 0.204/0.207 m mean/max circle err (first full-sim validation; gap vs SIL = vehicle-dynamics offset) |
| trackdrive, SIL | 3/3 laps, 0.10 m dev_max, 0 cones hit |
| EKF measured (83 m accel run) | 2.0 m drift = 2.4% of distance; yaw 2.5°; vel σ 0.017 m/s → knobs: drift 2.43 m/√min, yaw 3.08°/√min |
| accel ON EKF, full sim | driver belief 82.9 m vs true stop 82.88 m (2 cm) — while EKF ABSOLUTE pose >80 m off. Relative survives, absolute doesn't. |
| skidpad ON EKF, full sim | 0.33/0.69 m mean/max — mission completes, inside 1.5 m half-lane |
| **Prediction test** | SIL curve + dynamics offset predicted mean 0.32, measured 0.33 (±0.01!). Max underpredicted 2× — real drift is a bias RAMP, SIL knob is white random walk. Fidelity confirmed on mean; discrepancy explained + fix identified. |
| Perception dropout finding | uniform dropout nearly harmless (passes at 5% detection — frames integrate); binding constraints are range-concentrated misses, color flips, FPs, latency |
| Drift asymmetry finding | trackdrive steering immune to odom drift (perception-anchored) but lap counting breaks (phantom laps); skidpad error grows monotonically: 0.054/0.199/0.380/0.558 m max at 0/2/4/6 °/√min |

Bugs found by the platform (the thesis working):
1. UPSTREAM: `/cmd` consumes ONLY drive.acceleration+steering_angle;
   drive.speed silently ignored (control.cpp).
2. UPSTREAM: fused `/cones` advertised, never published (cone_fusion.cpp);
   real feed = `/cones/lenient`.
3. UPSTREAM: `/ros_can/twist` angular axes scrambled (x=pitch, y=yaw, z=roll)
   + covariance hard-zeroed (twist_publisher.cpp). NEVER fuse it.
4. UPSTREAM (minor): wheel speeds are rev/s in sim, msg comment says RPM;
   IMU msg covariance fields never filled from configured matrix.
5. OURS (found by first sweep): trackdrive blind-stop clocked from last cone
   MESSAGE, but a live detector seeing nothing still publishes empty arrays →
   blind car crept at min_speed forever. Fixed: clocked from last usable
   TARGET; blind car refuses to move.
6. OURS (found by prediction test): skidpad metric scored the drifted EXIT
   leg against the circles (16.9 m phantom). Fixed: x-span gate, both meters.

## 4. Hard-won facts / gotchas

- Sim local edit: `~/eufs/.../eufs_sim2/config/plugin_params.yaml` IMU noise
  raised to BMI088-class (var 1e-4 gyro / 2.5e-3 accel) + yaw-rate drift ramp
  1e-5 rad/s². Install is symlinked → edit is live; keep imu_frontend params
  in sync.
- ALWAYS run SIL tests under isolated `ROS_DOMAIN_ID` (run_sweeps uses 60).
- Headless mission control (no Foxglove): `/set_mission` (int16:
  ACCELERATION=1, SKIDPAD=2, AUTOCROSS=3, TRACK_DRIVE=4), `/go`, `/reset`,
  `/ebs` (all std_srvs/Trigger except set_mission).
- Mission strings have underscores: `TRACK_DRIVE`, `AUTOCROSS`.
- The sim renders NO images / point clouds — its "camera"/"lidar" are
  geometric FoV filters over ground-truth cones (output of perception, not
  input). In-loop images only via old Gazebo eufs_sim (domain gap for camera,
  small for lidar).
- EKF restarts: dead reckoning can't see `/reset` teleports — restart the
  estimation stack (or whole launch) whenever the sim resets.
- Foxglove: after any stack restart, wait for "sim is up" then Ctrl+R
  (panels one-shot their service queries).
- The vehicle: ads-dv params — wheelbase 1.53 m, steer ±0.37 rad, tyre
  radius 0.2525 m.

## 5. FUTURE IDEAS (compiled)

### 5.1 Near-term roadmap (Sem 7, roughly in order)
1. **Run the YOLO error-profile extraction** (STILL PENDING — needs the
   machine with `perception/data/processed`):
   `python scripts/extract_error_profile.py --weights
   perception/runs/detect/runs/yolov5_fsoco/weights/best.pt --data
   perception/data/processed --split test` → push `error_profile.json`.
   Then the headline sweep: `run_sweeps.py --profile ... --sweep
   p_detect_scale=1.0,0.8,0.6,0.4 --episodes 20`.
2. **Bias-ramp drift mode in odom_corruptor** — the prediction test showed
   real EKF drift is time-correlated, not white; add
   `odom_yaw_bias_ramp_deg_per_s2` and re-derive the skidpad curve. Closes
   the one open fidelity gap.
3. **Big-orange lap counter for trackdrive** — lap counting is its only odom
   dependency and breaks under drift (phantom laps). Count start/finish
   passes from big_orange cone-pair detections instead → fully
   perception-anchored controller → unblocks realistic-by-default.
4. **EKF Q-tuning grid search** — scripted: sweep Q diagonals, score RMSE vs
   ground truth per run, keep the best ("the sim auto-tunes our estimator").
   Also try fusing the OSS optical speed sensor (publishes with covariance).
5. **Full Monte-Carlo campaign** for the report graphs: perception knobs
   (color_flip_prob, false_positives_per_frame, latency_frames — dropout is
   known-boring), odom knobs (calibrated around measured EKF values),
   `--episodes 20`. Read curves backwards → requirements spec chapter.
6. **CI regression suite** — run_sweeps already knows all 3 missions + pass
   criteria; wrap `--episodes 1 --speed 3` per mission in `run_tests.sh`
   with exit codes, run in the Docker image on GitHub Actions per commit.
7. **Interface contract doc** — Layer 1 (raw: /imu/data, /ros_can/
   wheel_speeds, GNSS /ros_can/fix, OSS) ↔ Layer 2 (object: /cones/lenient
   semantics) + control contract (/cmd: accel+steer ONLY) + authority
   contract (state machine, RES, /go, /ebs). Document bugs 1–4 here. Add
   explicit QoS choices (sensor BEST_EFFORT etc.) — defaults WILL bite the
   cross-machine Jetson demo.
8. **Jetson**: procure EARLY (only external dependency). arm64 Docker build,
   sim on laptop ↔ stack on Jetson (same domain), benchmarks: odom→cmd
   latency, control-rate stability, CPU/RAM. Killer demo: same seeds, same
   RESULT lines on x86 and arm64 (SIL runs work on the Jetson, no sim
   needed). YOLO TensorRT FPS benchmark (no camera needed — FSOCO images).
9. **Upstream MRs** (gitlab.com/eufs): bugs 1–4 above + optional Foxglove
   cone-Marker converter. Credibility + report material.
10. **Realistic-by-default**: after items 2–4, re-qualify all four missions
    on the EKF repeatedly (vary sim `noise_seed`), then flip
    `realistic` default to true; keep `realistic:=false` as debug hatch.

### 5.2 Foolproofing / realism backlog (from the "make it bulletproof" list)
- **Fault injection**: EBS mid-run (assert /cmd silent within a tick), cone
  stream DIES entirely (silence ≠ empty arrays — different code path!), odom
  freeze at stale value. Likely finds: no stale-odom watchdog.
- **Stale-data watchdogs** in controllers: no fresh odom in 200 ms → brake.
- **Correlated/burst dropout** in the perception model (two-state Markov
  blind/good gate) — real detectors fail in bursts (glare, occlusion), and
  white dropout is known to flatter the controller.
- **More track layouts** for sil_trackdrive (hairpin = FoV starvation,
  chicane) — 3 tracks × N seeds ≫ 1 track × N seeds.
- **SIL-vs-full-sim cross-validation table** for trackdrive too (skidpad
  done via prediction test).
- **FS rulebook traceability**: cite FSG DV rule numbers for state machine /
  RES / EBS / 75 m claims in the contract doc.
- **Allan variance when a real IMU arrives**: log stationary hours →
  noise density + bias instability → into imu_frontend AND sim imu_plugin
  covariance → re-measure EKF → re-calibrate knobs. The sim then carries the
  real part's fingerprint before the car exists.

### 5.3 Perception track (Phases; 0+1 mostly done)
- Phase 1 remainder: extraction run (item 5.1.1). Range axis is approximate
  unless focal length known (FSOCO = mixed cameras; state the caveat).
- **Phase 2 (Sem 8): swap-in node** — camera images → YOLO → cone array on
  the contract topic (box-height → range via known cone height; depth camera
  if available). Image sources by effort: FSOCO-as-rosbag soak test
  (throughput/latency of the REAL node; no driving), live webcam + printed/
  miniature cones (demo — miniature cones are geometrically equivalent if
  height parameter set), Gazebo eufs_sim ZED (true closed loop; state the
  domain gap). Position-accuracy ground truth: phone + tripod + measuring
  tape + a few cones, one afternoon — FSOCO can never provide this (no 3D
  labels).
- **LiDAR module (Sem 8, if hardware or via Gazebo Velodyne)**: classical
  ground-removal + clustering (CPU-only, Open3D); measure P(detect|range)
  cliff, position σ, FP/scan; inject; the colour-less feed exposes the
  camera-lidar FUSION requirement — and the fusion plugin is exactly where
  upstream bug 2 lives. Fix it, then sweep camera+lidar profiles jointly.
- **Sem 8 candidates**: own bicycle-model EKF (vs the generic
  robot_localization motion model — derivation = report depth), cone-based
  localization (fixes drift properly; the drift-asymmetry data is its
  motivation), camera-lidar fusion study.

## 6. Differentiation vs what's online (viva ammunition)

Simulators (eufs_sim2, FSDS, CARLA) = a car in a world, no methodology/
verification/deployment. Team stacks (AMZ fsd_skeleton) = ROS 1, dead,
monolithic, one car's artifact. FSOCO = data, no system. Autoware =
oversized. None provide: the develop→verify→deploy pipeline, interfaces as
the documented product, verification cheap enough for thousands of closed-
loop runs, requirements specs DERIVED from sweeps, measured sim-fidelity
(prediction test). If pushed "just glue?": yes, deliberately — that layer is
called platform engineering and every FS team rebuilds it yearly. Do NOT
claim: algorithm novelty, the sim itself, superiority over photorealistic
sims for in-loop camera testing.

## 7. RES / authority model

RES = Remote Emergency System: marshal's radio remote (E-stop → EBS; GO =
final human authorization; lost link → auto-stop). Sim: Foxglove "Gross
Funk" panel = virtual RES; /go service; /ebs → EMERGENCY_BRAKE. Controllers
gate on `ASState: DRIVING` and cease commanding when it's lost — stops
obeyed by construction; same code obeys real RES via the real VCU.
Presentation text: `docs/presentation_accel_res.txt`, `docs/res_slide.txt`.

## 8. Asset inventory

| Where | What |
|---|---|
| `github.com/red-lance/FSDV-Sim` | THE repo (main branch; perception merged) |
| `README.md` | run instructions, containment rules, sweep runbook, EKF section, gotchas |
| `docs/PROJECT_HANDOFF.md` | this file |
| `docs/sem7-gantt.html` | 12-week Gantt as presented (title kept: "Autonomous Vehicle using LiDAR and Depth Camera"; actual work ran ~3 weeks ahead) |
| `docs/odom_profile_ekf_2026-08-12.json` | measured EKF error profile |
| `docs/skidpad_fullsim_{groundtruth,ekf}.json` | prediction-test data |
| `docs/presentation_accel_res.txt`, `docs/res_slide.txt` | presentation text |
| `~/eufs` | sim workspace — rebuildable from gitlab.com/eufs, EXCEPT our plugin_params.yaml noise edit (documented in §4) |

## 9. First moves when resuming

1. Run the YOLO profile extraction (5.1.1) — the only pending item that
   blocks the headline result.
2. Bias-ramp knob + big-orange lap counter (5.1.2–3) — small code, big wins.
3. Start Jetson procurement if not already moving.
4. Then 5.1 in order. When stuck on context, this doc + README + `git log
   --oneline` is the full story.
