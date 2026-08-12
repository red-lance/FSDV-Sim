# PROJECT HANDOFF — FS Driverless Sim-First Platform

**Written 2026-08-10.** This document compiles everything developed in AI-assisted
work sessions (July–Aug 2026) that is NOT already captured in code or in
`autonomy_ws/README.md`. The account those sessions ran under is going away;
this file is the durable copy of the strategy, roadmap, and ideas.

**How to use it:** read it yourself, and paste it (plus `README.md`) into any
future AI session as opening context — it is written to bootstrap either.

---

## 1. The thesis (what this project actually is)

Not "four controllers that drive a sim car." It is a **development platform**
for Formula Student driverless: a team can *develop* autonomy software against
a simulator, *verify* it automatically and cheaply, and *deploy the identical
software* to the car's computer (Jetson) unchanged.

> **Develop → Verify → Deploy.** Every piece of work is one leg of that tripod.

- Controllers = the platform's **proof of use** (reference implementations).
- SIL harnesses (software-in-the-loop) = its **proof of correctness** (cheap, repeatable closed loop).
- Containment rules + Docker = its **proof of portability** (sim-free deploy).

One-liner for report/viva: *"Simulators, stacks, and datasets all exist online —
what doesn't exist is a verified path between them: documented interfaces,
repeatable closed-loop testing, and identical-code deployment to embedded
hardware. That path is the platform."*

Scope fence (state it, don't apologize for it): no perception model training,
no SLAM, no learned control. The platform **defines their interfaces and
derives their requirements** (see §6); implementing them is future work / Sem 8.

## 2. State of the work (as of 2026-08-10)

Done and verified (numbers you can quote):

| Piece | Verified result |
|---|---|
| `accel_driver` | Holds 8 m/s, brakes at 75 m, dead stop ~84 m (matches v²/2a = 8 m physics) — sil_accel + full sim |
| `skidpad_driver` | 5 cm max radial error, exact 8π of turning, clean stop — vs sil_skidpad bicycle model |
| `trackdrive_driver` | 3/3 laps, 0.10 m max deviation, 0 cones hit — vs sil_trackdrive (92 m circuit, 110°/15 m/15 Hz sensor) |
| Upstream bug 1 | `/cmd` consumes ONLY drive.acceleration + steering_angle; drive.speed silently ignored (eufs_sim2 control.cpp) |
| Upstream bug 2 | Fused `/cones` advertised but NEVER published (cone_fusion.cpp); real feed is `/cones/lenient` |

Also done: mission-gated launch orchestration (`autonomy.launch.py` — sim never
launches user code; controllers self-gate on `AMIState`), multi-arch Dockerfile,
`scripts/check_bridge.py` diagnostics, DDS-isolation testing doctrine
(**always** run SIL-harness tests under isolated `ROS_DOMAIN_ID`, e.g. 42).

Not yet done at time of writing: skidpad/trackdrive/autocross validation runs in
the FULL sim (harness-verified only); everything in §4.

## 3. The 12-week plan (as presented to the guide)

`docs/sem7-gantt.html` — open in a browser, print or screenshot. Official
project title kept: "Autonomous Vehicle using LiDAR and Depth Camera".
Presented pacing (actual work ran ~3 weeks ahead of it): bring-up W1–2, audit
W1–2, workspace design W2–3, odom controllers W3–4, perception controllers
W4–5, harnesses W4–6, full-sim validation W6, CI W6–7, robustness W7–8,
contract doc W8–9, Jetson procurement W5–7 (parallel), embedded deploy +
benchmarks W9–10, presentation W10, upstream MRs + buffer W9–10, report W11–12.

## 4. Remaining work — the agreed roadmap, with the ideas fleshed out

### 4.1 CI regression suite
Wrap each SIL-harness run in a script with pass/fail exit codes
(`run_tests.sh`): launch harness + controller under an isolated
`ROS_DOMAIN_ID`, assert the mission-complete condition within a timeout
(accel: stopped past 75 m; skidpad: 8π + stop; trackdrive: N laps, 0 collisions).
Run inside the existing Docker image → works on any CI runner
(GitHub Actions / GitLab CI) on every commit. This is automation of what
already runs, not new construction.

### 4.2 Monte-Carlo robustness sweeps (the report's results chapter)
**STATUS 2026-08-11: machinery BUILT and verified.** sil_trackdrive now has a
parameterized error model (per-range P(detect), bearing/range noise, color
flips, false positives, latency frames, seeded RNG, realtime_factor) that can
load `error_profile.json` from scripts/extract_error_profile.py directly.
`scripts/run_sweeps.py` batch-runs seeded episodes to CSV;
`scripts/plot_sweeps.py` renders success-rate curves. First sweep immediately
found + fixed a REAL controller bug: blind-stop was clocked from the last cone
*message*, but a live detector seeing nothing still publishes empty arrays at
frame rate, so a never-seeing car crept at min_speed indefinitely (335 m off
track). Now clocked from the last usable *target*; a blind car refuses to
move. Early finding: uniform dropout is nearly harmless (passes at 5%
detection!) because detections integrate across frames — the binding
constraints will be range-concentrated misses, color flips, FPs, latency.
Remaining: run the real YOLO profile + big sweeps, make the graphs.
Sweep dimensions (all knobs already exist): cone dropout / shift / recolour
(`track_changer` plugin params or own harness), sensor FoV / range / rate,
position noise, sensor dropout intervals. Metrics per run: mission success,
max path deviation, cones hit. Thousands of runs are affordable because the
harnesses are object-level + substitute physics (seconds per run, no GPU).

**Headline idea — read the curves backwards:** the sweep output is a
*requirements specification* for future subsystems. "Success collapses below
X% detection rate inside 15 m / above Y m position noise" ⇒ that's the spec
the future perception team must meet. This turns robustness graphs from
pretty pictures into an engineering deliverable nobody online provides.

### 4.2b State estimation (added 2026-08-12) — the EKF thread
The odometry twin of the perception story: measure a real estimator, inject
its profile, swap it into the loop. BUILT: `wheel_odometry` (wheel speeds →
covariance-stamped twist; sim emits rev/s despite the msg saying RPM),
`imu_frontend` (fills covariances the sim zeroes), `config/ekf_params.yaml` +
`estimation.launch.py` (robot_localization dead-reckoning: IMU yaw rate +
accel, wheel vx; no GNSS/abs heading), `odom_topic` param on all controllers,
`odom_corruptor.py` knobs in all three SIL harnesses (score truth, publish
estimate), `extract_odom_profile.py` (EKF-vs-truth → calibrated knob values),
run_sweeps `--mission accel|skidpad|trackdrive`. Sim IMU noise raised to
BMI088-class realism (was identity) + yaw-drift ramp, in eufs plugin_params
(local upstream edit — symlinked install, survives until eufs rebuild).
FIRST DATA: skidpad circ_err_max 0.068/0.199/0.558 m at 0/2/6 deg/√min yaw
drift (monotonic, fails ~10); trackdrive immune to 3 m/√min position drift
except phantom extra laps. Headline: perception-anchored steering robust to
odom error, geometry-anchored fragile ⇒ the engineering case for cone-based
localization, from our own data. UPSTREAM BUG #3 found during recon:
/ros_can/twist has scrambled angular axes (x=pitch,y=yaw,z=roll) + zero
covariance — never fuse it; MR candidate. DONE 2026-08-12 — MEASURED (accel mission, 83 m, docs/odom_profile_ekf_*.json):
pos drift 2.0 m final = 2.4%/m traveled; yaw err 2.5°; vel σ 0.017 m/s.
Calibrated knobs: odom_drift 2.43 m/√min, yaw_drift 3.08 deg/√min, pos_noise
0.02, vel_noise 0.017. Cross-ref the skidpad sweep: 3.1 deg/√min ⇒ predicted
~0.3 m circle error — degraded but PASSING on this EKF (untested prediction,
good report material). IN-THE-LOOP DEMO DONE: accel mission closed-loop on
/odometry/filtered — driver belief "82.9 m" vs true stop 82.88 m (2 cm!),
while EKF ABSOLUTE pose was >80 m off (never saw /reset teleport, heading
drift across runs). Lesson for report: absolute pose drifts unboundedly;
relative quantities stay usable; controllers built on relative distance
survive dead-reckoning. Headless mission control (no Foxglove): services
/set_mission (int16, ACCELERATION=1..TRACK_DRIVE=4), /go, /reset, /ebs.
When real IMU arrives: Allan variance (desk, hours, allan_variance_ros) →
values into imu_frontend + sim imu_plugin covariance → re-measure. EKF is
tuned (Q/R), not trained; sim ground truth enables scripted Q grid-search.

PREDICTION TEST DONE (2026-08-12) — the model-fidelity result, all true-
geometry circle error via scripts/measure_skidpad.py (docs/skidpad_fullsim_*):
| run                              | mean   | max    |
| SIL kinematic, no drift          | 0.015  | 0.054  |
| full sim, ground-truth odom      | 0.204  | 0.207  |  <- 1st full-sim skidpad validation; gap vs SIL = vehicle-dynamics offset
| SIL @ measured 3.1 deg/sqrt-min  | ~0.14  | ~0.30  |  <- the prediction
| full sim ON THE EKF              | 0.33   | 0.69   |
MEAN validated: additive model (dynamics 0.19 + SIL drift 0.13 = 0.32) vs
measured 0.33 — within 0.01 m. MAX underpredicted ~2x: real EKF drift is a
time-correlated bias RAMP (error concentrates late in run), SIL knob is a
white random walk. Both are report gold: fidelity confirmed on the mean,
discrepancy on the max explained mechanistically + improvement path (add a
bias-ramp mode to odom_corruptor). Mission completed on EKF regardless:
full entry/laps/exit, true final lateral offset 1.21 m, 0.69 m worst circle
error still inside the 1.5 m half-lane. Metric caveat fixed on the way: the
|y|>1 gate scored the drifted EXIT leg against the circles (16.9 m phantom
max) — both sil_skidpad and measure_skidpad now gate on the circles' x-span.

### 4.3 Jetson deployment + benchmarks
JetPack 6 = Ubuntu 22.04 = Humble parity. Build the existing Dockerfile for
arm64, run sim on laptop + stack on Jetson over LAN (same DDS domain).
Benchmarks: end-to-end latency (odom→cmd), control-rate stability, CPU/RAM.
The demo: same code, x86 dev VM → arm64 embedded, zero changes. Procurement
is the only external dependency in the whole plan — start it EARLY.

### 4.4 Two-layer interface contract document
The formal write-up of what the audit discovered. Layer 1 (raw sensor):
/odom, IMU, wheel speeds, GNSS. Layer 2 (object): car-relative cone arrays
(`/cones/lenient` semantics: FoV-filtered, body frame). Plus the control
contract (`/cmd`: acceleration + steering ONLY — document bug 1 here) and the
authority contract (state machine: mission string gating, DRIVING gate,
`/ebs`, GO; see §7). For each topic: type, frame, rate, QoS, publisher of
record in sim vs on car.

### 4.5 Upstream contributions (credibility + report material)
- MR 1: publish the fused `/cones` in cone_fusion.cpp (ideally with optional noise).
- MR 2: document or implement `drive.speed` handling in control.cpp.
- Stretch: Foxglove cone-array → Marker converter plugin (removes need for cone_viz).

## 5. Perception testing strategy (the FSOCO / LiDAR discussion)

Key fact (verified in `eufs_sim2/config/plugin_params.yaml`): the sim renders
NO images and NO point clouds. Its "camera" (110°, 1–20 m, colour) and "lidar"
(180°, 1–100 m, colourless) are **geometric FoV filters over ground-truth
cones** — the sim models perception's *output*, not its *input*.

Three pathways to test perception with the platform:

**Path 1 — offline evaluation, in-loop consequences (the strong one).**
Train/evaluate the model offline on real data (camera: FSOCO dataset), extract
its ERROR PROFILE — P(detect|range), position σ vs range, colour-confusion
rate, false positives/scan, latency — then inject that measured profile into
the harness sensor models and run the Monte-Carlo sweeps. Answers: "can the
car drive with OUR detector's measured error characteristics?" Couples mAP to
mission outcome.

**Path 2 — swap-in at the object layer.** Any pipeline that outputs
car-relative cone detections replaces the sim's geometric sensor directly
(trackdrive_driver can't tell the difference). Sim provides ground truth for
free ⇒ automatic scoring, no hand labeling.

**Path 3 — images in the loop.** Only via the old Gazebo eufs_sim (simulated
ZED + Velodyne). Heavy, and camera models hit the sim-to-real domain gap
(model trained on real photos sees synthetic renders as out-of-distribution).
NOTE: the domain gap is much SMALLER for LiDAR — point clouds are geometry —
so Path 3 is defensible for LiDAR pipelines even where it isn't for camera.

**LiDAR specifics (Path 1 recipe).** Pipeline under test is classical:
ground removal → clustering → cone filter. Labeled data: record own bags
(cones are isolated clusters — labeling is cheap), check published team data
(e.g. AMZ Driverless), or Gazebo Velodyne. Measure vs range: P(detect|r)
(cliff when point count/cone gets small), position σ, FP/scan, latency.
Inject: distance-dependent dropout + Gaussian position noise + spurious cones
+ publish delay in sil_trackdrive's sensor model (our code, easy to extend);
`colour: false` downstream. The structural result: trackdrive steers on
blue-left/yellow-right, LiDAR has no colour ⇒ sweeps expose the fusion
requirement (camera = colour near, lidar = geometry far) — and the fusion
plugin is exactly where upstream bug 2 lives. Fix the bug, then sweep camera
+ lidar error profiles JOINTLY to show when fusion saves the mission.

## 6. Differentiation vs what's available online (viva ammunition)

- **Simulators** (eufs_sim2, FSDS/AirSim, CARLA): give you a car in a world,
  stop there. No methodology, no verification, no deployment path.
  Photorealistic ones need GPUs and can't do cheap Monte-Carlo.
- **Team stacks** (AMZ fsd_skeleton etc.): ROS 1, unmaintained, monolithic,
  shaped around one team's car. Artifacts of a car, not adoptable platforms.
- **Datasets** (FSOCO): data, no system.
- **General AV frameworks** (Autoware): industrial-scale, oversized for
  200 cones and a Jetson.

What none provide (= the contribution): the develop→verify→deploy pipeline as
a whole; interfaces as the documented product; verification cheap enough to
run thousands of closed-loop missions; requirements specs derived from sweeps;
ROS 2-current, container-first, GPU-free.

If pushed "isn't this just glue?": yes, deliberately — integration, interface
contracts, and verification evidence are exactly what's missing online; in
industry that layer is called platform engineering, and every new FS team
currently rebuilds it from scratch.

Do NOT claim: pure pursuit / P-control novelty, the sim itself, superiority
over photorealistic sims for in-loop camera testing.

## 7. RES / state-machine authority model (recap)

RES = Remote Emergency System: marshal's radio remote, E-stop (fires EBS) +
GO (final human authorization); lost radio link ⇒ automatic stop. In the sim:
Foxglove "Gross Funk" panel = virtual RES; GO moves the state machine
READY → DRIVING; `/ebs` service ⇒ EMERGENCY_BRAKE. Our controllers gate on
`ASState: DRIVING` and cease commanding the instant it's lost — RES stops are
obeyed by construction; identical code obeys real RES hardware via the real
VCU. Presentation text: `docs/presentation_accel_res.txt`, `docs/res_slide.txt`.

## 8. Asset inventory

| Where | What |
|---|---|
| `~/autonomy_ws` | THE deliverable. This repo. Git-init'd but **no commits yet** — commit + push to a personal remote FIRST. |
| `~/autonomy_ws/README.md` | Containment rules, node table, both upstream bugs, run instructions, gotchas. |
| `docs/sem7-gantt.html` | 12-week Gantt as presented (browser → print/screenshot). |
| `docs/presentation_accel_res.txt` | Spoken scripts + slide bullets + Q&A for accel controller and RES. |
| `docs/res_slide.txt` | Single-slide RES text + speaker note. |
| `~/eufs` | Sim workspace (rebuildable from gitlab.com/eufs public repos — not precious). |
| `~/Downloads/CLAUDE.md` | Original handoff context doc (historical). |

## 9. First moves when resuming

1. **Commit and push this repo to a personal GitHub/GitLab immediately.**
   Everything else in this document is recoverable from your head; the code
   and docs are not.
2. Validate skidpad + trackdrive + autocross in the FULL sim (small_track) —
   the only cheap unfinished verification.
3. Start Jetson procurement (longest lead time, only external dependency).
4. Then follow §4 in order: CI → Monte-Carlo → contract doc → Jetson →
   upstream MRs → report.
