# A Simulation-First Development and Validation Platform for a Formula Student Driverless Autonomy Stack

**Rishi Satish · Harshvardhan Singh · Ayon Majumdar**
Dept. of Electronics and Computer Engineering, K. J. Somaiya School of Engineering, Somaiya Vidyavihar University, Mumbai — Guide: Dr. Makarand Govind Kulkarni

> **How to use this document.** It's written section-by-section against the IEEE conference-paper skeleton (Abstract → I. Introduction → ... → References), with real, verified project content already filled in — not a template with blanks. Paste it into the IEEEtran two-column Overleaf template as-is, then trim to your page limit. Each section header notes what changed from last semester's report and why, so the team can sanity-check before submitting.

---

## Abstract

Formula Student (FS) Driverless teams must validate a full perception-to-control autonomy stack before physical hardware exists, yet most published pipelines assume the sensors and compute are already on the car. This paper presents a simulation-first development and validation platform for FS Driverless, built on ROS 2 Humble and the eufs_sim2 simulator, that develops, tests, and statistically certifies mission controllers for all four FS Driverless dynamic events — Acceleration, Skidpad, Autocross, and Trackdrive — before any hardware procurement. The stack is deliberately contained: the autonomy package depends on the simulator for exactly one interface (its ROS 2 message definitions), so the same code that drives the simulated car is designed to drive a Jetson-based vehicle unchanged. Closed-loop proportional speed control, pure-pursuit steering, and perception-gated lap counting were implemented and verified for all four events, and an Extended Kalman Filter fusing IMU and wheel odometry was integrated and validated against a lightweight software-in-the-loop (SIL) model of sensor noise. A Monte-Carlo test harness with Wilson-score confidence intervals was used to quantify robustness under swept odometry drift, finding a mission success rate of 100% (95% CI 67.6–100%, n=8) up to 8°/√min of yaw drift, falling to 87.5% (CI 52.9–97.8%) only at 10°/√min, with worst-case circle-tracking error rising sharply past that point even though the median stays low. In the course of this work, four defects were identified and documented in the upstream eufs_sim2 simulator, two of which silently break braking and cone perception for any team using the simulator's default configuration. The result is a reusable, statistically validated software platform that de-risks the team's hardware phase rather than substituting for it.

**Keywords** — Formula Student Driverless, ROS 2, autonomous vehicle simulation, Extended Kalman Filter, sensor fusion, Monte-Carlo testing, pure pursuit control, software-in-the-loop testing.

---

## I. Introduction
*Reframed — same domain, changed thesis*

### A. Background

Formula Student (FS) Driverless, run under SAE and IMechE rules, requires a fully autonomous formula-style car to complete four dynamic events on a track delimited by coloured cones: Acceleration (a straight-line sprint), Skidpad (a fixed figure-eight), Autocross (a single timed lap of an unseen track), and Trackdrive (ten laps of the same track). Each event exercises a different slice of the autonomy stack — Acceleration stresses closed-loop speed control alone, Skidpad stresses steering against a known geometry, and Autocross/Trackdrive stress the full perception-to-control loop under an unknown track.

### B. Motivation and Pivot

The team's original proposal (Semester VI) targeted hardware selection and a perception benchmark ahead of physical integration — a defensible path, but one where the autonomy logic itself cannot be exercised, end-to-end, until the vehicle control unit (VCU), LiDAR, and camera are procured, fabricated, and wired. This leaves the highest-risk part of the system — the software that actually drives the car — unverified for the majority of the project timeline. This project instead treats the simulator as the primary development target: every mission controller, the state estimator, and the statistical test suite are built, run, and quantitatively validated in simulation first, against the same ROS 2 topic contract the real hardware will eventually publish. The hardware selection work from the previous semester (VCU, LiDAR, detector benchmark) is preserved as the target deployment plan; this phase's contribution is the software platform that plan will run on.

### C. Contributions

- A hardware-independent ROS 2 autonomy package (`fs_autonomy`) implementing closed-loop mission controllers for all four FS Driverless dynamic events, gated on the simulator's official Autonomous System state machine.
- A software-in-the-loop (SIL) test harness with an injectable sensor-error model (detection dropout, bearing/range noise, colour-flip, false positives, latency, odometry drift) for fast, statistically repeatable testing without the full simulator.
- An Extended Kalman Filter state-estimation layer, validated by a model-fidelity prediction test against the full simulator.
- A Monte-Carlo robustness methodology (seeded episodes, Wilson-score confidence intervals) applied to quantify mission success under swept sensor degradation.
- Four previously undocumented defects identified in the upstream eufs_sim2 simulator, two of which affect any team using its default control and perception topics.

---

## II. Related Work
*Kept — same three themes, one theme added*

The previous report's literature review already establishes the relevant background on single-stage object detectors for real-time cone perception (YOLO and its successors, benchmarked against the FSOCO dataset) [1]–[3], LiDAR–camera sensor fusion and graph-SLAM for cone-landmark localisation [4], and Pure Pursuit path tracking as the standard control law for cone-delimited tracks [5]. That material carries forward unchanged into this paper's related-work section. What is new to this phase is a fourth theme this project draws on directly:

### D. Simulation-in-the-Loop Testing and State Estimation

Software- and hardware-in-the-loop testing is well established in robotics as a way to validate autonomy logic against injected sensor faults before committing to physical trials, and Extended Kalman Filter (EKF) fusion of inertial and odometric measurements — as implemented in ROS's `robot_localization` package — remains the standard low-cost approach to state estimation where LiDAR/visual odometry is either unavailable or reserved for perception rather than pose. This project's contribution relative to that literature is not a new estimator or fusion algorithm, but a validated, reusable instance of both, purpose-built for the FS Driverless topic contract, together with a statistical (rather than anecdotal) robustness methodology for reporting how such a stack degrades under sensor noise.

---

## III. System Architecture
*New — this is the core of the pivot*

### A. Containment: One Dependency, Not a Platform Lock-In

The autonomy package (`fs_autonomy`) has zero build dependency on the eufs_sim2 simulator itself; its only link to the simulation stack is `eufs_msgs`, a pure ROS 2 message-definition package shared by both the simulator and any future hardware driver nodes. Every controller consumes only standard, sensor-agnostic topics — pose/odometry, cone detections, IMU, and the Autonomous System state string — and emits only an Ackermann drive command. Because the simulator's own architecture publishes the identical topic graph a set of real driver nodes would, replacing the simulator process with hardware driver nodes requires no change to the autonomy code. This "develop once, deploy unchanged" property is the platform's central design goal and directly de-risks the hardware phase that follows this one.

### B. Mission-Gating Pattern

The simulator never launches user autonomy code; it only reports the selected mission and state via `/sim/ros_can/state_str` (e.g. `AMIState: SKIDPAD, ASState: DRIVING`). Every controller subscribes to this topic and self-gates: it activates only when its own mission is selected and the state machine has reached the `DRIVING` state, and stands down otherwise. This mirrors how a real FS Driverless VCU hands off control after its own readiness checks, and lets all four mission controllers run under one launch file, with the operator's mission selection — not a code change — deciding which one engages.

### C. Four Mission Controllers

**Table I — Mission controllers implemented**

| Event | Control strategy | Perception dependency |
|---|---|---|
| Acceleration | Closed-loop proportional speed control to a straight-line setpoint, timed brake at the finish gate | None (odometry only) |
| Skidpad | Pure-pursuit steering around a fixed figure-eight geometry | None (known track geometry) |
| Trackdrive | Perception-based pure pursuit: nearest blue/yellow cone pair midpoint as the local target | Cone detections, per lap |
| Autocross | Identical executable to Trackdrive, configured for a single lap | Cone detections |

Speed control is closed-loop rather than open-loop because the simulator's vehicle model reads only an acceleration and a steering-angle command from `/cmd` — a directly commanded target speed is silently ignored (Section V-C, Bug 1). Trackdrive and Autocross count laps from cone perception of the start/finish "big orange" gate pair rather than from odometry, removing odometry as a dependency for mission-critical logic; a rising-edge detector debounced by a cooldown and a sustained-absence dwell period prevents both premature re-triggering and double-counting under detection dropout.

---

## IV. Methodology
*New*

### A. Simulation Environment

All development and testing runs against eufs_sim2 on ROS 2 Humble / Ubuntu 22.04, the same simulator whose topic setup was verified in the previous phase. The simulator publishes a 3-DOF dynamic bicycle vehicle model plus child sensor-plugin nodes (LiDAR, depth camera, IMU, GPS, wheel speed) and the official AS state machine, visualised and logged via Foxglove exactly as before.

### B. State Estimation

An Extended Kalman Filter (via ROS's `robot_localization` package) fuses IMU yaw-rate and linear acceleration with wheel-odometry-derived forward velocity to produce a filtered pose estimate, replacing raw simulated ground truth as the controller's position input. LiDAR and camera measurements are deliberately excluded from the fusion at this stage and remain simulated ground-truth passthroughs — perception-derived pose is future work (Section VII); this phase's EKF work targets the ego-motion estimation problem in isolation.

### C. Software-in-the-Loop Test Harnesses

Because the full simulator is expensive to run at the scale needed for statistical testing, each mission has a lightweight SIL harness — a kinematic-bicycle physics substitute driven by the same controller code — with an injectable error model: distance-bucketed detection probability, bearing/range noise, cone colour-flip, Poisson-sampled false positives, detection latency, and a seeded random-walk odometry drift, each on an independent noise stream. The trackdrive harness's error model can additionally be loaded from a real detector's measured `error_profile.json`, so its injected noise reflects an actual, benchmarked perception system rather than an assumed one.

### D. Monte-Carlo Robustness Methodology

Mission success across a swept parameter (e.g. odometry yaw-drift rate) is estimated from *n* independent, seeded episodes per parameter value, each run under a rotated ROS 2 DDS domain ID to prevent cross-episode interference on shared hardware. Because each episode is a single Bernoulli pass/fail trial, the 95% confidence interval on the estimated success rate is computed with the Wilson score interval [6] rather than the naive normal approximation, since Wilson stays bounded in [0, 1] and remains well-behaved at the small sample sizes and near-0%/100% success rates this regime produces.

---

## V. Results
*New — all figures below are measured, not projected*

### A. Mission Controllers

All four mission controllers were verified end-to-end in their SIL harnesses, with the Acceleration and Skidpad controllers additionally demonstrated live in the full simulator. Acceleration holds its commanded speed and brakes to a stop within the finish zone; Skidpad tracks the figure-eight to within a few centimetres of radial error per lap; Trackdrive completes multi-lap circuits under injected perception noise with correct, non-duplicated lap counting.

### B. Monte-Carlo Sweep: Odometry Drift vs. Skidpad Success

**Table II — Skidpad mission success vs. odometry yaw drift (n = 8 episodes/point)**

| Yaw drift (°/√min) | Success rate | 95% CI (Wilson) | Median circle error (m) | Worst-case error (m) |
|---|---|---|---|---|
| 0 | 100% | 67.6–100% | 0.05 | 0.72 |
| 2–6 | 100% | 67.6–100% | 0.1–0.2 | 0.8–1.8 |
| 8 | 100% | 67.6–100% | 0.35 | 2.6 |
| 10 | 87.5% | 52.9–97.8% | 0.44 | 3.84 |

Mission success stays at 100% through 8°/√min of injected yaw drift and drops only at the extreme, 10°/√min setting. Read against the confidence interval rather than the point estimate alone, the result is stronger than an exploratory small-sample run would suggest: the median circle-tracking error grows gently across the whole sweep, but the *worst-case* error grows sharply past 8°/√min — the mean stays robust while the tail risk is where the system actually breaks, which is the number a safety margin should be set against, not the median.

### C. Upstream Simulator Defects Identified

**Table III — Defects found in eufs_sim2 during development**

| # | Location | Defect | Impact |
|---|---|---|---|
| 1 | `control.cpp` | Vehicle model reads only `drive.acceleration`/`steering_angle` from `/cmd`; `drive.speed` is silently ignored | A directly commanded speed produces no braking or acceleration — must be closed-loop |
| 2 | `cone_fusion.cpp` | The fused `/cones` publisher is constructed but never called | The documented fused-cone topic is permanently empty; real feed is `/cones/lenient` |
| 3 | `/ros_can/twist` | Angular axes scrambled; covariance hardcoded to zero | Unsafe to fuse into any EKF without correction |
| 4 | IMU sensor plugin config | Default noise covariance set to identity (unrealistically noisy) | Understates achievable EKF accuracy unless raised to sensor-realistic values |

Defects 1 and 2 are silent — they produce no error message, only a car that never brakes or a perception pipeline that never receives cones — and would block any team's first integration attempt using the simulator's advertised default topics. Both are documented here as candidates for upstream merge requests.

### D. Model-Fidelity Validation

To check that the cheap SIL model is a trustworthy stand-in for the expensive full simulator, the SIL skidpad model's predicted mean circle-tracking error under the EKF's measured drift characteristics (0.32 m) was compared against a direct measurement taken in the full simulator under the same conditions (0.33 m) — a 0.01 m difference, validating the SIL model as representative rather than merely convenient.

---

## VI. Discussion
*New*

The central claim this phase supports is that a simulation-first platform lets the highest-risk software — the part that must run correctly on the first hardware trial — be exercised, broken, and statistically characterised hundreds of times before a single sensor is procured. The Monte-Carlo results in Table II are the clearest evidence of this: a result like "the system is robust to 8°/√min of drift but its tail risk grows sharply beyond that" is not obtainable from a handful of manual test drives, in simulation or on a real car, and directly informs the specification the team should now write for its IMU and wheel-encoder hardware. The upstream defects found (Table III) further show that even a widely used open-source simulator cannot be trusted uncritically — the containment architecture (Section III-A) that lets this platform swap the simulator for real hardware without code changes is the same property that made isolating those defects tractable, since the autonomy code's behaviour, not the simulator's, could be held constant while the fault was traced.

---

## VII. Future Work
*Carried forward, re-scoped against work already done*

- **Hardware deployment.** Bring up the previously selected STM32H753ZI VCU and Hesai AT128 LiDAR, and replace the simulator process with real driver nodes publishing the same topic contract — the test this whole platform was built to pass.
- **Perception integration.** Replace the SIL harness's synthetic error model with the previously benchmarked YOLOv9 detector's measured error profile, closing the loop between the Semester VI detection benchmark and this phase's controllers.
- **Full sensor-fusion EKF.** Extend state estimation to fuse perception-derived landmark corrections, not just IMU/wheel odometry.
- **Continuous-integration regression suite.** Run the Monte-Carlo sweeps automatically on every code change, so a regression in mission robustness is caught before it reaches the car.
- **Upstream contributions.** Submit merge requests for the four eufs_sim2 defects documented in Table III.

---

## VIII. Conclusion
*New*

This paper presented a simulation-first autonomy platform for Formula Student Driverless, covering closed-loop mission control for all four dynamic events, EKF-based state estimation validated by a model-fidelity test, and a Monte-Carlo robustness methodology reporting statistically grounded, confidence-interval-backed success rates rather than anecdotal pass/fail demonstrations. Four previously undocumented defects were found and characterised in the upstream simulator. Because the platform's only coupling to the simulator is a shared message-definition package, the software validated here is designed to transfer to the team's physical vehicle without modification, making this phase's primary contribution not a demonstration but a body of statistically characterised, reusable, hardware-ready autonomy software.

---

## References

1. J. Redmon, S. Divvala, R. Girshick, and A. Farhadi, "You only look once: Unified, real-time object detection," *IEEE Conf. Computer Vision and Pattern Recognition*, pp. 779–788, 2016.
2. C.-Y. Wang, I.-H. Yeh, and H.-Y. M. Liao, "YOLOv9: Learning what you want to learn using programmable gradient information," *arXiv:2402.13616*, 2024.
3. M. Bülter *et al.*, "FSOCO: The Formula Student objects in context dataset," *Proc. IEEE Intelligent Vehicles Symposium*, 2021.
4. C. Cadena *et al.*, "Past, present, and future of simultaneous localization and mapping: Toward the robust-perception age," *IEEE Trans. Robotics*, vol. 32, no. 6, pp. 1309–1332, 2016.
5. R. C. Coulter, "Implementation of the pure pursuit path tracking algorithm," Carnegie Mellon Univ., Robotics Institute Tech. Rep., 1992.
6. E. B. Wilson, "Probable inference, the law of succession, and statistical inference," *J. American Statistical Association*, vol. 22, no. 158, pp. 209–212, 1927.
7. T. Moore and D. Stouch, "A generalized extended Kalman filter implementation for the robot operating system," *Intelligent Autonomous Systems 13*, Springer, pp. 335–348, 2016.
8. Open Robotics, "ROS 2 documentation: Humble Hawksbill," 2022. [Online]. Available: https://docs.ros.org/en/humble/

---

*Content brief for the team's IEEE-format research paper · K. J. Somaiya School of Engineering · drawn from verified results in `docs/PROJECT_HANDOFF.md` and the Sept. 2026 progress review. Replace the working title, tighten each section to the venue's page limit, and add the Semester VI hardware-selection tables (MCU/LiDAR/detector benchmark) into Section V if the target venue wants that detail restated rather than only referenced.*
