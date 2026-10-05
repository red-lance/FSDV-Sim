# System Diagrams

**Formula Student Driverless — Simulation-First Autonomy Platform**
Rishi Satish · Harshvardhan Singh · Ayon Majumdar
Dept. of Electronics and Computer Engineering, K. J. Somaiya School of Engineering
Guide: Dr. Makarand Govind Kulkarni

---

> **How to use this template.** For each section below: open the matching PlantUML code block from `diagrams.md` in your generator, render it, export as PNG or PDF, and paste the image in place of the `[ INSERT IMAGE HERE ]` line. The explanation paragraph under each is already written — edit it if you want, but it's submission-ready as is. Delete this note box before submitting.

---

## 1. Flowchart — Mission Controller Execution

*Required diagram. Source: `diagrams.md`, Diagram 1.*

**[ INSERT FLOWCHART IMAGE HERE ]**

### Explanation

This flowchart shows the control loop every mission controller (Acceleration, Skidpad, Trackdrive, Autocross) runs on every tick. It begins by gating on the vehicle's official state machine — the controller only acts once its mission is selected and the car has been released to drive. Once active, it reads the latest sensor data, computes a steering and speed command, and publishes a drive command, repeating continuously until the node stops. Two decision points are specifically error-handling logic written around real defects found in the simulator during development: a blind-stop timeout (the car slows and stops safely if cone detections go missing, rather than driving off blind) and a debounced lap-gate counter (preventing a single dropped detection frame from being miscounted as two separate laps). A third decision point works around a defect in the simulator itself, where a directly commanded speed is silently ignored — the controller instead measures actual speed and continuously corrects it in a closed loop.

---

## 2. State Diagram — Autonomous System (AS) State Machine

*Relevant diagram selected to accompany the flowchart. Source: `diagrams.md`, Diagram 2.*

**[ INSERT STATE DIAGRAM IMAGE HERE ]**

### Explanation

This diagram models the Autonomous System state machine that governs the vehicle's lifecycle: off, ready, driving, finished, or emergency-stopped. The transitions are driven externally by the Remote Emergency Stop (RES) system — a safety-critical slider and emergency-stop button required by Formula Student Driverless rules — rather than by the team's own software. Every mission controller subscribes to this state and only takes control of the vehicle once it reaches the driving state for its own selected mission; it stands down immediately if the wrong mission is active or the state leaves driving. This diagram is included because the entire autonomy stack's activation logic is built directly around it.

---

## 3. Data Design — ROS Message Schema (in place of an ER Diagram)

*No database is used in this project; this diagram serves as the required data-design artifact in place of an ER diagram. Source: `diagrams.md`, Diagram 6.*

**[ INSERT DATA DESIGN IMAGE HERE ]**

### Explanation

All data in this system is transient — passed between independent software components as typed messages over a publish/subscribe interface, with no persistent storage or database anywhere in the pipeline. This diagram documents the schema of every message type that actually crosses a component boundary: vehicle drive commands, pose and velocity estimates, IMU readings, detected cone positions, and the system state string. It also records field-level provenance — notably, that one field of the drive command (a directly commanded speed) is published by earlier code but silently ignored downstream, a defect discovered during development and designed around rather than relied upon.

---

## Appendix (optional — include only if required page count allows)

The following additional diagrams are available in `diagrams.md` if more coverage is wanted: a UML component/deployment diagram showing the simulation-to-Jetson deployment architecture, a data-flow diagram of the full ROS topic graph, and a timing diagram of sensor/fusion/control loop rates. Each follows the same export-and-paste process as Sections 1–3 above.
