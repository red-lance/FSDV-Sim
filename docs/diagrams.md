# System Diagrams — PlantUML Source

Six diagrams covering the assignment's checklist: a complete flowchart (decisions,
errors, repeated operations), a UML diagram, a state diagram, a data-flow diagram, a
timing diagram, and — since no database is used — a data-design diagram (class
diagram of the ROS message schema) in place of an ER diagram.

Paste each block into your PlantUML generator separately (one `@startuml`/`@enduml`
block = one diagram = one export). All six are self-contained; none depend on each
other. Deployment target is labelled **NVIDIA Jetson**, matching the project's actual
future-work target (not Raspberry Pi) — swap the label in Diagram 3 if your team is
actually targeting a Pi instead.

---

## 1. Flowchart — Mission Controller Execution (Activity Diagram)

**What it shows:** the generic control loop every one of the four mission
controllers (Acceleration, Skidpad, Trackdrive, Autocross) runs, including the state
gate, the closed-loop speed-control workaround for a real simulator bug, the
perception-gate debounce logic that fixed a real double-counting bug, and the
message-publish loop that repeats every tick.

```plantuml
@startuml flowchart_mission_controller
start

:Node starts;
:Subscribe to /sim/ros_can/state_str;

repeat
  :Receive state_str message;
  if (AMIState == my mission?) then (no)
    :Idle this tick;
  else (yes)
    if (ASState == DRIVING?) then (no)
      :Idle — wait for RES "GO";
    else (yes)
      if (first tick entering DRIVING?) then (yes)
        :Latch start pose from /odom;
        :Reset lap counter / distance;
      endif

      :Read latest sensor topics\n(/odom, /cones or /cones/lenient, /imu);

      if (mission == Trackdrive or Autocross?) then (yes)
        :Compute nearest blue/yellow\ncone-pair midpoint;
        if (no cones detected recently?) then (yes)
          :Slow down, then stop\n(blind-stop timeout);
          note right
            Fixed bug: timeout must be measured
            from last USABLE detection, not last
            message (empty messages still arrive)
          end note
        else (no)
          :Pure Pursuit steering\ntoward midpoint;
        endif

        :Check big-orange gate pair in view;
        if (gate newly visible AND\ncooldown elapsed?) then (yes)
          :Increment lap count;
          note right
            Debounced by gate_cooldown (8s) +
            gate_reset_time (3s absence dwell) —
            fixes a real double-count bug under
            20% detection dropout
          end note
        endif
      else (Acceleration/Skidpad)
        :Pure Pursuit (Skidpad)\nor straight-line target (Accel);
      endif

      :Compute target speed;
      :accel = Kp * (target_speed - measured_speed);
      note right
        Closed-loop, not open-loop:
        simulator silently ignores
        drive.speed on /cmd — only
        drive.acceleration and
        drive.steering_angle are read
      end note
      :Clamp accel to [-brake_limit, +accel_limit];

      if (finish condition reached?\n(distance / lap count / gate)) then (yes)
        :Set braking = true;
        if (speed below stop threshold?) then (yes)
          :Hold brake (prevents coast);
        endif
      endif

      :Publish AckermannDriveStamped on /cmd;
    endif
  endif
repeat while (node still running?) is (yes)
->no;
stop
@enduml
```

---

## 2. State Diagram — Autonomous System (AS) State Machine

**What it shows:** the official FS Driverless AS state machine as implemented by
eufs\_sim2 and driven externally by the RES (Remote Emergency Stop) slider and
emergency-stop button — not something the team built, but the exact interface every
mission controller gates on.

```plantuml
@startuml state_as_statemachine
[*] --> OFF

OFF --> READY : Set Mission (AMIState chosen)
READY --> DRIVING : RES slider -> GO
DRIVING --> FINISHED : mission end condition\n(distance / laps / gate)
DRIVING --> EMERGENCY : RES emergency-stop pressed
READY --> EMERGENCY : RES emergency-stop pressed
FINISHED --> OFF : reset
EMERGENCY --> OFF : reset

state DRIVING {
  [*] --> Idle
  Idle --> Active : mission controller\nself-gates on (AMIState\nmatches, ASState == DRIVING)
  Active --> Idle : wrong mission selected\n(controller stands down)
}

note right of EMERGENCY
  Independent of autonomy software —
  RES can cut power regardless of what
  the mission controller is doing,
  mirroring the real competition rule.
end note

@enduml
```

---

## 3. UML Component / Deployment Diagram — System Architecture

**What it shows:** the containment architecture — `fs_autonomy`'s only shared
dependency with the simulator is `eufs_msgs`, so the simulator can be swapped for
real driver nodes on an NVIDIA Jetson with no code change. Two deployment nodes are
shown side by side (today vs. future) sharing the identical topic contract.

```plantuml
@startuml component_architecture
skinparam componentStyle rectangle

package "Shared interface" {
  [eufs_msgs\n(message definitions only)] as msgs
}

node "Development target (today)\nUbuntu 22.04 VM" {
  component "eufs_sim2\n(simulator)" as sim
  component "AS state machine" as asm
  component "Sensor plugins\n(LiDAR, camera, IMU,\nGPS, wheel speed)" as sensors
  sim -down-> asm
  sim -down-> sensors
}

node "Deployment target (future work)\nNVIDIA Jetson" {
  component "Real sensor driver nodes\n(LiDAR, camera, IMU,\nwheel encoders)" as realsensors
  component "Real VCU interface node" as realvcu
}

package "fs_autonomy (ROS 2 package)" {
  component "wheel_odometry" as wo
  component "imu_frontend" as imf
  component "EKF\n(robot_localization)" as ekf
  component "accel_driver" as acc
  component "skidpad_driver" as skid
  component "trackdrive_driver\n(also serves Autocross)" as track
}

sensors ..> msgs : publishes\n(ConeWithColorProbabilityArray,\nImu, Odometry)
realsensors ..> msgs : publishes\n(same message types)

sensors -right-> wo
sensors -right-> imf
realsensors -right-> wo
realsensors -right-> imf
wo --> ekf
imf --> ekf
ekf --> acc
ekf --> skid
ekf --> track
asm -down-> acc : /sim/ros_can/state_str
asm -down-> skid
asm -down-> track

acc --> realvcu : /cmd (AckermannDriveStamped)
skid --> realvcu
track --> realvcu

note bottom of msgs
  fs_autonomy's ONLY build dependency
  on the simulator. Swapping the top node
  for the Jetson node requires zero
  changes to fs_autonomy's own code.
end note

@enduml
```

---

## 4. Data-Flow Diagram — Topics Between Nodes

**What it shows:** the actual ROS 2 topic graph as data flow — every arrow is a
real topic name and message type, not an abstraction. This doubles as the
"data design" required in place of an ER diagram, since there is no database in
this system; all state lives in in-memory node state and ROS topics.

```plantuml
@startuml dataflow_topics
left to right direction
skinparam rectangle {
  BackgroundColor<<node>> #EFE7D8
  BackgroundColor<<topic>> #E6EFEA
}

rectangle "Sensor plugins\n(sim or Jetson)" <<node>> as sensors
rectangle "/imu/data\n(sensor_msgs/Imu)" <<topic>> as t_imu
rectangle "/wheel_speeds" <<topic>> as t_wheel
rectangle "/cones/lenient\n(ConeWithColorProbability\nArray)" <<topic>> as t_cones
rectangle "/odom\n(nav_msgs/Odometry)" <<topic>> as t_odom

rectangle "imu_frontend" <<node>> as imf
rectangle "/imu/data_cov" <<topic>> as t_imucov

rectangle "wheel_odometry" <<node>> as wo
rectangle "/wheel_odometry/twist\n(TwistWithCovarianceStamped)" <<topic>> as t_twist

rectangle "EKF node\n(robot_localization)" <<node>> as ekf
rectangle "/odometry/filtered" <<topic>> as t_filtered

rectangle "Mission controller\n(accel / skidpad / trackdrive)" <<node>> as ctrl
rectangle "/sim/ros_can/state_str" <<topic>> as t_state
rectangle "/cmd\n(AckermannDriveStamped)" <<topic>> as t_cmd

rectangle "VCU\n(sim physics or\nJetson -> real VCU)" <<node>> as vcu

sensors --> t_imu
sensors --> t_wheel
sensors --> t_cones
sensors --> t_odom

t_imu --> imf
imf --> t_imucov
t_wheel --> wo
wo --> t_twist

t_imucov --> ekf
t_twist --> ekf
ekf --> t_filtered

t_filtered --> ctrl
t_cones --> ctrl
t_state --> ctrl
ctrl --> t_cmd
t_cmd --> vcu
vcu --> sensors : closes the loop\n(new vehicle state)

@enduml
```

---

## 5. Timing Diagram — Sensor, Fusion and Control Loop Rates

**What it shows:** the real update rates involved, which matter because the EKF
fuses two sensor streams at different natural rates and the control loop must not
run faster than the data it depends on.

```plantuml
@startuml timing_rates
robust "LiDAR / cone detection" as lidar
robust "IMU" as imu
robust "Wheel speed" as wheel
robust "EKF fusion" as ekf
robust "Mission controller tick" as ctrl

lidar has Idle, Publish
imu has Idle, Publish
wheel has Idle, Publish
ekf has Idle, Fuse
ctrl has Idle, Compute

@0
lidar is Idle
imu is Idle
wheel is Idle
ekf is Idle
ctrl is Idle

@0
imu is Publish
@5
imu is Idle
@20
imu is Publish
@25
imu is Idle

@0
wheel is Publish
@10
wheel is Idle
@20
wheel is Publish
@30
wheel is Idle

@0
lidar is Publish
@50
lidar is Idle
@100
lidar is Publish
@150
lidar is Idle

@2
ekf is Fuse
@4
ekf is Idle
@22
ekf is Fuse
@24
ekf is Idle
@32
ekf is Fuse
@34
ekf is Idle

@6
ctrl is Compute
@8
ctrl is Idle
@26
ctrl is Compute
@28
ctrl is Idle

@100
lidar is Publish

note right of ekf : EKF runs at 50 Hz (20 ms period),\nfusing IMU (~100-200 Hz effective)\nand wheel odometry (~100 Hz) —\nfaster than either sensor alone needs,\nso it never waits on a stale value.
note right of lidar : Cone detection is the slowest\nstream (~10-20 Hz) — the mission\ncontroller tolerates this via the\nblind-stop timeout, not by blocking.

@enduml
```

---

## 6. Data Design — ROS Message Schema (in place of an ER diagram)

**What it shows:** this project uses no database — all data is transient, in-memory
ROS messages passed between nodes. This class diagram is the equivalent "data
design" artifact: the schema of every message type that actually crosses a node
boundary, with the fields each controller reads.

```plantuml
@startuml data_design_messages
class AckermannDriveStamped {
  +Header header
  +AckermannDrive drive
}
class AckermannDrive {
  +float32 steering_angle
  +float32 speed  ' published but IGNORED by sim (Defect 1)
  +float32 acceleration  ' actually consumed
}
AckermannDriveStamped *-- AckermannDrive

class Odometry {
  +Header header
  +Pose pose
  +Twist twist
}

class Imu {
  +Header header
  +Quaternion orientation
  +Vector3 angular_velocity
  +Vector3 linear_acceleration
  +float64[9] orientation_covariance
  +float64[9] angular_velocity_covariance
  +float64[9] linear_acceleration_covariance
}

class TwistWithCovarianceStamped {
  +Header header
  +Twist twist
  +float64[36] covariance
}

class ConeWithColorProbabilityArray {
  +Header header
  +ConeWithColorProbability[] cones
}
class ConeWithColorProbability {
  +Point2d point
  +float32 prob_blue
  +float32 prob_yellow
  +float32 prob_orange_big
  +float32 prob_orange_small
}
ConeWithColorProbabilityArray *-- ConeWithColorProbability

class "state_str (std_msgs/String)" as StateStr {
  +string data
  .. parsed as ..
  AMIState : ACCELERATION|SKIDPAD|\n  AUTOCROSS|TRACK_DRIVE
  ASState : OFF|READY|DRIVING|\n  FINISHED|EMERGENCY
}

Imu --> TwistWithCovarianceStamped : fused by EKF into >
Odometry <.. TwistWithCovarianceStamped : EKF output\n(/odometry/filtered)
ConeWithColorProbabilityArray --> AckermannDriveStamped : consumed by\nmission controller to produce
StateStr --> AckermannDriveStamped : gates whether\ncontroller publishes at all

@enduml
```

---

### Brief explanations (for the submission writeup)

1. **Flowchart** — every mission controller is one state-gated loop; the two
   `note` callouts mark the two real bugs the error-handling logic was written to
   work around (silent `drive.speed`, lap double-counting).
2. **State diagram** — the AS state machine is provided by eufs\_sim2 (and by the
   real competition rules), not built by the team; it is the external trigger every
   controller's gate depends on.
3. **Component/deployment diagram** — visualises the "one shared dependency"
   claim directly: both the simulator and the Jetson deployment node connect to
   `fs_autonomy` through the same `eufs_msgs` contract.
4. **Data-flow diagram** — every arrow is a real topic name; this is the literal
   wiring of the system, not a simplification.
5. **Timing diagram** — shows why the EKF runs faster than either fused sensor
   needs to (never blocks on stale data) while the control loop tolerates the
   slower, noisier cone-detection stream via an explicit timeout rather than by
   blocking.
6. **Data design** — substitutes for an ER diagram since no database exists;
   documents the actual schema and field-level provenance of every message that
   crosses a node boundary, including which fields are silently unused (Defect 1).
