# Autonomous Warehouse Robot
Autonomous warehouse logistics robot built for NERC 2026. Features C++ hierarchical navigation and control stacks, direct-ramp path optimization, and CAD model of the robot's base. 
# Autonomous Warehouse Logistics Robot — NERC 2026

An autonomous mobile logistics platform engineered for the **National Engineering & Robotics Contest (NERC) 2026**. The system performs structured warehouse navigation, payload transport, and inclined ramp traversal using a deterministic C++ control architecture and a modular two-piece mechanical chassis.

---

## System Theory & Control Architecture

### 1. Hierarchical Finite State Machine (HFSM)
Autonomous execution is governed by a deterministic C++ state machine that decomposes the warehouse logistics task into discrete, verifiable states:
* **Zone Navigation & Localization:** Tracks robot pose and arena milestones using sensor feedback and odometry.
* **Payload Alignment & Handling:** Executes fine-grained approach maneuvers to interface reliably with warehouse storage zones.
* **Ramp Traversal:** Transitions into a high-torque, heading-locked control mode prior to incline entry to prevent slip or yaw drift.
* **Fault & Recovery Handling:** Monitors sensor bounds and state timeouts to prevent deadlock during high-speed runs.

### 2. Trajectory Optimization (Direct-Ramp Approach)
Rather than following a multi-turn zig-zag path across the arena, the navigation stack executes a direct linear trajectory toward the ramp:
* **Reduced Cycle Time:** Eliminates unnecessary deceleration, pivot turns, and re-acceleration phases.
* **Minimized Odometry Drift:** Fewer rotational maneuvers significantly reduce cumulative heading error from wheel slip and gear backlash.
* **Stable Incline Entry:** Approaching the ramp along a straight vector ensures both drive wheels engage the incline simultaneously, preventing asymmetric traction loss.

### 3. Closed-Loop Motion & Disturbance Rejection
* **Differential Drive Kinematics:** Translates linear velocity ($v$) and angular velocity ($\omega$) commands into individual wheel angular velocities ($\omega_L, \omega_R$) based on track width ($L$) and wheel radius ($r$).
* **Inclined Load Compensation:** Ascending a ramp introduces a longitudinal gravitational force component ($m g \sin\theta$). The closed-loop controller compensates for this external load disturbance to maintain steady climb velocity and straight-line tracking without stalling.

---

## Mechanical Architecture

The mechanical structure is split into two primary sub-assemblies (available in [`CAD/`](CAD/)):
* **Chassis Base (`base.STEP`):** Houses the drivetrain, power distribution, and control electronics. Designed with a low center of gravity (CoG) to maintain static and dynamic stability on inclined surfaces while carrying a payload.
* **Storage Rack (`storage rack.STEP`):** A dedicated upper payload frame mounted to the base, engineered to distribute cargo mass evenly over the drive axle and maintain traction under acceleration.

---

## Repository Structure

```text
Autonomous-Warehouse-Robot/
├── CAD/
│   ├── base.STEP               # Chassis base 3D CAD model (STEP AP214)
│   └── storage rack.STEP       # Payload storage rack 3D CAD model (STEP AP214)
├── code/                       # C++ navigation stacks, state machine, and motor drivers
├── Pics/                       # Hardware build photographs and CAD visual renders
├── .gitattributes
├── .gitignore
├── LICENSE
└── README.md