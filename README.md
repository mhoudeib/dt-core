# Navigation Robustified for DuckieTown 🚗♾️  
> Achieving indefinite autonomous driving in DuckieTown

<p align="center">
<a href="https://duckietown.com">
<img src="/assets/images/dtlogo.png" alt="Duckietown Logo" width="50%">
</a>
</p>

**Course:** IFT6757 – Autonomous Vehicles  
**Institution:** Université de Montréal  
**Date:** December 2025  

**Authors:**  
- Firmin Chapoulie  
- Guillaume Genois  
- Mohamad Houdeib  

**TODO:** Insert a high-quality GIF or screenshot of the Duckiebot driving indefinitely through intersections.

---

## Usage

```bash
dts matrix run --standalone -m assets/duckiematrix/maps/intersections
dts matrix attach [YOURBOTNAME] map_0/vehicle_0
dts devel build -H [YOURBOTNAME]
dts devel run -H [YOURBOTNAME] -M -L single_robot_indefinite_navigation
dts duckiebot keyboard_control [YOURBOTNAME]
```
After this, click the autopilot toggle in the keyboard_control window; this will launch the autonomous drive.

Using
```bash
dts duckiebot image_viewer [YOURBOTNAME]
```
You can also view the generated intersection trajectory published by `unicorn_intersection_node` on the topic:
```
/[YOURBOTNAME]/node/image_relayer/unicorn_intersection_node/debug/trajectory/jpeg
```
and view the Apriltags being captured on:
```
/[YOURBOTNAME]/node/image_relayer/apriltag_detector_node/detections/image/jpeg
```

## Table of Contents

- [Problem Statement & Motivation](#-problem-statement--motivation)
- [Related Work](#-related-work)
- [Methodology & System Design](#-methodology--system-design)
- [Implementation Details](#-implementation-details)
- [Lessons Learned](#-lessons-learned)
- [Future Work](#-future-work)
- [Usage](#-usage)

---

## Problem Statement & Motivation

Autonomous navigation in structured environments such as DuckieTown is often demonstrated on finite scenarios: a fixed map, a single intersection, or a limited number of maneuvers. However, indefinite navigation, where a robot can continuously drive, cross intersections, and return to lane following without manual resets, remains challenging due to accumulated perception errors, unstable state transitions, and unreliable intersection handling.

The objective of this project is to enable a Duckiebot to drive indefinitely in a DuckieTown environment with intersections **without traffic lights, by robustly detecting stop lines, interpreting intersection signs, planning trajectories, and safely transitioning back to lane following.

Concretely, we focus on the LF-I_noTL scenario (single robot, no traffic lights), which requires:
- Reliable stop line detection
- Correct identification of intersection type
- Safe and stable intersection crossing
- Robust state switching to avoid deadlocks or oscillations

![System Behaviors Hierarchy](src/_images/auto-behaviors-hierarchy.png)

This work serves as a foundation toward multi-robot indefinite navigation (LF-IV), where decentralized coordination becomes necessary :contentReference[oaicite:0]{index=0}.

---

## Related Work

Several prior efforts have addressed autonomous intersection management:

- Dresner & Stone (2008) proposed a reservation-based multi-agent system where vehicles negotiate intersection access through a request/confirm protocol. While efficient, this approach assumes reliable communication and global coordination.
- Azimi et al. (2013) introduced distributed V2V protocols with deadlock avoidance, achieving significant throughput improvements under realistic wireless conditions.

In DuckieTown specifically, prior student projects explored LED-based communication using frequency modulation to encode priorities at intersections. While conceptually elegant, these approaches were limited by:
- Sensitivity to noise
- Unstable ROS node frequencies
- Limited scalability (≤2 robots)

---

## Methodology & System Design

### High-Level Architecture

Our work consists of extending the standard DuckieTown autonomy stack. This stack is a robust intersection-handling pipeline integrated into a finite state machine (FSM):

1. **Lane Following**
2. **Stop Line Detection**
3. **Intersection Type Identification**
4. **Trajectory Generation**
5. **Intersection Control**
6. **Return to Lane Following**

**TODO:** add figure as in presentation. 


---

### Stop Line Detection

**Goal:** Detect red stop lines and estimate the robot’s pose relative to them.

**Inputs:**
- Red line segments from vision
- Lane pose estimate `(d, φ)` from the lane filter

**Methodology:**
- Reject red segments outside lane boundaries
- Transform segment endpoints into the lane frame
- Aggregate segment distances to estimate stop line position
- Require a minimum number of consistent detections

To improve robustness, we apply:
- Median smoothing over recent distance estimates
- Hysteresis with separate enter/exit thresholds
- Multi-frame confirmation to prevent flickering states

This significantly stabilizes the `at_stop_line` condition and avoids false exits :contentReference[oaicite:3]{index=3}.

---

### Adaptive Stop & Intersection Control

Upon approaching a stop line, the robot gradually reduces its velocity using a distance-based decay factor, ensuring enough spatial and temporal margin to plan the intersection crossing.

Once stopped, the controller switches to a waypoint-based trajectory follower. Linear and angular velocities are computed using proportional control laws in the robot frame:

- Linear velocity aligns the robot with the waypoint direction
- Angular velocity corrects heading error toward the target

This decoupled control scheme allows smooth execution of complex intersection maneuvers :contentReference[oaicite:4]{index=4}.

---

### Trajectory Generation

A key challenge was ensuring that generated trajectories respect lane geometry, especially for left turns.

The robot occasionally planned paths that drifted into the opposite lane.

A practical solution we implemented was to introduce a via point into the trajectory planning process. This via point helps guide the robot further into the intersection before it begins turning, ensuring that the path remains within the correct lane and reducing the chance of drifting into the opposite lane. Additionally, we adjusted the number of waypoints used for each maneuver: left turns are assigned more waypoints for smoothness and precision, while right turns use fewer waypoints, as they require less steering effort.

![Generated intersection trajectory](src/_images/generated-trajectory.jpeg)

---

### Multi-Frame Trajectory Handling

A critical challenge in intersection navigation was managing multiple coordinate frames correctly. Originally, waypoints were computed in the robot frame relative to the stop line, but this approach caused significant drift during execution as the robot moved.

When waypoints are computed in the robot's current frame and the robot moves, those waypoints become stale. The robot would either skip waypoints, stop prematurely, or drift off the intended path.

We implemented a multi-frame approach that separates planning from execution:

In the planning frame, also called the stop-line frame, waypoints are generated relative to the detected stop line pose. Using the stop line as a reference provides a stable, world-fixed coordinate system for planning. Goal poses for the robot are blended between canonical targets and those adjusted by the stop line pose to help reduce over-correction. For left turns, a via-point is additionally inserted so that the robot enters further into the intersection before beginning its turn, encouraging safer and more accurate maneuvering.

When planning occurs, the robot's current pose is used to transform the planned waypoints from the robot frame into the odometry frame. This transformation effectively "freezes" the waypoints at the planning moment, such that each odometry-frame waypoint is computed as `waypoint_odom = odom_T_robot × waypoint_robot`. During execution, the robot tracks its pose in the odometry frame using dead reckoning from wheel encoders. Because waypoint checking also occurs in the odometry frame, this approach prevents drift that could otherwise result from moving frames of reference.


**Benefits:**
Waypoints remain fixed in the global odometry frame, eliminating the accumulation of frame transformation errors as the robot moves. This provides consistent and reliable waypoint referencing throughout the maneuver. The waypoint reaching logic is made robust by decomposing the robot’s position error into along-track and cross-track components, which helps prevent skipping waypoints prematurely and avoids oscillatory behavior during trajectory execution.

---

### AprilTag-Based Alignment

To improve intersection entry consistency, we developed an AprilTag alignment system that orients the robot perpendicular to the stop line before trajectory execution. Since it is difficult to know the real heading of the robot in relation with the stop line, this is a way to minimize this.

When approaching the intersection, the robot first detects the AprilTag on the intersection sign and determines its 3D pose. The system then calculates the required coordinate transformation from the camera's optical frame to the robot's base frame. To achieve proper alignment, the robot adjusts its heading to face directly toward the tag, effectively orienting itself parallel to the tag’s z-axis and ensuring it is perpendicular to the stop line. A configurable heading offset—currently set to +0.23 radians (about 13°)—is applied to fine-tune the final alignment.

When the target AprilTag is not initially visible, the robot performs a systematic scan:
1. Rotates left (~23°) and checks for tag
2. Returns to center and checks
3. Rotates right (~23°) and checks
4. Returns to center for final alignment

**Parameters:**
- `align_tag_tolerance`: 0.15 rad (~8.6°) alignment accuracy
- `align_tag_k`: 1.5 proportional gain for rotation control
- `align_tag_omega_max`: 2.0 rad/s maximum rotation speed
- `align_tag_max_time`: 2.5s timeout per alignment attempt

This pre-alignment improves trajectory following accuracy and reduces the robot's tendency to drift into opposing lanes during turns.

---

### LED Communication Protocol

For future multi-robot scenarios, we implemented a 5-LED communication protocol that signals robot priority and intended direction at intersections.

**LED Array Structure:** `[Front Left, Rear Left, Top, Rear Right, Front Right]`

**Active LEDs:**
- **Front Left (Index 0):** Priority level (1-4) encoded as color
  - Priority 1: Red
  - Priority 2: Blue
  - Priority 3: Purple
  - Priority 4: White
- **Front Right (Index 4):** Direction or ready state
  - Left turn: Cyan
  - Straight: Yellow
  - Right turn: Pink
  - Ready to enter: Green

**State Sequence:**
1. **At stop line:** Display priority + direction colors
2. **Planning complete:** Front right LED turns green (ready signal)
3. **Executing turn:** Front right returns to direction color
4. **Intersection complete:** Full green pattern

**FSM Integration:**
The LED system is integrated with the finite state machine, with custom patterns allowed during `STOP_SIGN_INTERSECTION` and `INTERSECTION_CONTROL` states. A 0.5s re-application timer ensures patterns persist despite FSM state transitions.

While not yet used for actual coordination, this protocol provides a foundation for future decentralized multi-robot intersection management.

---

## Lessons Learned

Throughout development, it became clear that fine-tuning the system on real hardware is far more challenging than in simulation. The project also revealed that Finite State Machine (FSM) architectures can be fragile unless transitions are managed with care and precision. Even subtle timing inconsistencies in ROS nodes have the potential to propagate and lead to significant failures across the system. Perhaps most importantly, consistently using the correct reference frames—such as distinguishing between robot and odometry frames—is absolutely essential to ensure reliable navigation.

---

## Future Work

Potential extensions include:
- True multi-robot indefinite navigation experiments
- Integration of spatio-temporal intersection protocols (e.g., STIP-inspired planning)

---

## References

1. **Dresner, K., & Stone, P. (2008).** "A multiagent approach to autonomous intersection management." *Journal of Artificial Intelligence Research*, 31, 591-656.
   - Proposed reservation-based multi-agent intersection management where vehicles negotiate access through request/confirm protocols

2. **Azimi, R., Bhatia, G., Rajkumar, R. R., & Mudalige, P. (2013).** "Reliable intersection protocols using vehicular networks." *Proceedings of the ACM/IEEE 4th International Conference on Cyber-Physical Systems*, 1-10.
   - Introduced distributed V2V protocols with deadlock avoidance, achieving significant throughput improvements under realistic wireless conditions

3. **Paull, L., Tani, J., Ahn, H., Alonso-Mora, J., Carlone, L., Cap, M., ... & Rus, D. (2017).** "Duckietown: An open, inexpensive and flexible platform for autonomy education and research." *2017 IEEE International Conference on Robotics and Automation (ICRA)*, 1497-1504.
   - Core Duckietown platform architecture and autonomy stack

4. **Censi, A., Paull, L., Tani, J., & Frazzoli, E. (2019).** "The Duckietown Project: An Open Platform for Research and Education in Autonomy." *In The Future of Transportation*, MIT Press.
   - Educational framework and system design principles for autonomous vehicle research

---
