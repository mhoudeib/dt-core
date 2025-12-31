# Navigation Robustified for DuckieTown 🚗♾️  
> Achieving indefinite autonomous driving in DuckieTown

<p align="center">
<a href="https://duckietown.com">
<img src="assets/images/dtlogo.png" alt="Duckietown Logo" width="50%">
</a>
</p>

**Course:** IFT6757 – Autonomous Vehicles  
**Institution:** Université de Montréal  
**Date:** December 2025  

**Authors:**  
- Firmin Chapoulie  
- Guillaume Genois  
- Mohamad Houdeib  

<p align="center">
  <video src="assets/demo.webm" width="70%" autoplay loop muted playsinline>
    Your browser does not support the video tag.
  </video>
</p>

<!-- <p align="center">
  <img src="assets/images/overview.png" alt="Project Overview Diagram" width="80%">
</p> -->

---

## Usage

```bash
dts matrix run --standalone -m assets/duckiematrix/maps/intersections
dts matrix attach [YOURBOTNAME] map_0/vehicle_0
dts devel build -H [YOURBOTNAME]
dts devel run -H [YOURBOTNAME] -M -L single_robot_indefinite_navigation
dts duckiebot keyboard_control [YOURBOTNAME]
```
After this, click the autopilot toggle in the keyboard_control window; this will launch the autonomous drive. Wait until the light pattern changed to GREEN.

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

The objective of this project is to enable a Duckiebot to drive indefinitely in a DuckieTown environment with intersections without traffic lights, by robustly detecting stop lines, interpreting intersection signs, planning trajectories, and safely transitioning back to lane following.

Concretely, we focus on the LF-I_noTL scenario (single robot, no traffic lights), which requires:
- Reliable stop line detection
- Correct identification of intersection type
- Safe and stable intersection crossing
- Robust state switching to avoid deadlocks or oscillations

![System Behaviors Hierarchy](assets/images/auto-behaviors-hierarchy.png)

This work serves as a foundation toward multi-robot indefinite navigation (LF-IV), where decentralized coordination becomes necessary.

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

<img src="assets/images/architecture.png" alt="High-Level Architecture" width="800"/>


---

### Stop Line Detection

The goal of the stop line detection module is to identify red stop lines on the road and estimate the robot’s position relative to them. The system receives as input a set of red line segments detected by the vision pipeline, along with the lane pose estimate `(d, φ)` provided by the lane filter. The detection process begins by filtering out any red segments that fall outside the boundaries of the lane. The remaining segment endpoints are then transformed into the lane frame of reference, allowing for a more accurate spatial interpretation. By aggregating the distances of these valid segments, the system estimates the position of the stop line. To ensure reliable detection, it requires a minimum number of consistent observations before confirming the presence of a stop line.

To enhance the robustness of detection, several techniques are applied. First, median smoothing is performed over recent distance estimates to reduce the influence of outliers. The system also implements hysteresis, maintaining separate thresholds for entering and exiting the stop line state to prevent rapid switching caused by noise or transient errors. Finally, the detection must be confirmed across multiple frames, which helps avoid false positives and eliminates flickering between states. These combined strategies lead to a much more stable and reliable `at_stop_line` condition for state transitions, effectively preventing premature or incorrect state changes.

---

### Adaptive Stop & Intersection Control

Upon approaching a stop line, the robot gradually reduces its velocity using a distance-based decay factor, ensuring enough spatial and temporal margin to plan the intersection crossing.

The decrease is linear in the transition area with a null final speed to stop the vehicle.
The speed decrease factor is given by:

$$
f_{dec}(x) = {{x_{end}-x}\over{x_{end}-x_{start}}}
$$

Where $x_{end}$ & $x_{start}$ define the transition area.

Once stopped, the controller switches to a waypoint-based trajectory follower. Linear and angular velocities are computed using proportional control laws in the robot frame:

- Linear velocity aligns the robot with the waypoint direction
- Angular velocity corrects heading error toward the target

The navigation control command are given by:

$$
\nu = k_{\rho}((x_{w} - x)cos(\theta)-(y_{w} - y)sin(\theta))
$$
$$
\omega = k_{\alpha}[atan2((y_{w} - y), (x_{w} - x))-\theta]
$$

This control law required the tuning of two gains to work properly in real conditions.

This decoupled control scheme allows smooth execution of complex intersection maneuvers.

---

### Trajectory Generation

A key challenge was ensuring that generated trajectories respect lane geometry, especially for left turns.

The robot occasionally planned paths that drifted into the opposite lane.

A practical solution we implemented was to introduce a via point into the trajectory planning process. This via point helps guide the robot further into the intersection before it begins turning, ensuring that the path remains within the correct lane and reducing the chance of drifting into the opposite lane. Additionally, we adjusted the number of waypoints used for each maneuver: left turns are assigned more waypoints for smoothness and precision, while right turns use fewer waypoints, as they require less steering effort.

![Generated intersection trajectory](assets/images/trajectory.jpeg)

---

### Multi-Frame Trajectory Handling

A critical challenge in intersection navigation was managing multiple coordinate frames correctly. The implementation uses a stop-line-relative planning approach where waypoints are generated in the robot's coordinate frame at planning time, using the stop line pose as a stable reference.

The trajectory planning process works as follows:

**Planning Phase:**
1. The stop line pose is detected relative to the robot's current position. This pose is stored with theta set to 0.0 (assuming the robot is aligned with the stop line).
2. Canonical goal poses are defined in the stop-line coordinate frame for left, right, and straight maneuvers.
3. The goal pose is transformed from the stop-line frame to the robot frame using `robot_frame_goal_pose = inverse(g_stop_pose) × canonical_goal_pose`, where `g_stop_pose` represents the stop line pose relative to the robot.
4. For left turns with via-point enabled, the trajectory is split into two segments:
   - Segment 1: Interpolates from the stop line to the via-point (both transformed to robot frame using the same transformation)
   - Segment 2: Interpolates from the via-point to the goal pose (computed in stop-line frame, then transformed to robot frame)
5. For other maneuvers, waypoints are generated by interpolating along the trajectory from the stop line to the goal pose in the robot frame, using SE(2) Lie algebra interpolation.
6. All waypoints are clamped to prevent planning behind the stop line (x ≥ 0 in the stop-line frame).

**Execution Phase:**
The waypoints generated during planning are in the robot's coordinate frame at the moment of planning. During execution, the robot tracks its position using dead reckoning from wheel encoders. The waypoints are compared directly with the robot's odometry coordinates, effectively using the robot's planning-time frame as the execution reference frame.

**Benefits:**
Using the stop line as a reference point provides a stable frame for planning that adapts to the robot's position at the intersection. The waypoint reaching logic is made robust by decomposing the robot's position error into along-track and cross-track components, which helps prevent skipping waypoints prematurely and avoids oscillatory behavior during trajectory execution.

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

### Robust AprilTag Detection and Filtering

A critical challenge in intersection navigation is reliably identifying the correct AprilTag among multiple detections, especially when tags may be visible from oblique angles, at the edge of the camera's field of view, or at varying distances. To address this, we implemented a multi-criteria filtering system that selects only the most relevant tag for intersection identification.

The system evaluates each detected AprilTag against three spatial constraints:

1. **Perpendicularity Check**: The tag's surface normal must be roughly facing the camera, indicating the robot is approaching the intersection head-on rather than from an angle. The system extracts the tag's orientation quaternion, converts it to a rotation matrix, and computes the dot product between the tag's normal vector and the camera's optical axis. Tags are rejected if the angle exceeds configurable thresholds (typically ±45° from perpendicular), corresponding to `|dot_product| < 0.707`. This prevents misidentification when the robot glimpses tags from adjacent lanes or past intersections.

2. **Horizontal Viewing Angle**: To ensure the robot only considers tags directly ahead (not those to the left or already passed), the system calculates the horizontal angle using `arctan2(pos.y, pos.x)` where `pos.x` is forward distance and `pos.y` is lateral offset. Tags appearing on the left side of the camera view (negative horizontal angles below a threshold) are filtered out. This criterion is robust regardless of the robot's exact position in the lane and prevents the robot from reacting to intersection signs it has already passed.

3. **Distance Bounds**: Tags outside a configurable maximum distance are ignored to avoid readings of distant intersections. Among all tags passing the above filters, the system selects the **closest** one, ensuring the robot responds to the immediate intersection rather than one further ahead.

Only tags satisfying all geometric constraints are considered, and the nearest valid tag is selected for intersection type determination. 

---

### LED Communication Protocol

For future multi-robot scenarios, we implemented a 5-LED communication protocol that signals robot priority and intended direction at intersections. Our protocol uses color charts to overcome the instability frequency flickering problem previously encountered in Duckietown.

Our short range vehicle to vehicle communication protocol is based on SAE's J2735 standard basic safety messages. These messages aim to provide minimal information to plan intersection crossings with multiple vehicles for the Spatio-Temporal Intersection Protocols approach [oaicite:5]{index=5}.

**LED Array Structure:** `[Front Left, Rear Left, Top, Rear Right, Front Right]`

- **Front Left (LED 0):** Priority level (1-4) encoded as color

| Level | Color | Priority | 
| :--- | :--- | :--- | 
| 1 | 🟥 | 1 (highest) | 
| 2 | 🟦 | 2 | 
| 3 | 🟪 | 3 | 
| 4 | ⬜ | 4 (lowest) |

- **Front Right (LED 4):** Direction or ready state

| Index | Color | Planned Trajectory |
| :--- | :--- | :--- |
| 0 | Cyan | Turning Left |
| 1 | 🟨 Yellow | Going Straight |
| 2 | Pink | Turning Right |

- **State Sequence:**

| Phase	| LED 0 (Front Left)| LED 4 (Front Right)| Meaning |
| :--- | :--- | :--- | :--- |
| Approach / Stop |	Priority Color |	Direction Color	|Intent broadcasting (Negotiation) |
| Validation (Ready) | Priority Color | 🟩 | Scenario calculated, ready to move |
| Crossing | 🟨 | 🟨 | Occupying the intersection (Warning) |
| Exit | 🟩 | 🟩 | Intersection cleared |
| Idle | 🔲 | 🔲 | Normal driving / No conflict |

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

5. **R. Azimi, G. Bhatia, R. R. Rajkumar and P. Mudalige**, "STIP: Spatio-temporal intersection protocols for autonomous vehicles," 2014 ACM/IEEE International Conference on Cyber-Physical Systems (ICCPS), Berlin, Germany, 2014, pp. 1-12, doi: 10.1109/ICCPS.2014.6843706.

---
