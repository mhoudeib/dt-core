# Indefinite Navigation System - How It Works

This document explains how the Duckietown indefinite navigation system works, enabling a Duckiebot to drive continuously through intersections by detecting stop lines, determining intersection types, choosing paths, and executing turns.

## Overview

Indefinite navigation allows the Duckiebot to drive continuously through a Duckietown environment, navigating intersections by:
1. Detecting stop lines
2. Determining intersection type (stop sign vs traffic light)
3. Choosing a turn direction based on AprilTags
4. Executing the turn using dead reckoning
5. Returning to lane following

## The Complete Flow

### 1. FSM State Machine

**Location**: `packages/fsm/config/fsm_node/single_robot_indefinite_navigation.yaml`

The FSM (Finite State Machine) orchestrates the entire process with these key states:

```
LANE_FOLLOWING
  → DETECT_INTERSECTION_TYPE
  → {STOP_SIGN_INTERSECTION | TRAFFIC_LIGHT_INTERSECTION}
  → INTERSECTION_CONTROL
  → LANE_FOLLOWING
```

#### State Breakdown:

**LANE_FOLLOWING** (line 77)
- **Purpose**: Normal driving using lane controller
- **Active nodes**:
  - `line_detector_node` - Detects lane lines
  - `lane_filter_node` - Estimates lane pose
  - `lane_controller_node` - Controls steering
  - `stop_line_filter_node` - Watches for stop lines
  - `unicorn_intersection_node` - Monitors for intersection events
- **Transition**: When `at_stop_line` event triggers → moves to `DETECT_INTERSECTION_TYPE`
- **LED Color**: RED

**DETECT_INTERSECTION_TYPE** (line 90)
- **Purpose**: Figures out what kind of intersection this is
- **Active nodes**:
  - `apriltag_detector_node` - Detects AprilTags
  - `apriltag_postprocessing_node` - Processes tag data
  - `intersection_type_detector_node` - Determines intersection type
  - `unicorn_intersection_node` - Continues monitoring
- **Transitions**:
  - If `stop_sign_detected` → `STOP_SIGN_INTERSECTION`
  - If `traffic_light_detected` → `TRAFFIC_LIGHT_INTERSECTION`
- **LED Color**: YELLOW

**STOP_SIGN_INTERSECTION** (line 102)
- **Purpose**: Handle stop sign coordination (single or multi-robot)
- **Active nodes**:
  - `apriltag_detector_node`
  - `apriltag_postprocessing_node`
  - `random_april_tag_turns_node` - Chooses which way to turn
  - `unicorn_intersection_node` - Prepares for navigation
- **Transition**: When `stop_sign_intersection_go` → `INTERSECTION_CONTROL`
- **LED Color**: WHITE

**TRAFFIC_LIGHT_INTERSECTION** (line 113)
- **Purpose**: Handle traffic light coordination
- **Active nodes**:
  - `apriltag_detector_node`
  - `lane_filter_node`
  - `lane_controller_node`
  - `led_detector_node` - Detects traffic light color
  - `coordinator_node` - Handles coordination logic
- **Transition**: When `traffic_light_intersection_go` → `INTERSECTION_CONTROL`
- **LED Color**: WHITE

**INTERSECTION_CONTROL** (line 125)
- **Purpose**: Execute the turn through the intersection
- **Active nodes**:
  - `line_detector_node`
  - `lane_filter_node`
  - `lane_controller_node`
  - `unicorn_intersection_node` - Actively navigating
  - `random_april_tag_turns_node` - Provides turn direction
- **Transition**: When `intersection_done` → back to `LANE_FOLLOWING`
- **LED Color**: (controlled by navigation logic)

### 2. Stop Line Detection

**Location**: `packages/stop_line_filter/src/stop_line_filter_node.py`

The `stop_line_filter_node` detects red stop lines in the ground-projected line segments:

- **Input**: Receives ground-projected line segments from `ground_projection_node`
- **Processing**:
  - Filters for red-colored line segments
  - Calculates distance from robot to stop line
  - Estimates the pose of the stop line relative to the robot
- **Output**:
  - When robot gets within `stop_distance` threshold (configurable parameter)
  - Publishes `at_stop_line: True` to topic `~at_stop_line` (line 107)
  - Also publishes `StopLineReading` message with `stop_pose` information
- **FSM Trigger**: The `at_stop_line` event triggers FSM transition to `DETECT_INTERSECTION_TYPE`

### 3. Intersection Type Detection

**Location**: `packages/navigation/src/intersection_type_detector_node.py`

Once at the stop line, the `intersection_type_detector_node` determines what type of intersection this is:

**Process** (cbTag callback, lines 47-79):
1. Receives AprilTag detections from camera
2. Loops through all detected tags (line 52)
3. Filters for tags that are traffic signs (`tag_type == SIGN`)
4. Looks specifically for:
   - `STOP` sign tags (line 54)
   - `T_LIGHT_AHEAD` tags (line 55)
5. Finds the **nearest** relevant tag (lines 58-61)
6. Publishes to appropriate topic:
   - If `STOP` sign → publishes to `~stop_sign_intersection_detected` (lines 71-73)
   - If `T_LIGHT_AHEAD` → publishes to `~traffic_light_intersection_detected` (lines 74-76)

**FSM Integration**: These publications trigger the FSM to transition to either `STOP_SIGN_INTERSECTION` or `TRAFFIC_LIGHT_INTERSECTION` state.

### 4. Turn Direction Selection

**Location**: `packages/navigation/src/random_april_tag_turns_node.py`

The `random_april_tag_turns_node` decides which direction to turn:

**Turn Type Encoding**:
- `0` = Left turn
- `1` = Straight (continue forward)
- `2` = Right turn

**Process** (cbTag callback, lines 37-96):

1. **Find Topology Signs** (lines 43-58):
   - Subscribes to AprilTag detections
   - Filters for topology/navigation signs:
     - `NO_RIGHT_TURN`
     - `LEFT_T_INTERSECT`
     - `NO_LEFT_TURN`
     - `RIGHT_T_INTERSECT`
     - `T_INTERSECTION`
     - `FOUR_WAY`
   - Finds the nearest topology sign

2. **Determine Available Turns** (lines 69-79):
   Based on sign type, creates list of available turns:
   - `NO_RIGHT_TURN` or `LEFT_T_INTERSECT` → [0, 1] (left or straight)
   - `NO_LEFT_TURN` or `RIGHT_T_INTERSECT` → [1, 2] (straight or right)
   - `FOUR_WAY` → [0, 1, 2] (any direction)
   - `T_INTERSECTION` → [0, 2] (left or right, no straight)

3. **Random Selection** (lines 82-91):
   - **Randomly** chooses one turn from available options
   - This is truly random - no pathfinding or goal-seeking!
   - Stores choice in `self.turn_type`

4. **Publish Decisions** (lines 86-96):
   - Publishes `turn_type` (Int16) to `~turn_type`
   - Publishes `TurnIDandType` message with tag ID and turn type
   - Publishes `intersection_go` signal (BoolStamped) to trigger FSM transition

**FSM Integration**: The `intersection_go` publication triggers FSM transition to `INTERSECTION_CONTROL` state.

### 5. Intersection Execution ("Unicorn Intersection")

**Location**: `packages/unicorn_intersection/src/unicorn_intersection_node.py`

The `unicorn_intersection_node` (also called "semi-closed loop intersection navigation") executes the actual turn through the intersection.

#### Setup Phase

**Waiting for Information** (check_if_go method, lines 96-106):
- Maintains internal state machine with state `READY` initially
- Waits for TWO pieces of critical information:
  1. **stop_line_pose** - Robot's position relative to the stop line (from `stop_line_filter_node`)
  2. **turn_type** - Which direction to turn: 0=left, 1=straight, 2=right (from `random_april_tag_turns_node`)
- Only proceeds when BOTH are received AND internal state is `READY`

**Receiving Stop Line Pose** (cbStopLineReading, lines 86-94):
- Receives `StopLineReading` message
- Extracts `stop_pose` (x, y, theta relative to robot)
- Sets `stop_line_pose_received = True`
- Calls `check_if_go()` to see if ready to proceed

**Receiving Turn Type** (cbTurnType, lines 294-301):
- Receives `TurnIDandType` message
- Extracts `turn_type` (0, 1, or 2)
- Sets `turn_type_received = True`
- Calls `check_if_go()` to see if ready to proceed

#### Trajectory Planning Phase

**Goal Pose Configuration** (setupParams, lines 303-312):
- Loads canonical goal poses from ROS parameters:
  - `canonical_goal_pose_left`: Default {x: 0.0, y: 0.0, theta: 0.0}
  - `canonical_goal_pose_right`: Default {x: 0.0, y: 0.0, theta: 0.0}
  - `canonical_goal_pose_straight`: Default {x: 0.0, y: 0.0, theta: 0.0}
- These should be configured in the package config files with actual intersection geometry
- Also loads `num_waypoints` (default 2) and `speed` (default 0.30 m/s)

**Trajectory Calculation** (calculate_goal_trajectory, lines 112-148):

1. **Select Goal Pose** (lines 118-125):
   - Based on `turn_type`, selects appropriate canonical goal pose:
     - turn_type=0 → left goal pose
     - turn_type=1 → straight goal pose
     - turn_type=2 → right goal pose

2. **Transform to Robot Frame** (line 127):
   - Converts stop line pose to geometry library format
   - Calculates goal pose in robot's current reference frame
   - Uses SE2 (Special Euclidean group) transformations:
     ```python
     robot_frame_goal_pose = SE2.multiply(SE2.inverse(g_stop_pose), canonical_goal_pose)
     ```

3. **Generate Waypoints** (lines 133-143):
   - Interpolates along the trajectory to create waypoints
   - Uses SE2 algebra to interpolate smoothly
   - Creates `num_waypoints` evenly spaced waypoints
   - Each waypoint has position (x, y) and direction (theta)
   - Example: With 2 waypoints, creates intermediate point at 50% and goal at 100%

4. **Visualization** (lines 146-165):
   - If visualization enabled, publishes waypoints as Odometry messages
   - Can be viewed in RViz for debugging

5. **State Transition**:
   - Sets `internal_state = "EXECUTING"`
   - Now ready to navigate through intersection

#### Execution Phase (Dead Reckoning Navigation)

**Odometry Setup** (reset_odometry, lines 167-184):
- Initializes dead reckoning state:
  - Position: x=0, y=0, z=0
  - Orientation: yaw=0
  - Velocities: tv=0 (translational), rv=0 (rotational)
  - Encoder state tracking
- Parameters:
  - `ticks_per_meter = 656.0` (encoder resolution)
  - `wheelbase = 0.108` (m) (distance between wheels)

**Encoder Callback** (cb_ts_encoders, lines 186-282):

This is the main control loop, called whenever synchronized wheel encoder messages arrive:

1. **Skip if Not Executing** (lines 187-188):
   - Only runs when `internal_state == "EXECUTING"`
   - Ignores encoder data during other phases

2. **Calculate Odometry** (lines 190-258):
   - Gets encoder tick counts from left and right wheels
   - Calculates distance traveled by each wheel:
     ```python
     left_distance = left_dticks / ticks_per_meter
     right_distance = right_dticks / ticks_per_meter
     ```
   - Computes forward motion:
     ```python
     distance = (left_distance + right_distance) / 2
     ```
   - Computes rotation (differential drive):
     ```python
     dyaw = (right_distance - left_distance) / wheelbase
     ```
   - Updates robot pose:
     ```python
     yaw = yaw + dyaw
     x = x + distance * cos(yaw)
     y = y + distance * sin(yaw)
     ```

3. **Compute Steering Command** (lines 260-268):
   - Creates `Twist2DStamped` message
   - Sets forward velocity to constant `self.speed`
   - Computes steering rate (omega) using `compute_omega()` (lines 337-342):
     ```python
     target_yaw = arctan2(target_y - y, target_x - x)
     omega = factor * (target_yaw - current_yaw)
     ```
   - This is a simple proportional controller that steers toward the current waypoint
   - Publishes command to `~car_cmd` topic

4. **Check Waypoint Reached** (lines 270-281):
   - Calls `check_point()` to see if current waypoint reached (lines 344-362)
   - Uses distance threshold (0.1m) or x-threshold (0.08m) depending on which waypoint
   - If waypoint reached:
     - Increment waypoint counter: `self.iter_ += 1`
     - If all waypoints completed (`iter_ == num_waypoints`):
       - Set `internal_state = "READY"` (ready for next intersection)
       - Reset flags: `stop_line_pose_received = False`, `turn_type_received = False`
       - Publish `intersection_done` message (BoolStamped, data=True)
       - Reset odometry to zero
       - Log completion message

**FSM Integration**: The `intersection_done` publication triggers FSM transition back to `LANE_FOLLOWING` state.

### 6. Return to Lane Following

When `unicorn_intersection_node` publishes `intersection_done`:
- FSM transitions from `INTERSECTION_CONTROL` back to `LANE_FOLLOWING`
- Lane following nodes reactivate
- Robot resumes normal lane following behavior
- System is ready to detect the next stop line
- The cycle repeats indefinitely

## Key Code Locations

| Component | File Path |
|-----------|-----------|
| FSM Configuration | `packages/fsm/config/fsm_node/single_robot_indefinite_navigation.yaml` |
| Stop Detection | `packages/stop_line_filter/src/stop_line_filter_node.py` |
| Intersection Type | `packages/navigation/src/intersection_type_detector_node.py` |
| Turn Selection | `packages/navigation/src/random_april_tag_turns_node.py` |
| Turn Execution | `packages/unicorn_intersection/src/unicorn_intersection_node.py` |
| Launch Config | `packages/duckietown_demos/launch/indefinite_navigation.launch` |
| Launcher Script | `launchers/indefinite_navigation.sh` |

## Topic Communication Flow

```
stop_line_filter_node/at_stop_line
  → [FSM] → DETECT_INTERSECTION_TYPE

apriltag_detector_node/detections
  → intersection_type_detector_node/stop_sign_intersection_detected
  → [FSM] → STOP_SIGN_INTERSECTION

apriltag_detector_node/detections
  → random_april_tag_turns_node/tag
  → random_april_tag_turns_node/turn_type
  → unicorn_intersection_node/turn_id_and_type

random_april_tag_turns_node/intersection_go
  → [FSM] → INTERSECTION_CONTROL

stop_line_filter_node/stop_line_reading
  → unicorn_intersection_node/stop_line_reading

wheel_encoder_driver_node/tick
  → unicorn_intersection_node
  → unicorn_intersection_node/car_cmd
  → wheels_driver_node

unicorn_intersection_node/intersection_done
  → [FSM] → LANE_FOLLOWING
```

## Important Notes

### 1. Random Navigation
The robot chooses turns **randomly** from available options - it's not pathfinding to a goal! Each intersection is handled independently without considering previous turns or future destinations.

### 2. Dead Reckoning During Turns
The intersection navigation uses **only wheel encoders** (dead reckoning), not vision feedback. This means:
- No camera/line detection during the turn
- Odometry errors accumulate (wheel slip, uneven floors)
- Depends on accurate calibration of `ticks_per_meter` and `wheelbase`
- Works well for short intersection crossings but would drift over longer distances

### 3. Open-Loop Control
While the heading is corrected by steering toward waypoints (closed-loop in heading), the position is pure dead reckoning (open-loop in position). The robot trusts its wheel encoder odometry completely.

### 4. Coordination Options
The `indefinite_navigation.launch` enables explicit coordination (line 49), which handles multi-robot scenarios at intersections:
- Multiple robots can coordinate at stop signs
- Traffic light intersections use LED-based signaling
- Coordination logic is in separate nodes (`coordinator_node`, `communication_node`)

### 5. Configuration Requirements
For proper operation, the canonical goal poses must be configured correctly in:
- `packages/unicorn_intersection/config/unicorn_intersection_node/default.yaml`

These poses define the intersection geometry and must match the physical Duckietown layout.

### 6. FSM Control
All nodes involved are `fsm_controlled=True`, meaning they can be activated/deactivated by the FSM. This conserves computational resources and ensures clean state transitions.

## Configuration Parameters

Key parameters that affect indefinite navigation behavior:

**stop_line_filter_node**:
- `stop_distance`: Distance threshold for triggering `at_stop_line`

**unicorn_intersection_node**:
- `canonical_goal_pose_left`: {x, y, theta} for left turns
- `canonical_goal_pose_right`: {x, y, theta} for right turns
- `canonical_goal_pose_straight`: {x, y, theta} for straight
- `num_waypoints`: Number of intermediate waypoints (default: 2)
- `speed`: Forward speed during intersection (default: 0.30 m/s)
- `use_stop_pose`: Whether to use stop line pose (default: False)
- `visualization`: Enable trajectory visualization (default: True)

**Dead reckoning parameters** (hardcoded in unicorn_intersection_node.py):
- `ticks_per_meter`: 656.0
- `wheelbase`: 0.108 m

## Debugging and Visualization

To debug indefinite navigation:

1. **Monitor FSM state**:
   ```bash
   rostopic echo /ROBOTNAME/fsm_node/mode
   ```

2. **Check stop line detection**:
   ```bash
   rostopic echo /ROBOTNAME/stop_line_filter_node/at_stop_line
   ```

3. **View turn decisions**:
   ```bash
   rostopic echo /ROBOTNAME/random_april_tag_turns_node/turn_type
   ```

4. **Monitor intersection progress**:
   ```bash
   rostopic echo /ROBOTNAME/unicorn_intersection_node/intersection_done
   ```

5. **Visualize trajectory in RViz**:
   - Subscribe to `/ROBOTNAME/unicorn_intersection_node/reference_trajectory`
   - View Odometry messages as poses

## Potential Issues and Limitations

1. **Odometry Drift**: Dead reckoning accumulates errors, especially with wheel slip
2. **Calibration Sensitivity**: Requires accurate `ticks_per_meter` and `wheelbase` values
3. **No Obstacle Avoidance**: During intersection execution, robot follows trajectory blindly
4. **Fixed Speed**: Speed is constant during intersection, no acceleration profile
5. **Waypoint Thresholds**: May miss waypoints if thresholds too tight or overshoot if too loose
6. **AprilTag Detection**: Depends on reliable AprilTag detection at intersections
7. **Stop Line Detection**: Red line detection can be affected by lighting conditions

## Future Improvements

Possible enhancements to the system:
- Visual servoing during turns (use lane lines as feedback)
- IMU integration for better heading estimation
- Adaptive speed control based on trajectory curvature
- More sophisticated path planning (e.g., smooth curves instead of waypoint interpolation)
- Obstacle detection during intersection traversal
- Goal-directed navigation instead of random turns
