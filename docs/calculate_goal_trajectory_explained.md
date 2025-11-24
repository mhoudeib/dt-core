# Mathematical Explanation of `calculate_goal_trajectory` Function

## Overview

This function calculates a trajectory of waypoints that the robot should follow to navigate through an intersection. It uses SE(2) geometry (Special Euclidean group in 2D) to properly handle poses and transformations.

**Location**: [packages/unicorn_intersection/src/unicorn_intersection_node.py:112-148](../packages/unicorn_intersection/src/unicorn_intersection_node.py#L112-L148)

## Input Data

Before this function is called, the node has received:

1. **`self.stop_line_pose`** (Pose2D): The stop line's position relative to the robot
   - `x`: Distance forward to stop line (meters)
   - `y`: Lateral offset to stop line (meters)
   - `theta`: Angular offset to stop line (radians)

2. **`self.turn_type`** (int): Direction to turn
   - `0` = Left turn
   - `1` = Straight
   - `2` = Right turn

3. **`self.goal_poses`** (dict): Canonical goal poses relative to the stop line
   - Loaded from configuration file
   - Pre-converted to SE(2) format in `__init__`

## Step-by-Step Mathematical Breakdown

### Step 0: Convert Stop Line Pose to SE(2) Format

**Code** (line 113):
```python
g_stop_pose = self.ros_pose_to_geometry(self.stop_line_pose)
```

**Mathematical Operation**:

Convert ROS Pose2D `(x, y, θ)` to SE(2) group element (3×3 matrix):

```
g_stop_pose = T_robot→stop = [cos(θ)  -sin(θ)   x]
                               [sin(θ)   cos(θ)   y]
                               [  0        0      1]
```

**Physical Meaning**:
- This transformation matrix represents the stop line's pose **as seen from the robot's perspective**
- If you apply this transformation to a point in the stop line's coordinate frame, you get its position in the robot's coordinate frame

**Example**:
If `stop_line_pose = (x=0.5, y=0.02, θ=0.1)`:
```
g_stop_pose = [cos(0.1)  -sin(0.1)   0.5  ]   [0.995  -0.100   0.5  ]
               [sin(0.1)   cos(0.1)   0.02 ] = [0.100   0.995   0.02 ]
               [   0          0        1   ]   [  0       0      1   ]
```

This means:
- The stop line is 0.5m ahead
- The stop line is 0.02m to the left
- The robot is rotated 0.1 radians (~5.7°) relative to the stop line

---

### Step 1: Select Canonical Goal Pose

**Code** (lines 118-125):
```python
if self.turn_type == 0:
    canonical_goal_pose = self.goal_poses['left']
elif self.turn_type == 1:
    canonical_goal_pose = self.goal_poses['straight']
elif self.turn_type == 2:
    canonical_goal_pose = self.goal_poses['right']
```

**Mathematical Representation**:

Each canonical goal pose is an SE(2) element representing the desired final position **relative to the stop line**:

```
T_stop→goal = [cos(θ_goal)  -sin(θ_goal)   x_goal]
               [sin(θ_goal)   cos(θ_goal)   y_goal]
               [     0             0           1   ]
```

**From Configuration** (default.yaml):

**Left Turn**:
```
x_goal = 0.5 m
y_goal = 0.5 m
θ_goal = 1.57 rad (90°)

T_stop→goal_left = [cos(1.57)  -sin(1.57)   0.5]   [ 0  -1   0.5]
                    [sin(1.57)   cos(1.57)   0.5] = [ 1   0   0.5]
                    [    0           0        1 ]   [ 0   0    1 ]
```

**Right Turn**:
```
x_goal = 0.3 m
y_goal = -0.3 m
θ_goal = -1.57 rad (-90°)

T_stop→goal_right = [cos(-1.57)  -sin(-1.57)   0.3 ]   [ 0   1   0.3 ]
                     [sin(-1.57)   cos(-1.57)  -0.3 ] = [-1   0  -0.3 ]
                     [     0            0         1  ]   [ 0   0    1  ]
```

**Straight**:
```
x_goal = 0.6 m
y_goal = 0.0 m
θ_goal = 0.0 rad (0°)

T_stop→goal_straight = [cos(0)  -sin(0)   0.6]   [1   0   0.6]
                        [sin(0)   cos(0)   0.0] = [0   1   0.0]
                        [  0        0       1 ]   [0   0    1 ]
```

**Physical Meaning**:
- These poses describe where the robot should end up **relative to where the stop line is**
- For example, "left turn" means: move 0.5m forward from stop line, 0.5m to the left, and rotate 90° counterclockwise

---

### Step 2: Transform Goal Pose to Robot Frame

**Code** (line 127):
```python
robot_frame_goal_pose = g.SE2.multiply(g.SE2.inverse(g_stop_pose), canonical_goal_pose)
```

**Mathematical Operation**:

```
T_robot→goal = T_robot→stop^(-1) ⊗ T_stop→goal
```

Where `⊗` represents SE(2) group multiplication (matrix multiplication).

**Breaking It Down**:

1. **Inverse Transformation** `T_robot→stop^(-1)`:

   Given:
   ```
   T_robot→stop = [R   t]
                   [0   1]
   ```

   The inverse is:
   ```
   T_robot→stop^(-1) = T_stop→robot = [R^T   -R^T·t]
                                        [ 0       1   ]
   ```

   Where `R^T` is the transpose of rotation matrix R.

   For a 2D rotation matrix:
   ```
   R = [cos(θ)  -sin(θ)]
       [sin(θ)   cos(θ)]

   R^T = [cos(θ)   sin(θ)]  = [cos(-θ)  -sin(-θ)]
         [-sin(θ)  cos(θ)]    [sin(-θ)   cos(-θ)]
   ```

2. **Matrix Multiplication** `T_stop→robot ⊗ T_stop→goal`:

   ```
   T_robot→goal = [R_1   t_1] ⊗ [R_2   t_2]
                   [ 0     1 ]   [ 0     1 ]

                = [R_1·R_2   R_1·t_2 + t_1]
                  [   0            1      ]
   ```

**Concrete Example**:

Suppose:
- Robot sees stop line at: `(x=0.5, y=0.02, θ=0.1)` → `T_robot→stop`
- Want left turn: `(x=0.5, y=0.5, θ=1.57)` → `T_stop→goal`

Step 1: Compute inverse
```
T_robot→stop = [0.995  -0.100   0.5 ]
                [0.100   0.995   0.02]
                [ 0       0       1  ]

T_stop→robot = [0.995   0.100  -0.497]  (R^T and -R^T·t)
                [-0.100  0.995  -0.030]
                [ 0       0       1   ]
```

Step 2: Multiply
```
T_robot→goal = T_stop→robot ⊗ T_stop→goal

             = [0.995   0.100  -0.497] ⊗ [ 0  -1   0.5]
               [-0.100  0.995  -0.030]   [ 1   0   0.5]
               [ 0       0       1   ]   [ 0   0    1 ]

             = [0.1    -0.995   0.450]  (approximately)
               [0.995   0.100   0.420]
               [ 0       0        1  ]
```

Extracting (x, y, θ):
```
x ≈ 0.450 m
y ≈ 0.420 m
θ ≈ 1.47 rad (84°)
```

**Physical Meaning**:
- From the robot's **current position**, it needs to move:
  - 0.450m forward
  - 0.420m to the left
  - Rotate 84° counterclockwise
- This accounts for the robot not being perfectly centered at the stop line

**Why This Matters**:
- If the robot is offset to the right (negative y), the goal will be adjusted leftward
- If the robot is rotated, the goal direction will be adjusted
- This transformation ensures the goal is correct regardless of robot's pose at stop line

---

### Step 3: Extract and Display Goal (Debug)

**Code** (lines 129-130):
```python
p, d = g.translation_angle_from_SE2(robot_frame_goal_pose)
print(f"goal_pose in robot frame: position {p}, angle {d}")
```

**Mathematical Operation**:

From SE(2) matrix:
```
T = [R   t]
    [0   1]
```

Extract:
- **Position**: `p = t = [x, y]` (translation vector)
- **Angle**: `d = atan2(R[1,0], R[0,0])` (extract angle from rotation matrix)

For rotation matrix:
```
R = [cos(θ)  -sin(θ)]
    [sin(θ)   cos(θ)]
```

The angle is:
```
θ = atan2(sin(θ), cos(θ)) = atan2(R[1,0], R[0,0])
```

**Output Example**:
```
goal_pose in robot frame: position [0.450, 0.420], angle 1.47
```

---

### Step 4: Convert Goal Pose to Lie Algebra (Velocity)

**Code** (line 133):
```python
vel = g.SE2.algebra_from_group(robot_frame_goal_pose)
```

**Mathematical Operation**: Logarithmic map (group → algebra)

The Lie algebra se(2) represents velocities/twists in the tangent space of SE(2).

**For SE(2)**:

Given transformation:
```
T = [R   t]
    [0   1]
```

The logarithm gives:
```
ξ = log(T) ∈ se(2)
```

**2D Case (simplified)**:

For small rotations, the algebra element can be represented as a 3-vector:
```
ξ = [v_x, v_y, ω]
```

Where:
- `v_x`: Linear velocity in x (forward)
- `v_y`: Linear velocity in y (lateral)
- `ω`: Angular velocity (rotation rate)

**Or as a 3×3 matrix**:
```
ξ_matrix = [  0   -ω   v_x]
            [  ω    0   v_y]
            [  0    0    0 ]
```

**Exact Formula** (for SE(2)):

If `θ = angle(R)` and `t = [x, y]`:

```
If θ ≠ 0:
    V = (1/θ) * [ sin(θ)      -(1-cos(θ))]
                 [(1-cos(θ))    sin(θ)   ]

    [v_x, v_y]^T = V^(-1) · [x, y]^T
    ω = θ

If θ = 0 (pure translation):
    v_x = x
    v_y = y
    ω = 0
```

**Physical Meaning**:
- The velocity vector `ξ` represents the **direction and magnitude** of motion from origin to goal
- Think of it as: "If I move with constant velocity ξ for time t=1, I'll reach the goal"
- This is a **velocity in the robot's body frame**

**Example**:

For `robot_frame_goal_pose` = move (0.450m, 0.420m) with rotation 1.47 rad:

```
θ = 1.47 rad

V^(-1) ≈ some matrix that accounts for rotation

[v_x, v_y] ≈ [0.6, 0.3]  (rough approximation)
ω = 1.47

So: ξ = [0.6, 0.3, 1.47]
```

This means: "Move forward at 0.6 units/time, left at 0.3 units/time, rotate at 1.47 rad/time"

---

### Step 5: Generate Alpha Values for Interpolation

**Code** (line 134):
```python
alphas = [x/self.num_waypoints for x in range(1, self.num_waypoints+1)]
```

**Mathematical Operation**:

Create evenly spaced values from 0 to 1 (exclusive of 0, inclusive of 1).

**With `num_waypoints = 4`** (default):
```python
x = 1, 2, 3, 4
alphas = [1/4, 2/4, 3/4, 4/4] = [0.25, 0.50, 0.75, 1.00]
```

**Physical Meaning**:
- Each alpha represents percentage of completion along the trajectory
- `α = 0.25` → 25% of the way to goal
- `α = 0.50` → 50% of the way to goal
- `α = 0.75` → 75% of the way to goal
- `α = 1.00` → 100% of the way (at goal)

**Note**: We skip `α = 0` because that would be the starting position (current robot pose)

---

### Step 6: Generate Waypoints via Interpolation

**Code** (lines 137-143):
```python
for alpha in alphas:
    rel = g.SE2.group_from_algebra(vel * alpha)
    inter_pose = g.SE2.multiply(g_stop_pose, rel)
    position, direction = g.translation_angle_from_SE2(inter_pose)
    waypoints.append(position)
    directions.append(direction)
```

This loop runs 4 times (for alphas = [0.25, 0.50, 0.75, 1.00])

#### Step 6a: Scale Velocity by Alpha

**Code**:
```python
vel * alpha
```

**Mathematical Operation**:

Scale the velocity vector by alpha:
```
ξ_α = α · ξ = α · [v_x, v_y, ω] = [α·v_x, α·v_y, α·ω]
```

**Example** (α = 0.25):
```
ξ = [0.6, 0.3, 1.47]
ξ_0.25 = 0.25 · [0.6, 0.3, 1.47] = [0.15, 0.075, 0.3675]
```

**Physical Meaning**:
- This represents "25% of the total velocity"
- If the full motion takes "time 1", this is the velocity for "time 0.25"

#### Step 6b: Convert Scaled Velocity Back to Pose

**Code**:
```python
rel = g.SE2.group_from_algebra(vel * alpha)
```

**Mathematical Operation**: Exponential map (algebra → group)

Convert velocity ξ_α back to a transformation matrix:
```
T_α = exp(ξ_α)
```

**For SE(2)**:

Given algebra element `ξ = [v_x, v_y, ω]`:

```
If ω ≠ 0:
    V = (1/ω) * [ sin(ω)      -(1-cos(ω))]
                [(1-cos(ω))    sin(ω)   ]

    [x, y]^T = V · [v_x, v_y]^T
    θ = ω

    T = [cos(θ)  -sin(θ)   x]
        [sin(θ)   cos(θ)   y]
        [  0        0      1]

If ω = 0:
    T = [1   0   v_x]
        [0   1   v_y]
        [0   0    1 ]
```

**Example** (α = 0.25):
```
ξ_0.25 = [0.15, 0.075, 0.3675]

Applying exponential map:
θ = 0.3675 rad

[x, y] ≈ some calculation ≈ [0.112, 0.105] (rough approximation)

T_0.25 = [cos(0.3675)  -sin(0.3675)   0.112]
          [sin(0.3675)   cos(0.3675)   0.105]
          [     0             0          1  ]
```

**Physical Meaning**:
- `T_0.25` represents the pose 25% of the way along the trajectory
- **In the robot's current coordinate frame**
- This is a **relative pose**: "move from current position to this intermediate position"

#### Step 6c: Transform to Stop Line Frame ⚠️ **POTENTIAL BUG HERE**

**Code**:
```python
inter_pose = g.SE2.multiply(g_stop_pose, rel)
```

**Mathematical Operation**:
```
T_intermediate = T_robot→stop ⊗ T_rel
```

**What This Does**:
1. `T_rel` is a relative transformation in the robot frame (25% to goal)
2. Multiplying by `T_robot→stop` transforms it to... **the stop line frame**?

**Wait, This Seems Wrong!**

Let's trace through what coordinate frame we're in:

- `robot_frame_goal_pose` = goal in **robot frame** ✓
- `vel` = velocity in **robot frame** ✓
- `vel * alpha` = scaled velocity in **robot frame** ✓
- `rel` = intermediate pose in **robot frame** ✓
- `g_stop_pose ⊗ rel` = ???

**Mathematical Analysis**:

```
T_robot→stop ⊗ T_rel_in_robot_frame
```

This doesn't make sense dimensionally! Let me reconsider...

**Actually**, if `rel` is in robot frame, then:
```
g_stop_pose ⊗ rel
```

Would give you the pose in... well, it's composing two transformations, but they're in different frames.

**The Issue**:

The code seems to be treating `rel` as if it's in the stop line's frame, but it was derived from `robot_frame_goal_pose`, which is in the robot's frame.

**What Should Happen**:

Since `rel` is already in the robot frame (it's an interpolation between robot's current pose and the goal in robot frame), we should use:

```python
inter_pose = rel  # Already in robot frame!
```

**OR**, if we want waypoints in world frame for visualization:

```python
# If tracking absolute positions
inter_pose = g.SE2.multiply(robot_world_pose, rel)
```

But currently, the code multiplies by `g_stop_pose`, which transforms the waypoint to a strange mixed coordinate frame.

#### Step 6d: Extract Position and Direction

**Code**:
```python
position, direction = g.translation_angle_from_SE2(inter_pose)
waypoints.append(position)
directions.append(direction)
```

**Mathematical Operation**:

Extract (x, y) position and θ angle from the transformation matrix.

**Example Output**:

For 4 waypoints with left turn:
```
Waypoint 1 (α=0.25): position [0.112, 0.105], angle 0.37
Waypoint 2 (α=0.50): position [0.225, 0.210], angle 0.74
Waypoint 3 (α=0.75): position [0.337, 0.315], angle 1.10
Waypoint 4 (α=1.00): position [0.450, 0.420], angle 1.47
```

**Physical Meaning**:
- These are the (x, y, θ) coordinates the robot should pass through
- The coordinate frame depends on what `inter_pose` represents (see issue above)

---

### Step 7: Visualize Trajectory (Optional)

**Code** (lines 146-147):
```python
if self.visualization:
    self.visualize_trajectory(waypoints, directions)
```

Publishes waypoints as Odometry messages for RViz visualization (not mathematically significant).

---

### Step 8: Return Waypoints

**Code** (line 148):
```python
return waypoints
```

Returns list of 4 positions: `[[x1, y1], [x2, y2], [x3, y3], [x4, y4]]`

---

## Summary of Transformations

### Coordinate Frames Involved

1. **Robot Frame**: Origin at robot's current position
   - X-axis: Forward
   - Y-axis: Left
   - θ=0: Facing forward

2. **Stop Line Frame**: Origin at the stop line
   - X-axis: Perpendicular to stop line (forward into intersection)
   - Y-axis: Along the stop line (to the left)
   - θ=0: Perpendicular to stop line

3. **World Frame**: (Not explicitly used, but could be)
   - Fixed reference frame

### Transformation Chain

```
Step 0:  T_robot→stop               (stop line pose from robot's view)
Step 1:  T_stop→goal                (canonical goal from stop line)
Step 2:  T_robot→goal = T_robot→stop^(-1) ⊗ T_stop→goal
Step 4:  ξ = log(T_robot→goal)      (convert to velocity)
Step 6a: ξ_α = α · ξ                 (scale velocity)
Step 6b: T_rel = exp(ξ_α)            (convert back to pose)
Step 6c: T_intermediate = T_robot→stop ⊗ T_rel  ⚠️ (questionable)
```

---

## The Bug and The Fix

### Current Behavior (Line 139)

```python
inter_pose = g.SE2.multiply(g_stop_pose, rel)
```

**What it does**:
- Takes waypoint in robot frame (`rel`)
- Transforms it by stop line pose
- Results in waypoints in a **mixed/incorrect coordinate frame**

**Problem**:
- When odometry resets to (0,0,0) during execution, the robot starts from origin
- But waypoints were calculated relative to stop_pose, not relative to origin
- This causes the robot to aim for the wrong positions

### Correct Behavior (Line 139 should be)

```python
inter_pose = rel
```

**What it does**:
- Waypoints stay in robot frame (where robot's current position is origin)
- Matches the odometry frame (which starts at origin)
- Robot correctly navigates to waypoints

**Why this works**:
- Odometry starts at (0, 0, 0) when navigation begins
- Waypoints are in robot frame: (0, 0, 0) → (x₁, y₁, θ₁) → ... → (x_goal, y_goal, θ_goal)
- Everything is consistent in the same coordinate frame

---

## Geometric Intuition

### What the Function Should Do

1. **Input**:
   - "Stop line is 0.5m ahead, 0.02m to the left, at angle 0.1 rad"
   - "I want to turn left"

2. **Reasoning**:
   - "Left turn from stop line means: go 0.5m forward from stop line, 0.5m left, rotate 90°"
   - "But stop line is 0.5m ahead of me"
   - "So relative to ME, the goal is at: (0.5 + 0.5 = ~1.0m forward, considering angles)"
   - "Account for my offset and rotation to calculate exact goal in MY frame"

3. **Output**:
   - "From MY current position, I need to reach (0.45m, 0.42m, 1.47 rad)"
   - "Waypoints: 25% there, 50% there, 75% there, 100% there"
   - All relative to MY current position (which becomes origin when I start moving)

### The SE(2) Math Does This Correctly

The SE(2) algebra interpolation:
- Properly handles the coupling between translation and rotation
- Generates smooth curves (geodesics on the SE(2) manifold)
- Ensures waypoints are geometrically consistent

### Visualization

```
Robot Frame (origin at robot's current position):
     Y (left)
     ↑
     |    Goal (0.45, 0.42, 84°)
     |     ×
     |    /
     |   / waypoint 3
     |  ×
     | / waypoint 2
     |× waypoint 1
     ×------------→ X (forward)
   (0,0) Robot

Stop Line Frame (origin at stop line):
     Y (left)
     ↑
     | Goal (0.5, 0.5, 90°)
     | ×
     |
     |
     |
     ×------------→ X (forward)
   (0,0) Stop Line

     Robot is at (-0.5, -0.02, -0.1) in this frame
```

The math transforms the goal from "stop line frame" to "robot frame" correctly.

---

## Conclusion

The `calculate_goal_trajectory` function uses sophisticated SE(2) geometry to:

1. Transform the goal pose from stop line frame to robot frame
2. Convert the goal to a velocity representation (Lie algebra)
3. Interpolate waypoints by scaling the velocity
4. Convert waypoints back to poses (via exponential map)

**However**, there appears to be a bug at line 139 where waypoints are incorrectly transformed by `g_stop_pose`, putting them in the wrong coordinate frame. The fix is to use `inter_pose = rel` directly, keeping waypoints in the robot frame where they belong.

The mathematical framework is sound, but the coordinate frame handling needs this correction.
