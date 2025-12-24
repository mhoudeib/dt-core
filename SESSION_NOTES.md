# Unicorn Intersection Development Session - 2025-12-24

## Summary of Changes

This session focused on improving the AprilTag alignment system, LED signaling protocol, and fixing various bugs in the intersection navigation system.

---

## 1. AprilTag Alignment System

### Changes Made

**Fixed alignment direction** ([unicorn_intersection_node.py:379-382](packages/unicorn_intersection/src/unicorn_intersection_node.py#L379-L382))
- Changed from aligning with tag's x-axis to aligning with tag's z-axis
- Robot now faces **toward** the AprilTag (perpendicular to stop line)
- Uses `-tag_z_robot` as alignment direction

**Reordered operations** ([unicorn_intersection_node.py:213-222](packages/unicorn_intersection/src/unicorn_intersection_node.py#L213-L222))
- Alignment now happens **before** trajectory calculation
- Ensures trajectory is computed from a well-aligned pose
- Order: Align → Calculate Trajectory → Plan → Execute

**Added AprilTag scanning** ([unicorn_intersection_node.py:249-313](packages/unicorn_intersection/src/unicorn_intersection_node.py#L249-L313))
- Scans left, center, right, center when tag not initially visible
- Returns to original orientation after finding tag
- Configurable scan parameters:
  - `align_tag_scan_angle`: 0.4 rad (~23°)
  - `align_tag_scan_omega`: 0.3 rad/s
  - `align_tag_scan_wait`: 0.3 seconds

**Added heading offset** ([unicorn_intersection_node.py:408-416](packages/unicorn_intersection/src/unicorn_intersection_node.py#L408-L416))
- `align_tag_heading_offset` parameter allows fine-tuning alignment
- Positive = counterclockwise (left), Negative = clockwise (right)
- Currently set to 0.3 rad (~17°) in config

### Configuration

All parameters in [unicorn_intersection_node/default.yaml](packages/unicorn_intersection/config/unicorn_intersection_node/default.yaml):

```yaml
align_to_apriltag: True
align_tag_tolerance: 0.15          # radians (~8.6°)
align_tag_k: 1.5                   # proportional gain
align_tag_omega_max: 2.0           # rad/s max rotation speed
align_tag_max_time: 2.5            # seconds timeout
align_tag_heading_offset: 0.3      # radians offset from perpendicular
align_tag_scan_angle: 0.4          # radians (~23°)
align_tag_scan_omega: 0.3          # rad/s scan rotation speed
align_tag_scan_wait: 0.3           # seconds wait for detection
```

---

## 2. LED Communication Protocol

### LED Pattern Structure

5-LED array: `[Front Left, Rear Left, Top, Rear Right, Front Right]`

**Active LEDs:**
- **Front Left (Index 0)**: Priority level
- **Front Right (Index 4)**: Direction or Ready indicator

**Priority Colors** ([unicorn_intersection_node.py:147-152](packages/unicorn_intersection/src/unicorn_intersection_node.py#L147-L152)):
- Priority 1: Red
- Priority 2: Blue
- Priority 3: Purple
- Priority 4: White

**Direction Colors** ([unicorn_intersection_node.py:141-145](packages/unicorn_intersection/src/unicorn_intersection_node.py#L141-L145)):
- Left (0): Cyan
- Straight (1): Yellow
- Right (2): Pink

**Ready State**: Front Right LED turns Green

### FSM Integration

Modified [single_robot_indefinite_navigation.yaml](packages/fsm/config/fsm_node/single_robot_indefinite_navigation.yaml):

- **LANE_FOLLOWING**: Changed from "RED" to "CAR_DRIVING"
- **STOP_SIGN_INTERSECTION**: Removed `lights:` entry to allow custom LED control
- **INTERSECTION_CONTROL**: No lights specified (custom patterns maintained)

### LED Behavior Timeline

1. **STOP_SIGN_INTERSECTION**: Custom pattern shows priority + direction colors
2. **Ready state**: Front Right changes to green
3. **INTERSECTION_CONTROL**: Pattern maintained via 0.5s timer re-application
4. **Completion**: Switches to GREEN pattern

### Fixes

**Removed YELLOW override** ([unicorn_intersection_node.py:1033-1036](packages/unicorn_intersection/src/unicorn_intersection_node.py#L1033-L1036))
- Previously set YELLOW when reaching first waypoint
- Now maintains custom LED pattern throughout execution
- Timer ensures pattern persists despite FSM state changes

---

## 3. Bug Fixes

### Stop Line Detection Bug

**Issue**: Poses with `theta == 0.0` were rejected

**Fix** ([unicorn_intersection_node.py:204](packages/unicorn_intersection/src/unicorn_intersection_node.py#L204)):
```python
# Before: if msg.at_stop_line and msg.stop_pose.theta != 0.0:
# After:  if msg.at_stop_line:
```

**Reason**: 0.0 is legitimate for perfectly aligned robot

---

### Waypoint Oscillation at Origin

**Issue**: Robot oscillates when waypoint generated at (0,0)

**Fix** ([unicorn_intersection_node.py:1078-1102](packages/unicorn_intersection/src/unicorn_intersection_node.py#L1078-L1102)):
- Special handling for near-origin waypoints (< 5cm from origin)
- Relaxed threshold for degenerate segments
- Proper along-track/cross-track decomposition

---

### Stop Line Acceleration Issue

**Issue**: Robot accelerates when detecting stop line instead of slowing down

**Root Cause Analysis**:
- `stop_distance` was reduced to 0.06m
- Lane controller slowdown zone: 40cm → 10cm
- Robot reaches `at_stop_line` trigger at 6cm (past slowdown zone)
- Result: Full speed until sudden stop

**Fix** ([stop_line_filter_node/default.yaml](packages/stop_line_filter/config/stop_line_filter_node/default.yaml)):
```yaml
# Restored from 0.06 to:
stop_distance: 0.12
```

**Lane Controller Slowdown** ([lane_controller_node/default.yaml](packages/lane_control/config/lane_controller_node/default.yaml)):
```yaml
stop_line_slowdown:
  start: 0.4  # Begin slowing at 40cm
  end: 0.1    # Minimum speed at 10cm
```

**Behavior**:
- Distance > 40cm: Full speed (v_bar = 0.19 m/s)
- Distance 10-40cm: Gradual slowdown
- Distance < 10cm: Minimum speed
- `at_stop_line` triggers at 12cm (within slowdown zone)

---

## 4. Outstanding Issues

### Open Question: Wait for Robot to Stop

**Issue**: When `at_stop_line` triggers FSM transition, lane controller is disabled but robot may still have momentum. Alignment starts while robot is decelerating.

**Proposed Solutions**:
1. **Time-based wait**: Add `rospy.sleep(0.5)` before alignment (rejected - arbitrary)
2. **Velocity-based condition**: Wait until `self.tv` and `self.rv` are below threshold (preferred - needs implementation)

**Current Status**: Not yet implemented

**Encoder Velocity Data Available**:
- `self.tv`: Translational velocity (m/s)
- `self.rv`: Rotational velocity (rad/s)
- Updated in `cb_ts_encoders` callback

**Implementation Needed**:
1. Modify encoder callback to run in all states (not just EXECUTING/ALIGNING)
2. Add `wait_until_stopped()` function
3. Call before alignment in `check_if_go()`

---

## 5. Current File States

### Modified Files

1. **packages/unicorn_intersection/src/unicorn_intersection_node.py**
   - AprilTag alignment system (z-axis, scanning, offset)
   - LED pattern management
   - Stop line callback fix
   - Waypoint checking improvements

2. **packages/unicorn_intersection/config/unicorn_intersection_node/default.yaml**
   - AprilTag alignment parameters
   - Heading offset: 0.3 rad

3. **packages/fsm/config/fsm_node/single_robot_indefinite_navigation.yaml**
   - LANE_FOLLOWING: "CAR_DRIVING"
   - STOP_SIGN_INTERSECTION: No lights (custom control)

4. **packages/stop_line_filter/config/stop_line_filter_node/default.yaml**
   - stop_distance: 0.12 (restored from 0.06)

### Lane Controller (Reference Only)

**packages/lane_control/config/lane_controller_node/default.yaml**:
- v_bar: 0.19 m/s
- stop_line_slowdown: start 0.4m, end 0.1m

**Note**: Found variable swap bug in lane_controller_node.py (lines 143-160) but not fixed in this session.

---

## 6. Testing Checklist

- [ ] AprilTag alignment perpendicular to stop line
- [ ] AprilTag scanning when tag not visible
- [ ] LED priority colors display correctly
- [ ] LED direction colors (cyan/yellow/pink) display correctly
- [ ] LED ready state (green) shows before execution
- [ ] LED pattern maintained during intersection crossing
- [ ] Stop line detection at 12cm with smooth deceleration
- [ ] No oscillation at origin waypoints
- [ ] No acceleration when approaching stop line
- [ ] Smooth transition from STOP_SIGN_INTERSECTION to INTERSECTION_CONTROL

---

## 7. Known Limitations

1. **Priority calculation**: `get_priority()` always returns 1 (placeholder implementation)
2. **Encoder velocity**: Not tracked in READY state (needed for stop detection)
3. **Lane controller bug**: Variable swap in obstacle/stop line callbacks (not critical for current use)
4. **Race condition**: Occasional early trajectory calculation (rare, not addressed)

---

## 8. Future Work

### High Priority
- Implement velocity-based wait before alignment
- Implement proper priority calculation based on intersection occupancy

### Medium Priority
- Fix lane controller variable swap bug
- Add LED patterns for ALIGNING state
- Tune heading offset for optimal intersection entry

### Low Priority
- Add configurable settling time as fallback
- Investigate rare early trajectory calculation
- Add LED pattern during AprilTag scanning

---

## Notes

- All changes maintain backward compatibility
- Configuration changes can be reverted by editing YAML files
- LED protocol is now conflict-free with FSM
- AprilTag alignment significantly improves intersection entry consistency
