# Intersection Navigation Instructions

## Run Commands (sim/virtual Duckiebot)
- Start virtual bot and map:  
  `dts duckiebot virtual start vggquack`  
  `dts fleet discover`  
  `dts matrix run --standalone -m assets/duckiematrix/maps/intersections`  
  `dts matrix attach vggquack map_0/vehicle_0`
- Build and run the stack:  
  `dts devel build -H vggquack`  
  `dts devel run -H vggquack -M -L single_robot_indefinite_navigation`
- Teleop (optional): `dts duckiebot keyboard_control vggquack`
- GUI/RViz tools: `dts gui vggquack`  
  View the debug image: `rosrun image_view image_view image:=/vggquack/unicorn_intersection_node/debug/trajectory _image_transport:=compressed`

## Key Topics
- Planned waypoints (odom frame): `/vggquack/unicorn_intersection_node/reference_trajectory`
- Debug image (stop-line frame overlay): `/vggquack/unicorn_intersection_node/debug/trajectory`
- Path/markers for RViz: `/vggquack/unicorn_intersection_node/path`, `/vggquack/unicorn_intersection_node/markers`
- Control commands: `/vggquack/unicorn_intersection_node/car_cmd`

## What Was Fixed
- Frame mismatch: waypoints are now transformed into odom at planning time so control uses the same frame as odometry; visualization stays in the stop-line frame.
- Left-turn curvature: added a via-point split for left turns and per-maneuver waypoint counts (`left/right/straight_num_waypoints`).
- Waypoint skipping: introduced per-maneuver thresholds in `check_point` so left turns can tolerate larger errors while right/straight use tighter gates.
- Intersection overlay: debug JPEG now shows a four-way intersection (red stop lines, yellow center lines) with the origin at the left stop line, plus the robot pose (both actual offset and normalized reference) and offset trajectory overlay.

## Tuning (config: `packages/unicorn_intersection/config/unicorn_intersection_node/default.yaml`)
- Waypoint counts: `left_num_waypoints`, `right_num_waypoints`, `straight_num_waypoints`.
- Waypoint reach thresholds: `left_x/dist_threshold`, `right_x/dist_threshold`, `straight_x/dist_threshold`.
- Left turn via point: `left_turn_via_point`, `use_left_turn_via_point`.
- Stop-line offset if needed: `stop_line_offset`.
