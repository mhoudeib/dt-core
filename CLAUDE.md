# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

`dt-core` is the core software stack for the Duckietown project, enabling autonomous driving on Duckiebots and autonomous flight on Duckiedrones. The codebase is built on ROS (Robot Operating System) and uses a modular, package-based architecture with catkin as the build system.

## Build and Development Commands

### Building the Project

The project uses Docker-based development with the Duckietown Shell (`dts`):

```bash
# Build on the robot (from dt-core root)
dts devel build -f -H ROBOTNAME.local

# Build for exercises (from mooc-exercises/project folder)
dts exercises build
```

### Running on Robot

```bash
# Run with default launcher
dts devel run -f -s -M -H ROBOTNAME.local

# Run with specific launcher
dts devel run -f -s -M -H ROBOTNAME.local -c /bin/bash
# Then: ./launchers/[launcher-name].sh

# Run exercises (from mooc-exercises/project folder)
dts exercises test --duckiebot_name ROBOTNAME
```

### Important Setup Commands

After robot reboot, always execute for accurate LED control:
```bash
ssh duckie@ROBOTNAME.local "echo '400000' | sudo tee /sys/bus/i2c/devices/i2c-1/bus_clk_rate"
```

Update commands:
```bash
dts update
dts desktop update
dts duckiebot update ROBOTNAME
```

### Available Launchers

Key launchers in `launchers/` directory:
- `default.sh` - Routes to robot-type-specific launcher
- `default-duckiebot.sh` - Basic car interface
- `lane-following.sh` - Lane following demo
- `indefinite_navigation.sh` - Indefinite navigation with AprilTags
- `single_robot_indefinite_navigation.sh` - Single robot navigation
- `communication.sh` - Traffic light and stop sign coordination
- `default-duckiedrone.sh` - Duckiedrone interface
- Various duckiedrone-specific launchers for altitude, PID, optical flow, etc.

## Architecture

### Package Structure

The codebase is organized into ROS packages under `packages/`:

**Core Duckiebot Packages:**
- `lane_control/` - Lane controller node for steering control
- `lane_filter/` - Lane pose estimation using histogram filter
- `line_detector/` - Line detection from camera images
- `ground_projection/` - Projects line segments to ground plane
- `image_processing/` - Image rectification and decoding
- `anti_instagram/` - Color correction for varying lighting

**Navigation:**
- `navigation/` - Intersection navigation, AprilTag turns, action dispatchers
- `fsm/` - Finite State Machine for behavioral control
- `unicorn_intersection/` - Intersection coordination logic
- `stop_line_filter/` - Stop line detection

**Perception:**
- `apriltag/` - AprilTag detection and postprocessing
- `obstacle_detection/` - Vehicle and obstacle detection
- `led_emitter/`, `led_joy_mapper/`, `led_pattern_switch/` - LED control system

**Robot-Specific:**
- `robots/duckiebot/` - Duckiebot hardware interfaces (car_interface, dagu_car, joy_mapper)
- `robots/duckiedrone/` - Duckiedrone packages (altitude, state_estimator, pid_controller, optical_flow, localization, slam)

**Demos:**
- `duckietown_demos/` - Launch files for complete demos

**Other:**
- `visualization_tools/` - RViz visualization nodes
- `experimental/` - Experimental features and communication protocols
- `old/` - Deprecated packages

### ROS Architecture

The system follows ROS conventions:
- **Nodes**: Python nodes typically inherit from a base ROS node pattern
- **Launch files**: XML files in `packages/*/launch/` define node startup configurations
- **Master launch**: `packages/duckietown_demos/launch/master.launch` is the main entry point that conditionally starts subsystems based on arguments
- **Topics**: Communication uses ROS topics with remapping defined in launch files
- **Parameters**: Node configuration in `packages/*/config/*/` YAML files

### Key Integration Points

**Lane Following Stack Flow:**
1. Camera image → `line_detector_node` → detected line segments
2. Line segments → `ground_projection_node` → ground-projected segments
3. Projected segments → `lane_filter_node` → lane pose estimate
4. Lane pose → `lane_controller_node` → motor commands
5. FSM coordinates state transitions and behavior

**Finite State Machine (FSM):**
- Controls high-level robot behavior states
- Coordinates between lane following, intersection navigation, and coordination
- Can be configured per-demo via `fsm_file_name` parameter

**Demo Configuration:**
The `master.launch` file uses boolean arguments to enable/disable subsystems:
- Set demo-specific arguments (e.g., `lane_following=true`)
- Conditionally includes package launch files
- Handles topic remapping between nodes

### Duckiedrone Architecture

Duckiedrone uses a different control flow:
- Flight controller driver interface in `dt-duckiebot-interface` repo
- State estimation combining IMU, optical flow, and altitude sensors
- PID controller for altitude and position control
- Commands multiplexer switching between manual and autonomous modes

## Development Workflow

### Typical Development Cycle

1. Clone repo to **local computer** (not on the robot)
2. Make code changes locally
3. Build on robot: `dts devel build -f -H ROBOTNAME.local`
4. Run on robot: `dts devel run -f -s -M -H ROBOTNAME.local`
5. Monitor with RQT or rostopic tools

### Testing and Debugging

Use ROS GUI tools for debugging:
```bash
# Start GUI tools container
dts start_gui_tools --name CONTAINER_NAME ROBOTNAME

# Inside container, use:
rqt &                    # RQT for topic visualization
rostopic echo /topic     # Monitor topics
rosnode info /node       # Node information
```

### Configuration Changes

Modify node behavior via YAML config files in `packages/*/config/*/default.yaml`

For standalone testing (e.g., communication_node):
- Set demo mode flags (`tl_demo_mode`, `ss_demo_mode`) in config YAML
- Rebuild and redeploy

### Launcher Modification

To change what runs on startup, edit `launchers/default.sh` to call a different launcher or roslaunch command.

## Code Patterns

### ROS Node Structure

Python nodes typically:
- Use `rospy` for ROS interactions
- Subscribe/publish to topics
- Load parameters from ROS parameter server
- Implement callback functions for topic messages

### Launch File Patterns

Launch files use:
- `<arg>` tags for configuration
- `<remap>` tags to connect topics between nodes
- `<include>` tags to compose launch files
- `<group if="$(arg flag)">` for conditional launching

### Catkin Build System

- Each package has `package.xml` (dependencies, metadata) and `CMakeLists.txt` (build config)
- ROS messages in `duckietown_msgs` package (external dependency)
- Build happens in Docker container with catkin workspace

## Branch Information

- Default branch for development: `ente` (for Duckiedrone work)
- Current branch: `ente-indefinite-navigation`
- Main integration happens through PRs to main branches

## Dependencies

### System Dependencies
- ROS Noetic (implied by catkin and ros-node-utils)
- Python 3
- OpenCV (via cv_bridge)
- Docker

### Python Dependencies (dependencies-py3.txt)
- `dt-apriltags==3.1.7` - AprilTag detection
- `duckietown-utils-ente==7.0.0` - Duckietown utilities
- `lib-dt-computer-vision` (git, ente branch) - Computer vision library
- `lib-dt-state-estimation` (git, ente branch) - State estimation library
- `filterpy==1.4.5` - Kalman filtering (for drones)
- `numpy-quaternion==2023.0.4` - Quaternion math (for drones)
- `simple-pid<=2.0.1` - PID control (for drones)
- Various compmake/comptests packages

### Docker Build Process

The Dockerfile:
1. Installs apt dependencies from `dependencies-apt.txt`
2. Installs Python dependencies from `dependencies-py3.txt`
3. Copies packages to `${PROJECT_PATH}/packages`
4. Runs `catkin build` to compile workspace
5. Installs launcher scripts from `launchers/`
6. Sets default command to `dt-launcher-${DT_LAUNCHER}`

## Important Environment Variables

- `VEHICLE_NAME` - Robot hostname
- `ROBOT_TYPE` - duckiebot, watchtower, or duckiedrone
- `ROBOT_CONFIGURATION` - DB19, DB20, WT18, etc.
- `FSM_NODE_CONTROL` - Controls FSM behavior (default "1")
- `DUCKIETOWN_ROOT` - Source directory path
- `DT_PROJECT_PATH` - Project installation path

## Special Considerations

### LED Control Timing
LED flashing consistency depends on `fifo-bridge` update rate. For stop sign demos, prefer `dts devel run` over `dts exercises test` for better LED control.

### Communication Node Modes
The communication_node can operate in:
- **Default**: Integrated with lane following and FSM
- **Traffic Light Demo**: Set `tl_demo_mode: True` for standalone TL testing
- **Stop Sign Demo**: Set `ss_demo_mode: True` for standalone SS testing

### IMU Calibration
For Duckiedrone, calibrate IMU via ROS service before flight:
```bash
# In rqt: Plugins->Services->Service Caller
# Call: /ROBOTNAME/flight_controller_node/calibrate_imu
```

### Heartbeat Checks
Disable heartbeat checks for debugging by setting values to `false` in:
`packages/flight_controller_driver/config/flight_controller_node/default.yaml`
