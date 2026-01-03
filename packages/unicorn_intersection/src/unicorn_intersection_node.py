#!/usr/bin/env python3
import json
import numpy as np
import rospy
from duckietown_msgs.msg import (BoolStamped,
    TurnIDandType,
    WheelEncoderStamped,
    Twist2DStamped,
    StopLineReading,
    LEDPattern,
    FSMState,
    AprilTagDetectionArray,
    AprilTagDetection
    )
from std_msgs.msg import String
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
import math 
from geometry_msgs.msg import Quaternion, Twist, Pose2D, Point, Vector3, TransformStamped, Transform
from duckietown_msgs.srv import SetCustomLEDPattern, SetCustomLEDPatternRequest, ChangePattern

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage

import message_filters
from tf import transformations as tr

import geometry as g
import cv2

class UnicornIntersectionNode(DTROS):
    def __init__(self, node_name):
        super(UnicornIntersectionNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL,
            fsm_controlled=True)

        self.node_name = node_name
        self.internal_state = "READY"
        self.turn_type_received = False
        self.stop_line_pose_received = False

        ## setup Parameters
        self.setupParams()

        self.goal_poses = {
            "left": self.dictionary_pose_to_geometry(self.canonical_goal_pose_left),
            "right": self.dictionary_pose_to_geometry(self.canonical_goal_pose_right),
            "straight": self.dictionary_pose_to_geometry(self.canonical_goal_pose_straight)
        }

        self.reference_trajectory = []

        ## Internal variables
        self.turn_type = -1
        self.stop_line_pose = Pose2D()

        self.debug = False
        self.apriltag_detections = {}
        self.turn_tag_id = None
        self.validated_tag_detection = None  # Store the validated tag from random_april_tag_turns_node


        ## Subscribers
        self.sub_turn_type = rospy.Subscriber("~turn_id_and_type", TurnIDandType, self.cbTurnType)
        # Subscribe to validated tag detection from random_april_tag_turns_node
        self.sub_validated_tag = rospy.Subscriber("random_april_tag_turns_node/validated_tag_detection", AprilTagDetection, self.cbValidatedTag, queue_size=1)
        self.sub_encoder_left = message_filters.Subscriber("~left_wheel_encoder_driver_node/tick", WheelEncoderStamped)
        self.sub_encoder_right = message_filters.Subscriber("~right_wheel_encoder_driver_node/tick", WheelEncoderStamped)
        self.sub_stop_line_reading = rospy.Subscriber("~stop_line_reading", StopLineReading, self.cbStopLineReading)
        # Raw Apriltag detections (AprilTagDetectionArray) from apriltag_detector_node
        self.sub_apriltags = rospy.Subscriber("apriltag_detector_node/detections", AprilTagDetectionArray, self.cbApriltags, queue_size=1)
        """
        self.sub_fsm_node_mode = rospy.Subscriber(
            name='fsm_node/mode',   # The full, resolved topic name
            data_class=FSMState,     # The message type it expects to receive
            callback=self.onFSMStateChange, 
            queue_size=1
        )
        """

        ## Publisher
        self.pub_int_done = rospy.Publisher("~intersection_done", BoolStamped, queue_size=1)
        self.car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)
        self.reference_trajectory_pub = rospy.Publisher(
            "~reference_trajectory",
            Odometry,
            queue_size=self.num_waypoints,
        )
        self.pub_debug_trajectory_img = rospy.Publisher(
            "~debug/trajectory/compressed",
            CompressedImage,
            queue_size=1,
        )

        # LED service proxies - wait for services to be available
        try:
            rospy.wait_for_service('led_emitter_node/set_pattern', timeout=5.0)
            rospy.wait_for_service('led_emitter_node/set_custom_pattern', timeout=5.0)
            self.led_pattern_service = rospy.ServiceProxy('led_emitter_node/set_pattern', ChangePattern)
            self.led_custom_pattern_service = rospy.ServiceProxy('led_emitter_node/set_custom_pattern', SetCustomLEDPattern)
            rospy.loginfo(f"[{self.node_name}] LED services connected")
        except rospy.ROSException as e:
            rospy.logwarn(f"[{self.node_name}] LED services not available: {e}. LEDs may not work.")
            self.led_pattern_service = None
            self.led_custom_pattern_service = None

        # Store current LED pattern state for re-application after FSM changes
        self.current_led_pattern = None
        self.current_led_pattern_type = None  # 'predefined' or 'custom'
        self.led_pattern_timer = None  # Timer to periodically re-apply LED pattern

        self.ts_encoders = message_filters.ApproximateTimeSynchronizer(
            [self.sub_encoder_left, self.sub_encoder_right], 1, 1
        )
        self.ts_encoders.registerCallback(self.cb_ts_encoders)

        ## update Parameters timer
        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)

        ## Deadreckoning 

        # introducing deadreckoning
        self.reset_odometry()

        self.alpha = 0.0
        
        #Led protocol

        self.direction_colors = {
            -1: 'switchedoff', #Default
            0: 'cyan',   # Left
            1: 'yellow', # Straight
            2: 'pink'    # Right
        }

        self.priority_colors = {
            1: 'red',
            2: 'blue',
            3: 'purple',
            4: 'white'
        }

        #Intersection planning mannagement variables
        self.priority_level = -1
        self.negotiation_end = False
        self.led_priority_set = False
        self.led_ready_set = False

        # Track FSM state to abort operations when switching modes
        self.fsm_state = "LANE_FOLLOWING"  # Default state

        self.log("Initialialized unicorn intersection node")

    def onFSMStateChange(self, msg):
        """Handle FSM state transitions and reset node state accordingly."""
        new_state = msg.state
        self.fsm_state = new_state
        rospy.loginfo(f"[{self.node_name}] FSM state changed to: {new_state}")

        if new_state == "NORMAL_JOYSTICK_CONTROL":
            # Reset all state when entering joystick mode
            rospy.loginfo(f"[{self.node_name}] Entering joystick control - resetting state")
            self.reset_odometry()
            self.internal_state = "READY"
            self.stop_line_pose_received = False
            self.turn_type_received = False
            self.g_stop_pose_plan = None
            self.priority_level = 0
            self.negotiation_end = False
            self.led_priority_set = False
            self.led_ready_set = False
            self.current_led_pattern = None
            self.current_led_pattern_type = None
            self.stop_line_pose = Pose2D()
            self.apriltag_detections = {}
            self.validated_tag_detection = None
            self.reference_trajectory = []
            self.iter_ = 0

        elif new_state == "LANE_FOLLOWING":
            # Full reset when returning to autopilot
            rospy.loginfo(f"[{self.node_name}] Returning to autopilot - full reset")
            self.reset_odometry()
            self.internal_state = "READY"
            self.stop_line_pose_received = False
            self.turn_type_received = False
            self.g_stop_pose_plan = None
            self.priority_level = 0
            self.negotiation_end = False
            self.led_priority_set = False
            self.led_ready_set = False
            self.current_led_pattern = None
            self.current_led_pattern_type = None
            self.stop_line_pose = Pose2D()
            self.apriltag_detections = {}
            self.validated_tag_detection = None
            self.reference_trajectory = []
            self.iter_ = 0

            # Clear any stale wheel commands
            stop_cmd = Twist2DStamped()
            stop_cmd.header.stamp = rospy.Time.now()
            stop_cmd.v = 0.0
            stop_cmd.omega = 0.0
            self.car_cmd.publish(stop_cmd)

        elif new_state == "INTERSECTION_CONTROL":
            # Re-apply LED pattern after FSM transition
            if self.current_led_pattern is not None:
                rospy.Timer(rospy.Duration(0.3), lambda event: self._reapply_led_pattern(), oneshot=True)
            if self.internal_state == "EXECUTING" and self.current_led_pattern is not None:
                self._start_led_pattern_timer()
        else:
            self._stop_led_pattern_timer()
    
    def cbStopLineReading(self, msg):
        """Process stop line detections from stop_line_filter_node."""
        if self.fsm_state == "NORMAL_JOYSTICK_CONTROL":
            return

        if self.stop_line_pose_received:
            return

        if msg.at_stop_line:
            # Validate that the stop line pose is reasonable (not at origin)
            # A stop line at exactly (0, 0) is likely invalid/stale
            # if abs(msg.stop_pose.x) < 0.01 and abs(msg.stop_pose.y) < 0.01:
            #     rospy.logwarn(f"[unicorn_intersection_node] Ignoring invalid stop line pose at origin: {msg.stop_pose}")
            #     return

            self.stop_line_pose = msg.stop_pose
            self.stop_line_pose_received = True
            rospy.loginfo(f"[{self.node_name}] Received stop line pose: x={self.stop_line_pose.x:.3f}, y={self.stop_line_pose.y:.3f}, theta={self.stop_line_pose.theta:.3f}")
            self.check_if_go()

    def check_if_go(self):
        """Verify all requirements met before starting intersection navigation."""
        if (self.stop_line_pose_received and self.turn_type_received and self.internal_state == "READY"):
            rospy.loginfo(f"[{self.node_name}] All prerequisites met, preparing for intersection")

            yaw_before_alignment = self.yaw

            # Align robot heading to AprilTag before trajectory planning
            if self.align_to_apriltag:
                # rospy.loginfo(f"[{self.node_name}] Yaw before alignment: {yaw_before_alignment:.3f} rad")
                self.align_to_apriltag_heading()
                # rospy.loginfo(f"[{self.node_name}] Yaw after alignment: {self.yaw:.3f} rad")

                # Apply heading correction to stop_line_pose
                heading_correction = self.yaw - yaw_before_alignment
                # rospy.loginfo(f"[{self.node_name}] Heading correction: {heading_correction:.3f} rad")
                self.stop_line_pose.theta -= heading_correction

            # Calculate trajectory from aligned pose
            self.reference_trajectory = self.calculate_goal_trajectory()
            self.intersection_planning()

            if self.negotiation_end and self.internal_state != "EXECUTING":
                car_control_msg = Twist2DStamped()
                car_control_msg.header.stamp = rospy.Time.now()
                car_control_msg.header.seq = 0
                car_control_msg.v = 0
                car_control_msg.omega = 0
                self.car_cmd.publish(car_control_msg)
                rospy.loginfo(f"[{self.node_name}] Starting intersection navigation")
                self.internal_state = "EXECUTING"
        else:
            self.intersection_planning()
            rospy.loginfo(f"[unicorn_intersection_node] We don't have what we need yet: "
                      f"stop_line received: {self.stop_line_pose_received} " 
                      f"turn_type_received: {self.turn_type_received} "
                      f"internal_state:{self.internal_state} ")

    def cbValidatedTag(self, msg):
        """Store the validated tag detection from random_april_tag_turns_node."""
        self.validated_tag_detection = msg
        rospy.loginfo(f"[{self.node_name}] Received validated tag {msg.tag_id} from random_april_tag_turns_node")
        rospy.loginfo(f"[{self.node_name}] Tag position: x={msg.transform.translation.x:.3f}, y={msg.transform.translation.y:.3f}, z={msg.transform.translation.z:.3f}")

    def cbApriltags(self, msg):
        """Store latest Apriltag detections (raw AprilTagDetectionArray) for alignment."""
        if len(msg.detections) == 0:
            return

        for detection in msg.detections:
            self.apriltag_detections[detection.tag_id] = detection

    def _scan_for_apriltag(self, tag_id):
        """
        Scan for an AprilTag by rotating left, then right, then returning to center.
        Returns the detection if found, None otherwise.
        """
        # Store initial yaw
        initial_yaw = self.yaw
        rospy.loginfo(f"[{self.node_name}] Starting AprilTag scan. Initial yaw: {initial_yaw:.3f} rad ({np.degrees(initial_yaw):.1f} deg)")

        # Function to rotate to a target angle
        def rotate_to_angle(target_yaw, timeout=3.0):
            start_time = rospy.Time.now()
            rate = rospy.Rate(20)
            while not rospy.is_shutdown():
                yaw_err = self.shortest_angle(target_yaw - self.yaw)
                if abs(yaw_err) < 0.05:  # 3 degree tolerance
                    break
                elapsed = (rospy.Time.now() - start_time).to_sec()
                if elapsed > timeout:
                    rospy.logwarn(f"[{self.node_name}] Scan rotation timed out")
                    break
                omega = max(-self.align_tag_scan_omega, min(self.align_tag_scan_omega, 2.0 * yaw_err))
                cmd = Twist2DStamped()
                cmd.header.stamp = rospy.Time.now()
                cmd.v = 0.0
                cmd.omega = omega
                self.car_cmd.publish(cmd)
                rate.sleep()
            # Stop
            cmd = Twist2DStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.v = 0.0
            cmd.omega = 0.0
            self.car_cmd.publish(cmd)

        # Scan sequence: left, center, right, center
        scan_positions = [
            (initial_yaw + self.align_tag_scan_angle, "left"),
            (initial_yaw, "center"),
            (initial_yaw - self.align_tag_scan_angle, "right"),
            (initial_yaw, "center (final)")
        ]

        for target_yaw, position_name in scan_positions:
            rospy.loginfo(f"[{self.node_name}] Scanning {position_name} (target: {np.degrees(target_yaw):.1f} deg) for tag {tag_id}")
            rotate_to_angle(target_yaw)

            # Wait for detections to update
            rospy.sleep(self.align_tag_scan_wait)

            # Check if tag is now visible
            detection = self.apriltag_detections.get(tag_id)
            if detection is not None:
                rospy.loginfo(f"[{self.node_name}] Found tag {tag_id} at {position_name} position!")
                # Return to initial orientation before returning
                if abs(self.shortest_angle(self.yaw - initial_yaw)) > 0.05:
                    rospy.loginfo(f"[{self.node_name}] Returning to initial orientation before alignment")
                    rotate_to_angle(initial_yaw)
                    rospy.sleep(self.align_tag_scan_wait)
                    # Get fresh detection at initial position
                    detection = self.apriltag_detections.get(tag_id)
                return detection

        rospy.logwarn(f"[{self.node_name}] Tag {tag_id} not found after complete scan")
        return None

    def align_to_apriltag_heading(self):
        """Rotate robot to align parallel with the Apriltag's x-axis before executing."""
        tag_id = self.turn_tag_id
        if tag_id is None:
            rospy.logwarn(f"[{self.node_name}] align_to_apriltag enabled but no tag_id provided with turn_type.")
            return

        # Use the validated tag detection from random_april_tag_turns_node
        # This tag has already passed geometric filtering (perpendicularity, distance, horizontal angle)
        detection = self.validated_tag_detection

        # Validate the tag before using it
        if detection is not None:
            # Check if validated tag matches expected tag_id
            if detection.tag_id != tag_id:
                rospy.logwarn(f"[{self.node_name}] Validated tag ID {detection.tag_id} does not match expected tag ID {tag_id}!")
                rospy.logwarn(f"[{self.node_name}] This likely means the validated tag is stale from a previous intersection.")
                rospy.logwarn(f"[{self.node_name}] Discarding validated tag and falling back to fresh detection.")
                detection = None
            else:
                rospy.loginfo(f"[{self.node_name}] Using validated tag {detection.tag_id} from random_april_tag_turns_node")
                # Log the tag position for debugging
                pos_debug = detection.transform.translation
                rospy.loginfo(f"[{self.node_name}] Validated tag position (camera frame): x={pos_debug.x:.3f}, y={pos_debug.y:.3f}, z={pos_debug.z:.3f}")

        # Fallback: if no validated tag or it was stale, check apriltag_detections dictionary
        if detection is None:
            rospy.logwarn(f"[{self.node_name}] No valid validated tag - falling back to apriltag_detections dictionary")
            rospy.loginfo(f"[{self.node_name}] Available tags in dictionary: {list(self.apriltag_detections.keys())}")

            detection = self.apriltag_detections.get(tag_id)

            # If tag not found, try scanning for it
            if detection is None:
                rospy.logwarn(f"[{self.node_name}] Tag {tag_id} not in dictionary. Starting scan procedure.")
                detection = self._scan_for_apriltag(tag_id)
                # If still not found after scanning, give up and continue without alignment
                if detection is None:
                    rospy.logwarn(f"[{self.node_name}] Tag {tag_id} not found after scanning. Proceeding without alignment.")
                    return
            else:
                rospy.loginfo(f"[{self.node_name}] Found tag {tag_id} in apriltag_detections dictionary (not validated!)")
                pos_debug = detection.transform.translation
                rospy.loginfo(f"[{self.node_name}] Tag position (camera frame): x={pos_debug.x:.3f}, y={pos_debug.y:.3f}, z={pos_debug.z:.3f}")

        prev_state = self.internal_state
        self.internal_state = "ALIGNING"
        # Extract AprilTag pose in camera frame
        pos = detection.transform.translation
        rot = detection.transform.rotation

        # Convert quaternion to rotation matrix
        R_cam_tag = tr.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3, :3]

        # Extract tag axes in camera optical frame
        # In camera optical frame: x=right, y=down, z=forward
        tag_x_cam = R_cam_tag[:, 0]  # tag's x-axis (red arrow)
        tag_y_cam = R_cam_tag[:, 1]  # tag's y-axis (green arrow)
        tag_z_cam = R_cam_tag[:, 2]  # tag's z-axis (blue arrow, normal to tag surface)

        # Transform from camera optical frame to robot base frame
        # Camera optical frame: x=right, y=down, z=forward
        # Robot base frame: x=forward, y=left, z=up
        # Transformation: robot_x = cam_z, robot_y = -cam_x, robot_z = -cam_y
        tag_x_robot = np.array([tag_x_cam[2], -tag_x_cam[0], -tag_x_cam[1]])
        tag_z_robot = np.array([tag_z_cam[2], -tag_z_cam[0], -tag_z_cam[1]])


        # Also get tag position in robot frame for reference
        tag_pos_robot = np.array([pos.z, -pos.x, -pos.y])
        rospy.loginfo(f"[{self.node_name}] Tag position in robot frame: {tag_pos_robot}")

        # To align perpendicular to the stop line, we want to face TOWARD the tag
        # This means aligning opposite to the tag's z-axis (which points away from the tag toward us)
        # So we want to face in the direction of -tag_z_robot
        alignment_direction = -tag_z_robot
        rospy.loginfo(f"[{self.node_name}] Alignment direction: {alignment_direction}")
        # Calculate desired yaw to face toward the tag (perpendicular to stop line)
        if np.linalg.norm(alignment_direction[:2]) < 1e-6:
            # Fallback: tag is directly above/below, use position vector
            rospy.logwarn(f"[{self.node_name}] Tag directly above/below, using position-based alignment")
            desired_yaw = math.atan2(tag_pos_robot[1], tag_pos_robot[0])
        else:
            # The alignment direction has two opposite options (180° apart)
            # Choose the one that requires the smallest rotation from current heading
            yaw_option_1 = math.atan2(alignment_direction[1], alignment_direction[0])
            yaw_option_2 = self.shortest_angle(yaw_option_1 + math.pi)

            # Calculate angular errors from current yaw
            error_1 = self.shortest_angle(yaw_option_1 - self.yaw)
            error_2 = self.shortest_angle(yaw_option_2 - self.yaw)

            # Select the direction with smallest angular error from current yaw
            if abs(error_1) < abs(error_2):
                desired_yaw = yaw_option_1
                selected_error = error_1
            else:
                desired_yaw = yaw_option_2
                selected_error = error_2

            rospy.loginfo(f"[{self.node_name}] Current yaw: {self.yaw:.3f} rad ({np.degrees(self.yaw):.1f} deg)")
            rospy.loginfo(f"[{self.node_name}] Option 1: {yaw_option_1:.3f} rad ({np.degrees(yaw_option_1):.1f} deg), error: {error_1:.3f} rad ({np.degrees(error_1):.1f} deg)")
            rospy.loginfo(f"[{self.node_name}] Option 2: {yaw_option_2:.3f} rad ({np.degrees(yaw_option_2):.1f} deg), error: {error_2:.3f} rad ({np.degrees(error_2):.1f} deg)")
            rospy.loginfo(f"[{self.node_name}] Selected desired yaw (before offset): {desired_yaw:.3f} rad ({np.degrees(desired_yaw):.1f} deg)")

            # Check if the selected turn angle exceeds the maximum allowed
            if abs(selected_error) > self.align_tag_max_turn_angle:
                rospy.logwarn(f"[{self.node_name}] Turn angle {abs(selected_error):.3f} rad ({np.degrees(abs(selected_error)):.1f} deg) exceeds max {self.align_tag_max_turn_angle:.3f} rad ({np.degrees(self.align_tag_max_turn_angle):.1f} deg)")
                rospy.logwarn(f"[{self.node_name}] Switching to opposite direction to avoid excessive turn")
                # Switch to the other option
                if desired_yaw == yaw_option_1:
                    desired_yaw = yaw_option_2
                    selected_error = error_2
                else:
                    desired_yaw = yaw_option_1
                    selected_error = error_1
                rospy.loginfo(f"[{self.node_name}] New desired yaw: {desired_yaw:.3f} rad ({np.degrees(desired_yaw):.1f} deg), error: {selected_error:.3f} rad ({np.degrees(selected_error):.1f} deg)")

        # Apply heading offset
        # Positive offset = rotate counterclockwise (left), Negative = clockwise (right)
        desired_yaw_with_offset = self.shortest_angle(desired_yaw + self.align_tag_heading_offset)

        if abs(self.align_tag_heading_offset) > 0.01:  # Only log if offset is significant
            rospy.loginfo(f"[{self.node_name}] Heading offset: {self.align_tag_heading_offset:.3f} rad ({np.degrees(self.align_tag_heading_offset):.1f} deg)")
            rospy.loginfo(f"[{self.node_name}] Final desired yaw (with offset): {desired_yaw_with_offset:.3f} rad ({np.degrees(desired_yaw_with_offset):.1f} deg)")

        desired_yaw = desired_yaw_with_offset

        if abs(desired_yaw) > 0.5: #TODO: find problem of wrong tag being chosen as target instead of hard constraint
            self.internal_state = prev_state
            rospy.logwarn(f"[{self.node_name}] Desired yaw {desired_yaw:.3f} is too large, cannot align to AprilTag")
            return

        start_time = rospy.Time.now()
        rate = rospy.Rate(20)

        rospy.loginfo(f"[{self.node_name}] Aligning to AprilTag with final desired yaw: {desired_yaw:.3f} rad ({np.degrees(desired_yaw):.1f} deg)")

        while not rospy.is_shutdown():
            yaw_err = self.shortest_angle(desired_yaw - self.yaw)
            # rospy.loginfo(f"[{self.node_name}] Yaw error: {yaw_err:.3f} rad ({np.degrees(yaw_err):.1f} deg)")
            if abs(yaw_err) < self.align_tag_tolerance:
                break
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > self.align_tag_max_time:
                rospy.logwarn(f"[{self.node_name}] align_to_apriltag timed out after {elapsed:.2f}s, error: {yaw_err:.3f} rad")
                break
            # Proportional control with saturation
            omega = max(-self.align_tag_omega_max, min(self.align_tag_omega_max, self.align_tag_k * yaw_err))
            cmd = Twist2DStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.v = 0.0
            cmd.omega = omega
            self.car_cmd.publish(cmd)
            rate.sleep()

        # stop rotation
        stop_cmd = Twist2DStamped()
        stop_cmd.header.stamp = rospy.Time.now()
        stop_cmd.v = 0.0
        stop_cmd.omega = 0.0
        self.car_cmd.publish(stop_cmd)
        self.internal_state = prev_state

    # Calculate the pose that we want to navigate to relative to where we are. If we are using
    # the stop line pose then we need to calculate the stop line relative to the robot, and then the
    # goal pose relative to the stop line. If not using the stop line then we can use some fixed offset based on the
    # stop line distance? TODO
    def calculate_goal_trajectory(self):
        # self.stop_line_pose.theta = 0.0 # assumes robot is aligned
        g_stop_pose = self.ros_pose_to_geometry(self.stop_line_pose)
        rospy.loginfo(f"[unicorn_intersection_node] stop_line_pose: {self.stop_line_pose}")
        self.g_stop_pose_plan = g_stop_pose
        # TODO what if we don't want to use the stop_pose?

        # Step 1 - calculate the goal pose in the robot frame
        if self.turn_type == 0:
            canonical_goal_pose = self.goal_poses['left']
            self.num_waypoints = self.left_num_waypoints
            rospy.loginfo(f"[unicorn_intersection_node] We are turning left")
        elif self.turn_type == 1:
            canonical_goal_pose = self.goal_poses['straight']
            self.num_waypoints = self.straight_num_waypoints
            rospy.loginfo(f"[unicorn_intersection_node] We are going straigth")
        elif self.turn_type == 2:
            canonical_goal_pose = self.goal_poses['right']
            self.num_waypoints = self.right_num_waypoints
            rospy.loginfo(f"[unicorn_intersection_node] We are turning right")
        else:
            rospy.logerr("[unicorn_intersection_node] Something went wrong, invalid turn type")
        
        robot_frame_goal_pose = g.SE2.multiply( g.SE2.inverse(g_stop_pose), canonical_goal_pose)

        p, d = g.translation_angle_from_SE2(robot_frame_goal_pose)
        print(f"goal_pose in robot frame: position {p}, angle  {d}")

        # Step 2: Interpolate along the trajectory to generate waypoints
        waypoints = []
        directions = []

        if self.turn_type == 0 and self.use_left_turn_via_point:
            via_pose = self.dictionary_pose_to_geometry(self.left_turn_via_point)
            robot_frame_via_pose = g.SE2.multiply( g.SE2.inverse(g_stop_pose), via_pose)

            p, d = g.translation_angle_from_SE2(robot_frame_via_pose)
            rospy.loginfo(f"[unicorn_intersection_node] via_pose in robot frame: position {p}, angle  {d}")

            # Segment 1: Stop → Via (both in stop-line frame)
            seg1_count = max(1, self.left_num_waypoints // 2)
            seg2_count = max(1, self.left_num_waypoints - seg1_count)

            vel1 = g.SE2.algebra_from_group(robot_frame_via_pose)
            alphas1 = [x/seg1_count for x in range(1, seg1_count+1)]
            for alpha in alphas1:
                rel = g.SE2.group_from_algebra(vel1 * alpha)
                inter_pose = g.SE2.multiply(g_stop_pose, rel)
                position, direction = g.translation_angle_from_SE2(inter_pose)
                if position[0] < 0.0:
                    position = np.array([0.0, position[1]])
                waypoints.append(position)
                directions.append(direction)

            rospy.loginfo(f"[unicorn_intersection_node] 0")
            # Segment 2: Via → Goal (both in stop-line frame)
            via_to_goal_in_stop_frame = g.SE2.multiply(
                g.SE2.inverse(robot_frame_via_pose),  # Use stop-line frame via
                robot_frame_goal_pose                  # goal already in stop-line frame
            )
            rospy.loginfo(f"[unicorn_intersection_node] 1")
            vel2 = g.SE2.algebra_from_group(via_to_goal_in_stop_frame)
            rospy.loginfo(f"[unicorn_intersection_node] 2")
            alphas2 = [x/seg2_count for x in range(1, seg2_count+1)]
            rospy.loginfo(f"[unicorn_intersection_node] 3")
            for alpha in alphas2:
                rel = g.SE2.group_from_algebra(vel2 * alpha)
                # Compose: via + (alpha * via→goal displacement)
                waypoint_in_stop_frame = g.SE2.multiply(robot_frame_via_pose, rel)
                # Transform to world frame (matching segment 1 and other turns)
                inter_pose = g.SE2.multiply(g_stop_pose, waypoint_in_stop_frame)
                position, direction = g.translation_angle_from_SE2(inter_pose)
                if position[0] < 0.0:
                    position = np.array([0.0, position[1]])
                rospy.loginfo(f"[unicorn_intersection_node] 4")
                waypoints.append(position)
                directions.append(direction)
        else:
            vel = g.SE2.algebra_from_group(robot_frame_goal_pose)
            alphas = [x/self.num_waypoints for x in range(1, self.num_waypoints+1)]
            for alpha in alphas:
                rel = g.SE2.group_from_algebra(vel * alpha)
                inter_pose = g.SE2.multiply(g_stop_pose, rel)
                position, direction = g.translation_angle_from_SE2(inter_pose)
                # Prevent planning behind the stop line: clamp x >= 0 in the stop-line frame
                if position[0] < 0.0:
                    position = np.array([0.0, position[1]])
                print(f"Adding waypoint:  position {position}, angle {direction}")
                waypoints.append(position)
                directions.append(direction)

        # Step 3 (optional): Publish the trajectory for visualization in RVIZ
        if self.visualization:
            self.visualize_trajectory(waypoints, directions)
            self.publish_trajectory_debug_image(waypoints, directions)
        return waypoints

    def visualize_trajectory(self,waypoints, directions):
        for i in range(len(waypoints)):
            p = Odometry()
            p.header.frame_id = "map"
            p.header.stamp = rospy.Time.now()

            p.pose.pose.position.x = waypoints[i][0]
            p.pose.pose.position.y = waypoints[i][1]
            p.pose.pose.position.z = 0

            p.pose.pose.orientation.x = 0
            p.pose.pose.orientation.y = 0
            p.pose.pose.orientation.z = np.sin(directions[i] / 2)
            p.pose.pose.orientation.w = np.cos(directions[i] / 2)

            self.reference_trajectory_pub.publish(p)

    def reset_odometry(self):
        self.left_encoder_last = None
        self.right_encoder_last = None
        self.encoders_timestamp_last = None
        self.encoders_timestamp_last_local = None
        self.timestamp = None
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.q = [0.0, 0.0, 0.0, 1.0]
        self.tv = 0.0
        self.rv = 0.0

        self.ticks_per_meter = 656.0
        self.wheelbase = 0.108
        self.iter_ = 0
        self.final_state = 0

    def cb_ts_encoders(self, left_encoder, right_encoder):
        if self.internal_state not in ("EXECUTING", "ALIGNING"):
            return

        timestamp_now = rospy.get_time()

        # Use the average of the two encoder times as the timestamp
        left_encoder_timestamp = left_encoder.header.stamp.to_sec()
        right_encoder_timestamp = right_encoder.header.stamp.to_sec()
        timestamp = (left_encoder_timestamp + right_encoder_timestamp) / 2

        if not self.left_encoder_last:
            self.left_encoder_last = left_encoder
            self.right_encoder_last = right_encoder
            self.encoders_timestamp_last = timestamp
            self.encoders_timestamp_last_local = timestamp_now

        # Skip this message if the time synchronizer gave us an older message
        dtl = left_encoder.header.stamp - self.left_encoder_last.header.stamp
        dtr = right_encoder.header.stamp - self.right_encoder_last.header.stamp
        if dtl.to_sec() < 0 or dtr.to_sec() < 0:
            self.loginfo("Ignoring stale encoder message")
            return

        left_dticks = left_encoder.data - self.left_encoder_last.data
        right_dticks = right_encoder.data - self.right_encoder_last.data

        left_distance = left_dticks * 1.0 / self.ticks_per_meter
        right_distance = right_dticks * 1.0 / self.ticks_per_meter

        # Displacement in body-relative x-direction
        distance = (left_distance + right_distance) / 2

        # Change in heading
        dyaw = (right_distance - left_distance) / self.wheelbase

        dt = timestamp - self.encoders_timestamp_last

        if dt < 1e-6:
            dt = 1e-6

        self.tv = distance / dt
        self.rv = dyaw / dt

        if self.debug:
            self.loginfo(
                "Left wheel:\t Time = %.4f\t Ticks = %d\t Distance = %.4f m"
                % (left_encoder.header.stamp.to_sec(), left_encoder.data, left_distance)
            )

            self.loginfo(
                "Right wheel:\t Time = %.4f\t Ticks = %d\t Distance = %.4f m"
                % (right_encoder.header.stamp.to_sec(), right_encoder.data, right_distance)
            )

            self.loginfo(
                "TV = %.2f m/s\t RV = %.2f deg/s\t DT = %.4f" % (self.tv, self.rv * 180 / math.pi, dt)
            )

        dist = self.tv * dt
        dyaw = self.rv * dt

        self.yaw = self.angle_clamp(self.yaw + dyaw)
        self.x = self.x + dist * math.cos(self.yaw)
        self.y = self.y + dist * math.sin(self.yaw)
        self.q = tr.quaternion_from_euler(0, 0, self.yaw)
        self.timestamp = timestamp

        self.left_encoder_last = left_encoder
        self.right_encoder_last = right_encoder
        self.encoders_timestamp_last = timestamp
        self.encoders_timestamp_last_local = timestamp_now

        # Only run trajectory tracking when executing
        if self.internal_state != "EXECUTING":
            return

        car_control_msg = Twist2DStamped()
        #TODO
        car_control_msg.header.stamp = rospy.Time.now()
        car_control_msg.header.seq = 0

        # Add commands to car message
        gainV = 0.75
        wayPoint = self.reference_trajectory[self.iter_]
        car_control_msg.v = max(0.1, min(self.speed, gainV*(np.cos(self.yaw)*(wayPoint[0]-self.x)+np.sin(self.yaw)*(wayPoint[1])-self.y)))
        car_control_msg.omega = self.compute_omega(self.reference_trajectory[self.iter_],self.x,self.y,self.yaw,dt)
        self.car_cmd.publish(car_control_msg)

        # Get previous waypoint for along-track/cross-track check
        prev_point = None
        if self.iter_ > 0:
            prev_point = self.reference_trajectory[self.iter_ - 1]

        if self.check_point( np.array([self.x,self.y]),self.reference_trajectory[self.iter_], prev_point):
            if self.iter_ == 0:
                self.update_leds(['yellow', 'yellow', 'yellow', 'yellow', 'yellow']) #LED message to broadcast intersection navigation in progress
            self.iter_ += 1
            if self.iter_ == self.num_waypoints:
                self.update_leds(['green', 'green', 'green', 'green', 'green']) #LED message to broadcast intersection navigation complete
                self.internal_state = "READY"
                self.stop_line_pose_received = False
                self.turn_type_received = False
                # Publish intersection done
                msg_done = BoolStamped()
                msg_done.data = True
                self.pub_int_done.publish(msg_done)
                self.reset_odometry()
                self.priority_level = -1
                self.turn_type = -1
                self.negotiation_end = False
                self.led_priority_set = False
                self.led_ready_set = False
                self.current_led_pattern = None
                self.current_led_pattern_type = None
                rospy.loginfo("[unicorn intersection node] intersection navigation complete")


    @staticmethod
    def dictionary_pose_to_geometry(dict_param):
        return g.SE2_from_xytheta([dict_param['x'], dict_param['y'], dict_param['theta']])

    @staticmethod
    def ros_pose_to_geometry(ros_pose):
        return g.SE2_from_xytheta([ros_pose.x, ros_pose.y, ros_pose.theta])

    def cbTurnType(self, msg):
        """Process turn type decision from random_april_tag_turns_node."""
        if self.fsm_state == "NORMAL_JOYSTICK_CONTROL":
            return

        if self.turn_type_received:
            return

        self.turn_type = msg.turn_type
        self.turn_tag_id = getattr(msg, "tag_id", None)
        self.turn_type_received = True
        rospy.loginfo(f"[{self.node_name}] Received turn type: {self.turn_type} from tag {self.turn_tag_id}")

        # Stop robot to clear residual scanning commands
        stop_cmd = Twist2DStamped()
        stop_cmd.header.stamp = rospy.Time.now()
        stop_cmd.v = 0.0
        stop_cmd.omega = 0.0
        self.car_cmd.publish(stop_cmd)

        self.check_if_go()

    def setupParams(self):
        self.use_stop_pose = self.setupParam("~use_stop_pose", False)
        self.num_waypoints = self.setupParam("~num_waypoints", 2)
        self.visualization = self.setupParam("~visualization", True)
        default_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0 }
        self.canonical_goal_pose_right = self.setupParam("~canonical_goal_pose_right", default_pose)
        self.canonical_goal_pose_left = self.setupParam("~canonical_goal_pose_left", default_pose)
        self.canonical_goal_pose_straight = self.setupParam("~canonical_goal_pose_straight", default_pose)
        self.goal_blend_factor = self.setupParam("~goal_blend_factor", 1.0)
        # Intermediate "via point" for left turns to avoid cutting into opposite lane
        # This point is placed forward of the robot before the lateral turn begins
        default_left_via = {'x': 0.3, 'y': 0.0, 'theta': 0.0}
        self.left_turn_via_point = self.setupParam("~left_turn_via_point", default_left_via)
        self.use_left_turn_via_point = self.setupParam("~use_left_turn_via_point", True)
        self.speed = self.setupParam("~speed", 0.30)
        # Waypoint counts per maneuver
        self.left_num_waypoints = self.setupParam("~left_num_waypoints", 6)
        self.right_num_waypoints = self.setupParam("~right_num_waypoints", 4)
        self.straight_num_waypoints = self.setupParam("~straight_num_waypoints", 6)
        # Waypoint reach thresholds per maneuver
        self.left_x_threshold = self.setupParam("~left_x_threshold", 0.12)
        self.left_dist_threshold = self.setupParam("~left_dist_threshold", 0.12)
        self.right_x_threshold = self.setupParam("~right_x_threshold", 0.06)
        self.right_dist_threshold = self.setupParam("~right_dist_threshold", 0.08)
        self.straight_x_threshold = self.setupParam("~straight_x_threshold", 0.06)
        self.straight_dist_threshold = self.setupParam("~straight_dist_threshold", 0.08)
        # Distance past the stop line where trajectory should start (in meters)
        # Positive value = start trajectory after crossing stop line
        # Negative value = start trajectory before stop line
        self.stop_line_offset = self.setupParam("~stop_line_offset", 0.0)
        # Pre-alignment to Apriltag before executing the trajectory
        self.align_to_apriltag = self.setupParam("~align_to_apriltag", False)
        self.align_tag_tolerance = self.setupParam("~align_tag_tolerance", 0.15)  # radians
        self.align_tag_k = self.setupParam("~align_tag_k", 1.5)  # proportional gain
        self.align_tag_omega_max = self.setupParam("~align_tag_omega_max", 2.0)  # rad/s cap
        self.align_tag_max_time = self.setupParam("~align_tag_max_time", 2.5)  # seconds
        self.align_tag_heading_offset = self.setupParam("~align_tag_heading_offset", 0.0)  # radians offset from perpendicular
        self.align_tag_max_turn_angle = self.setupParam("~align_tag_max_turn_angle", math.pi / 2)  # radians, max turn before reversing direction
        # AprilTag scan parameters (when tag not initially visible)
        self.align_tag_scan_angle = self.setupParam("~align_tag_scan_angle", 0.4)  # radians (~23 deg)
        self.align_tag_scan_omega = self.setupParam("~align_tag_scan_omega", 0.3)  # rad/s
        self.align_tag_scan_wait = self.setupParam("~align_tag_scan_wait", 0.3)  # seconds

    def updateParams(self, event):
        pass

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        #rospy.loginfo(f"[{self.node_name}] {param_name} = {value} ")
        return value

    def onShutdown(self):
        rospy.loginfo("[UnicornIntersectionNode] Shutdown.")

    @staticmethod
    def angle_interp(theta0, theta1, alpha):
        """Interpolate angles along the shortest arc."""
        # wrap difference to [-pi, pi]
        d = (theta1 - theta0 + np.pi) % (2 * np.pi) - np.pi
        return theta0 + alpha * d

    def interpolate_segment(self, g_stop_pose, end_pose, num_points):
        """Interpolate SE(2) trajectory between two poses."""
        robot_frame_goal_pose = g.SE2.multiply(g.SE2.inverse(g_stop_pose), end_pose)
        vel = g.SE2.algebra_from_group(robot_frame_goal_pose)
        alphas = [x/num_points for x in range(1, num_points+1)]
        waypoints = []
        directions = []
        for alpha in alphas:
            rel = g.SE2.group_from_algebra(vel * alpha)
            inter_pose = g.SE2.multiply(g_stop_pose, rel)
            position, direction = g.translation_angle_from_SE2(inter_pose)
            # Prevent planning behind the stop line: clamp x >= 0 in the stop-line frame
            if position[0] < 0.0:
                position = np.array([0.0, position[1]])
            print(f"Adding waypoint:  position {position}, angle {direction}")
            waypoints.append(position)
            directions.append(direction)

        return waypoints, directions

    def publish_trajectory_debug_image(self, waypoints, directions):
        """
        Create and publish a debug image showing the trajectory waypoints and path
        """
        # Create a blank image (800x800 pixels, white background)
        img_size = 800
        img = np.ones((img_size, img_size, 3), dtype=np.uint8) * 255

        # Scale factor to convert meters to pixels (adjust based on typical trajectory size)
        # Assuming trajectories are roughly -1 to 1 meters, we'll use center of image as origin
        scale = 200  # pixels per meter
        center_x = img_size // 2
        center_y = img_size // 2

        def to_pixel_coords(point):
            """Convert from meters (robot frame) to pixel coordinates"""
            px = int(center_x + point[0] * scale)
            py = int(center_y - point[1] * scale)  # Flip y-axis for image coordinates
            return (px, py)

        # Draw grid lines for reference
        grid_step = 0.25  # meters
        for i in np.arange(-2, 2.1, grid_step):
            # Vertical lines
            x_px = int(center_x + i * scale)
            cv2.line(img, (x_px, 0), (x_px, img_size), (230, 230, 230), 1)
            # Horizontal lines
            y_px = int(center_y - i * scale)
            cv2.line(img, (0, y_px), (img_size, y_px), (230, 230, 230), 1)

        # Draw axes
        cv2.line(img, (center_x, 0), (center_x, img_size), (200, 200, 200), 2)  # Y-axis
        cv2.line(img, (0, center_y), (img_size, center_y), (200, 200, 200), 2)  # X-axis

        # Draw an intersection layout similar to simulation: red stop lines on each approach,
        # yellow center lines through the intersection.
        stop_offset = 0.585 / 2.0  # distance from center to each stop line
        stop_len = 0.585 / 2.0
        stop_half = stop_len / 2.0
        x_offset = 0.585 / 2.0
        y_offset = 0.585 / 4.0

        def draw_stop_line(cx, cy, heading, color, thickness=3):
            dx = stop_half * np.cos(heading + np.pi / 2)
            dy = stop_half * np.sin(heading + np.pi / 2)
            p1 = to_pixel_coords((cx - dx + x_offset, cy - dy))
            p2 = to_pixel_coords((cx + dx + x_offset, cy + dy))
            cv2.line(img, p1, p2, color, thickness)

        # Stop lines (red) for four approaches
        draw_stop_line(-x_offset, -stop_half + y_offset, 0.0, (0, 0, 255))         # coming from left
        draw_stop_line(x_offset, stop_half + y_offset, np.pi, (0, 0, 255))        # coming from right
        draw_stop_line(stop_half, -stop_offset + y_offset, np.pi / 2, (0, 0, 255))   # coming from bottom
        draw_stop_line(-stop_half, stop_offset + y_offset, -np.pi / 2, (0, 0, 255))   # coming from top

        # Center lines (yellow) extending away from the center
        center_len = 0.5
        center_color = (0, 255, 255)

        cv2.line(img, to_pixel_coords((x_offset*2, y_offset)), to_pixel_coords((center_len + x_offset*2, y_offset)), center_color, 2)
        cv2.line(img, to_pixel_coords((0.0, y_offset)), to_pixel_coords((-center_len, y_offset)), center_color, 2)
        cv2.line(img, to_pixel_coords((x_offset, stop_offset + y_offset)), to_pixel_coords((x_offset, stop_offset + center_len + y_offset)), center_color, 2)
        cv2.line(img, to_pixel_coords((x_offset, -y_offset)), to_pixel_coords((x_offset, -center_len - y_offset)), center_color, 2)

        # Draw robot position(s) (transformed into stop-line frame for visualization)
        if self.g_stop_pose_plan is not None:
            # Draw the inverse of the stop pose as a coordinate frame
            norm_pos, norm_heading = g.translation_angle_from_SE2(self.g_stop_pose_plan)
            norm_px = to_pixel_coords(norm_pos)
            cv2.circle(img, norm_px, 12, (0, 200, 0), -1)  # Pale green
            arrow_length = 30
            end_x = int(norm_px[0] + arrow_length * np.cos(norm_heading))
            end_y = int(norm_px[1] - arrow_length * np.sin(norm_heading))
            cv2.arrowedLine(img, norm_px, (end_x, end_y), (180, 230, 180), 2, tipLength=0.3)
            cv2.putText(img, "Duckiebot", (norm_px[0] - 20, norm_px[1] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 120, 80), 2)

        # Draw ideal trajectory path (connecting lines) - what's planned
        if len(waypoints) > 1:
            for i in range(len(waypoints) - 1):
                pt1 = to_pixel_coords(waypoints[i])
                pt2 = to_pixel_coords(waypoints[i + 1])
                cv2.line(img, pt1, pt2, (255, 0, 0), 3)  # Blue line for ideal trajectory

        # Draw waypoints with direction arrows
        for i, (wp, direction) in enumerate(zip(waypoints, directions)):
            pixel_pos = to_pixel_coords(wp)

            # Draw waypoint circle
            color = (0, 0, 255) if i < len(waypoints) - 1 else (255, 0, 255)  # Red for waypoints, magenta for final
            cv2.circle(img, pixel_pos, 8, color, -1)

            # Draw direction arrow
            arrow_length = 30
            end_x = int(pixel_pos[0] + arrow_length * np.cos(direction))
            end_y = int(pixel_pos[1] - arrow_length * np.sin(direction))  # Flip y for image coords
            cv2.arrowedLine(img, pixel_pos, (end_x, end_y), color, 2, tipLength=0.3)

            # Add waypoint number
            cv2.putText(img, str(i), (pixel_pos[0] + 10, pixel_pos[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        # Add title and information
        turn_type_str = {0: "LEFT", 1: "STRAIGHT", 2: "RIGHT"}.get(self.turn_type, "UNKNOWN")
        cv2.putText(img, f"Intersection Trajectory - Turn: {turn_type_str}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        cv2.putText(img, f"Waypoints: {len(waypoints)}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)

        # Add legend
        legend_y = 30
        cv2.circle(img, (img_size - 150, legend_y), 8, (0, 0, 255), -1)
        cv2.putText(img, "Waypoint", (img_size - 130, legend_y + 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

        legend_y += 20
        cv2.circle(img, (img_size - 150, legend_y), 8, (255, 0, 255), -1)
        cv2.putText(img, "Goal", (img_size - 130, legend_y + 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

        legend_y += 20
        cv2.line(img, (img_size - 155, legend_y), (img_size - 145, legend_y), (255, 0, 0), 3)
        cv2.putText(img, "Ideal Traj", (img_size - 130, legend_y + 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

        # Convert to ROS CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', img)[1]).tobytes()

        # Publish the debug image
        self.pub_debug_trajectory_img.publish(msg)
        rospy.loginfo(f"[{self.node_name}] Published trajectory debug image with {len(waypoints)} waypoints")
    
    def shortest_angle(self, theta):
        """Wrap angle to [-pi, pi]."""
        return (theta + math.pi) % (2 * math.pi) - math.pi

    def angle_clamp(self, theta):
        if theta > 2 * math.pi:
            return theta - 2 * math.pi
        elif theta < -2 * math.pi:
            return theta + 2 * math.pi
        else:
            return theta      

    def path_plan(self,obstacle,lane):
            return 0

    def compute_omega(self,targetxy,x,y,current,dt):
        factor = 0.75 # PARAM 
        target_yaw = np.arctan2( (targetxy[1] - y),(targetxy[0]- x) )
        omega = factor* ((target_yaw - current))

        return omega

    def check_point(self, current_point, target_point, prev_point = None):
        threshold = 0.1
        threshold_x = 0.08
        dist_x = np.zeros((1,2))
        dist_x[0, 0] = (current_point[0] - self.alpha) - target_point[0]
        dist_x[0, 1] = (current_point[1]) - target_point[1]
        current_x = current_point[0]
        current_y = current_point[1]
        if self.iter_ == (self.num_waypoints - 1):
            # Last waypoint: check if close enough or if passed it along travel direction
            dist = np.sqrt((current_x - target_point[0])**2 + (current_y - target_point[1])**2)
            if dist < threshold:
                rospy.loginfo(f"[{self.node_name}] Reached waypoint {self.iter_}, dist: {dist}, threshold: {threshold}")
                return True

            # Check if robot has passed the waypoint along travel direction (prevents oscillation)
            if prev_point is not None:
                # Segment direction from prev_point to target_point
                seg_x = target_point[0] - prev_point[0]

                seg_y = target_point[1] - prev_point[1]
                seg_length = np.sqrt(seg_x**2 + seg_y**2)

                if seg_length >= 1e-6:
                    # Unit tangent along segment
                    t_x = seg_x / seg_length
                    t_y = seg_y / seg_length

                    # Vector from prev waypoint to robot
                    v_x = current_x - prev_point[0]
                    v_y = current_y - prev_point[1]

                    # Along-track progress (signed distance along segment direction)
                    s = v_x * t_x + v_y * t_y
                    # Advance if passed the waypoint along travel direction
                    if s >= seg_length:
                        #rospy.loginfo(f"[{self.node_name}] Reached waypoint {self.iter_}, s: {s}, seg_length: {seg_length}")
                        return True

                    # print -np.sign(target_point[0] - self.reference_trajectory[0][0]) * (current_x - target_point[0]) > threshold_x
                    direction = np.sign(target_point[0] - self.reference_trajectory[0][0])  # +1 if goal x ahead, -1 if behind
                    signed_overrun = direction * (current_x - target_point[0])

                    if signed_overrun > threshold_x:
                        #rospy.loginfo(f"[{self.node_name}] Reached waypoint {self.iter_}, overrun_x: {signed_overrun:.3f}, threshold_x: {threshold_x}")
                        return True
            return False

        else:
            dist = np.sqrt(((current_point[0]-self.alpha) - target_point[0])**2 + ((current_point[1]-self.alpha) - target_point[1])**2 )
            # rospy.loginfo(f"[{self.node_name}] dist: {dist}, threshold: {threshold}")
            # If close enough to waypoint, advance
            if dist < threshold:
                return True

            # Special handling for waypoints very close to origin (e.g., (0,0))
            # which can cause issues with direction calculations
            if np.sqrt(target_point[0]**2 + target_point[1]**2) < 0.05:  # waypoint within 5cm of origin
                # For near-origin waypoints, just use distance check with relaxed threshold
                if dist < threshold * 1.5:
                    #rospy.loginfo(f"[{self.node_name}] Near-origin waypoint, dist: {dist:.3f}, relaxed threshold: {threshold * 1.5:.3f}")
                    return True

            # If no previous point, fall back to distance check only
            if prev_point is None:
                return False

            # Along-track/cross-track decomposition
            # Segment direction from prev_point to target_point
            seg_x = target_point[0] - prev_point[0]
            seg_y = target_point[1] - prev_point[1]
            seg_length = np.sqrt(seg_x**2 + seg_y**2)

            # Handle degenerate segment (waypoints too close together)
            if seg_length < 1e-6:
                # If segment is degenerate, just check distance to target
                if dist < threshold * 1.2:
                    #rospy.loginfo(f"[{self.node_name}] Degenerate segment, using distance check")
                    return True
                return False

            # Unit tangent along segment
            t_x = seg_x / seg_length
            t_y = seg_y / seg_length

            # Vector from prev waypoint to robot
            v_x = current_x - prev_point[0]
            v_y = current_y - prev_point[1]

            # Along-track progress (signed distance along segment direction)
            s = v_x * t_x + v_y * t_y
            # Cross-track error (lateral distance to segment line)
            cross_track_x = v_x - s * t_x
            cross_track_y = v_y - s * t_y
            e = np.sqrt(cross_track_x**2 + cross_track_y**2)
            # Advance if passed the waypoint along travel direction
            if s >= seg_length:
                rospy.loginfo(f"[{self.node_name}] Passed waypoint along track, s: {s:.3f}, seg_length: {seg_length:.3f}")
                return True

            # Advance if close to segment line and near the end (helps when cutting corners)
            if e < threshold_x and s >= 0.90 * seg_length:
                rospy.loginfo(f"[{self.node_name}] Near waypoint, e: {e:.3f}, s/seg_length: {s/seg_length:.2f}")
                return True
            return False

    def set_led_pattern(self, pattern_name, force=False):
        """
        Set a predefined LED pattern by name (e.g., "BLUE", "GREEN", "YELLOW", "RED").
        
        Args:
            pattern_name: String name of the predefined pattern
        """
        if self.led_pattern_service is None:
            rospy.logwarn(f"[{self.node_name}] LED pattern service not available, cannot set pattern: {pattern_name}")
            return

        # Skip if already set to this pattern and not forcing an update
        if (not force and self.current_led_pattern_type == 'predefined'
                and self.current_led_pattern == pattern_name):
            return
        
        pattern_msg = String()
        pattern_msg.data = pattern_name
        try:
            self.led_pattern_service(pattern_name=pattern_msg)
            self.current_led_pattern = pattern_name
            self.current_led_pattern_type = 'predefined'
            #rospy.loginfo(f"[{self.node_name}] Set LED pattern to: {pattern_name}")
        except rospy.ServiceException as e:
            rospy.logerr(f"[{self.node_name}] Failed to set LED pattern {pattern_name}: {e}")

    def update_leds(self, color_list, frequency=0.0, force=False):
        """
        Set a custom LED pattern using a list of colors.

        Args:
            color_list: List of 5 color names (e.g., ['red', 'blue', 'white', 'green', 'yellow'])
            frequency: Blinking frequency in Hz (0.0 for solid)
        """
        if self.led_custom_pattern_service is None:
            rospy.logwarn(f"[{self.node_name}] LED custom pattern service not available, cannot set custom pattern")
            return

        # Skip if this exact pattern is already active and not forcing an update
        normalized_pattern = (tuple(color_list), frequency)
        if (not force and self.current_led_pattern_type == 'custom'
                and self.current_led_pattern == normalized_pattern):
            return
        
        pattern_msg = LEDPattern()
        # We assign the list of 5 colors
        pattern_msg.color_list = color_list 
        pattern_msg.frequency = frequency
        # We activate all the leds (no flickering by default)
        pattern_msg.color_mask = [1, 1, 1, 1, 1]
        pattern_msg.frequency_mask = [0, 0, 0, 0, 0]
        
        try:
            self.led_custom_pattern_service(pattern=pattern_msg)
            self.current_led_pattern = normalized_pattern
            self.current_led_pattern_type = 'custom'
            #rospy.loginfo(f"[{self.node_name}] Set custom LED pattern: {color_list}")
        except rospy.ServiceException as e:
            rospy.logerr(f"[{self.node_name}] Failed to set custom LED pattern: {e}")

    def _reapply_led_pattern(self, event=None, force=False):
        """Re-apply the current LED pattern (used after FSM state changes or timer callbacks)"""
        if self.current_led_pattern is None:
            return
        
        if self.current_led_pattern_type == 'predefined':
            self.set_led_pattern(self.current_led_pattern, force=force)
        elif self.current_led_pattern_type == 'custom':
            color_list, frequency = self.current_led_pattern
            self.update_leds(list(color_list), frequency, force=force)

    def _start_led_pattern_timer(self):
        """Start a timer to periodically re-apply LED pattern during execution"""
        self._stop_led_pattern_timer()  # Stop any existing timer
        self.led_pattern_timer = rospy.Timer(rospy.Duration(0.5), self._reapply_led_pattern)

    def _stop_led_pattern_timer(self):
        """Stop the LED pattern re-application timer"""
        if self.led_pattern_timer is not None:
            self.led_pattern_timer.shutdown()
            self.led_pattern_timer = None

    def get_led_pattern(self, priority_level, direction_index, is_ready=False):
        """
        priority_level: int (1-4)
        direction_index: int (0-2)
        is_ready: bool (If True, the right LED turns green to confirm the start)
        """
        # 1. Priority color (LED 0 - Left front)
        priority_color = self.priority_colors.get(priority_level, 'white')

        # 2. Direction color (LED 4 - Right front)
        if is_ready:
            direction_color = 'green'
        else:
            direction_color = self.direction_colors.get(direction_index, 'white')

        # 3. Compose the pattern [FL, RL, TOP, RR, FR]
        pattern = [
            priority_color,  # Index 0: Front Left
            'switchedoff',   # Index 1: Rear Left
            'switchedoff',   # Index 2: Top
            'switchedoff',   # Index 3: Rear Right
            direction_color  # Index 4: Front Right
        ]
        
        return pattern

    def intersection_planning(self):
        if self.priority_level == -1:
            self.priority_level = self.get_priority()
            Led_pattern = self.get_led_pattern(priority_level= self.priority_level, direction_index= self.turn_type)
            self.update_leds(Led_pattern)
            rospy.loginfo(f"[unicorn_intersection_node] Set priority to: {self.priority_level} ")

        if(self.stop_line_pose_received and self.turn_type_received and self.internal_state == "READY"):
            Led_pattern = self.get_led_pattern(priority_level= self.priority_level, direction_index= self.turn_type)
            self.update_leds(Led_pattern)
            #TODO generate the intersection scenario
            rospy.Duration(1.0)#emulate the time of scenario computing
            rospy.loginfo(f"[unicorn_intersection_node] Intersection scenario ready")
            Led_pattern = self.get_led_pattern(priority_level= self.priority_level, direction_index= self.turn_type, is_ready=True)
            self.update_leds(Led_pattern)
            while self.negotiation_end != True:
                #TODO control the start posibility according to scenario
                rospy.Duration(1.0)#emulate the time of control
                self.negotiation_end = True

    def get_priority(self):
        """
        Get the priority of the vehicule by counting the number of duckiebot in the intersection
        :param self: Description
        """

        #TODO implement a way to count vehicules in the intersection
        prio = 1
        return prio

if __name__ == "__main__":
    unicorn_intersection_node = UnicornIntersectionNode(node_name="unicorn_intersection_node")
    rospy.on_shutdown(unicorn_intersection_node.onShutdown)
    rospy.spin()
