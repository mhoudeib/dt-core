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
    AprilTagDetectionArray
    )
from duckietown_msgs.srv import ChangePattern, SetCustomLEDPattern
from std_msgs.msg import String


from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
import math 
from geometry_msgs.msg import Quaternion, Twist, Pose2D, Point, Vector3, TransformStamped, Transform

from nav_msgs.msg import Odometry

import message_filters
from tf import transformations as tr

import geometry as g

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


        ## Subscribers
        self.sub_turn_type = rospy.Subscriber("~turn_id_and_type", TurnIDandType, self.cbTurnType)
        self.sub_encoder_left = message_filters.Subscriber("~left_wheel_encoder_driver_node/tick", WheelEncoderStamped)
        self.sub_encoder_right = message_filters.Subscriber("~right_wheel_encoder_driver_node/tick", WheelEncoderStamped)
        self.sub_stop_line_reading = rospy.Subscriber("~stop_line_reading", StopLineReading, self.cbStopLineReading)
        # Raw Apriltag detections (AprilTagDetectionArray) from apriltag_detector_node
        self.sub_apriltags = rospy.Subscriber("~detections", AprilTagDetectionArray, self.cbApriltags, queue_size=1)
        self.sub_fsm_node_mode = rospy.Subscriber(
            name='fsm_node/mode',   # The full, resolved topic name
            data_class=FSMState,     # The message type it expects to receive
            callback=self.onFSMStateChange, 
            queue_size=1
        )


        ## Publisher
        self.pub_int_done = rospy.Publisher("~intersection_done", BoolStamped, queue_size=1)
        self.car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)
        self.reference_trajectory_pub = rospy.Publisher(
            "~reference_trajectory",
            Odometry,
            queue_size=self.num_waypoints,
        )

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

        self.log("Initialialized unicorn intersection node")

    def cbStopLineReading(self, msg):
        if self.stop_line_pose_received:
            return

        if msg.at_stop_line:
            self.stop_line_pose = msg.stop_pose
            self.stop_line_pose_received = True
            rospy.loginfo(f"[unicorn_intersection_node] Received stop line pose: {self.stop_line_pose}")
            self.check_if_go()

    def check_if_go(self):
        if (self.stop_line_pose_received and self.turn_type_received and self.internal_state == "READY"):
            rospy.loginfo("[unicorn_intersection_node] We have what we need, calculating reference trajectory")
            self.reference_trajectory = self.calculate_goal_trajectory()
            rospy.loginfo(f"[unicorn_intersection_node] Reference trajectory calculated: {self.reference_trajectory}")
            # Optional pre-alignment to Apriltag for this turn
            if self.align_to_apriltag:
                self.align_to_apriltag_heading()
            self.intersection_planning()
            if self.negotiation_end and self.internal_state != "EXECUTING":
                car_control_msg = Twist2DStamped()
                car_control_msg.header.stamp = rospy.Time.now()
                car_control_msg.header.seq = 0
                car_control_msg.v = 0
                car_control_msg.omega = 0
                self.car_cmd.publish(car_control_msg)
                rospy.loginfo(f"[unicorn_intersection_node] We start intersection navigation")
                self.internal_state = "EXECUTING"
        else:
            rospy.loginfo(f"[unicorn_intersection_node] We don't have what we need yet: "
                      f"stop_line received: {self.stop_line_pose_received} " 
                      f"turn_type_received: {self.turn_type_received} "
                      f"internal_state:{self.internal_state} ")
            
    def cbApriltags(self, msg):
        """Store latest Apriltag detections (raw AprilTagDetectionArray) for alignment."""
        if len(msg.detections) == 0:
            return
        self.apriltag_detections = {}
        for detection in msg.detections:
            self.apriltag_detections[detection.tag_id] = detection

    def align_to_apriltag_heading(self):
        """Rotate robot to face the Apriltag of the current turn type before executing."""
        tag_id = self.turn_tag_id
        if tag_id is None:
            rospy.logwarn(f"[{self.node_name}] align_to_apriltag enabled but no tag_id provided with turn_type.")
            return
        detection = self.apriltag_detections.get(tag_id)
        if detection is None:
            rospy.logwarn(f"[{self.node_name}] align_to_apriltag enabled but tag {tag_id} not currently detected.")
            return

        prev_state = self.internal_state
        self.internal_state = "ALIGNING"

        pos = detection.transform.translation
        desired_yaw = math.atan2(pos.y, pos.x)  # face the tag

        start_time = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            yaw_err = self.shortest_angle(desired_yaw - self.yaw)
            if abs(yaw_err) < self.align_tag_tolerance:
                break
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > self.align_tag_max_time:
                rospy.logwarn(f"[{self.node_name}] align_to_apriltag timed out after {elapsed:.2f}s")
                break
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
        g_stop_pose = self.ros_pose_to_geometry(self.stop_line_pose)
        # TODO what if we don't want to use the stop_pose?

        # TODO this should really be turned into an enum
        # Step 1 - calculate the goal pose in the robot frame
        if self.turn_type == 0:
            canonical_goal_pose = self.goal_poses['left']
            rospy.loginfo(f"[unicorn_intersection_node] We are turning left")
        elif self.turn_type == 1:
            canonical_goal_pose = self.goal_poses['straight']
            rospy.loginfo(f"[unicorn_intersection_node] We are going straigth")
        elif self.turn_type == 2:
            canonical_goal_pose = self.goal_poses['right']
            rospy.loginfo(f"[unicorn_intersection_node] We are turning right")
        else:
            rospy.logerr("[unicorn_intersection_node] Something went wrong, invalid turn type")

        # Blend between stop-aligned goal and canonical goal to reduce over-adjustment
        blended_goal = self.blend_goal_pose(g_stop_pose, canonical_goal_pose, self.goal_blend_factor)
        robot_frame_goal_pose = g.SE2.multiply(g.SE2.inverse(g_stop_pose), blended_goal)

        p, d = g.translation_angle_from_SE2(robot_frame_goal_pose)
        print(f"goal_pose in robot frame: position {p}, angle  {d}")

        # Store goal pose in robot frame for visualization
        self.robot_frame_goal_pose = robot_frame_goal_pose

        waypoints = []
        directions = []

        if self.turn_type == 0 and self.use_left_turn_via_point:
            via_pose = self.dictionary_pose_to_geometry(self.left_turn_via_point)
            blended_via = self.blend_goal_pose(g_stop_pose, via_pose, self.goal_blend_factor)
            robot_frame_via_pose = g.SE2.multiply( g.SE2.inverse(g_stop_pose), blended_via)

            p, d = g.translation_angle_from_SE2(robot_frame_via_pose)
            rospy.loginfo(f"[unicorn_intersection_node] via_pose in robot frame: position {p}, angle  {d}")

            seg1_count = max(1, self.left_num_waypoints // 2)
            seg2_count = max(1, self.left_num_waypoints - seg1_count)

            w1, d1 = self.interpolate_segment(g_stop_pose, robot_frame_via_pose, seg1_count)
            w2, d2 = self.interpolate_segment(robot_frame_via_pose, robot_frame_goal_pose, seg2_count)
            waypoints.extend(w1 + w2)
            directions.extend(d1 + d2)
        else:
            w, d = self.interpolate_segment(g_stop_pose, robot_frame_goal_pose, self.num_waypoints)
            waypoints.extend(w)
            directions.extend(d)

        # Transform waypoints to odometry frame
        # odom_T_robot = g.SE2_from_xytheta([self.x, self.y, self.yaw])
        # waypoints_odom = []
        # for wp, dir in zip(waypoints, directions):
        #     wp_robot = g.SE2_from_xytheta([wp[0], wp[1], dir])
        #     wp_odom = g.SE2.multiply(odom_T_robot, wp_robot)
        #     pos, _ = g.translation_angle_from_SE2(wp_odom)
        #     waypoints_odom.append(pos)
        # rospy.loginfo(f"[unicorn_intersection_node] waypoints_odom: {waypoints_odom}")

        if self.visualization:
            self.visualize_trajectory(waypoints, directions)
            self.last_waypoints = waypoints
            self.last_directions = directions
        else:
            self.last_waypoints = []
            self.last_directions = []

        self.publish_path_and_markers(waypoints, directions, g_stop_pose)
        return waypoints  # Return waypoints!

    def blend_goal_pose(self, g_stop_pose, canonical_goal_pose, blend):
        """
        Blend between the stop-aligned goal (stop frame) and the canonical goal pose.
        blend=1.0 => fully adjusted by stop pose (current behavior)
        blend=0.0 => ignore stop pose adjustment, use canonical directly
        
        Returns blended goal pose in the same frame as canonical_goal_pose.
        """
        blend = max(0.0, min(1.0, blend))
        
        # If blend=0, return canonical goal as-is
        if blend == 0.0:
            return canonical_goal_pose
        
        # Extract translation and angle from canonical goal pose
        t_canonical, theta_canonical = g.translation_angle_from_SE2(canonical_goal_pose)
        
        # Get canonical goal in stop frame (this represents the stop-aligned version)
        stop_T_goal = g.SE2.multiply(g.SE2.inverse(g_stop_pose), canonical_goal_pose)
        t_goal_stop, theta_goal_stop = g.translation_angle_from_SE2(stop_T_goal)
        
        # Transform stop frame representation back to canonical frame
        # This gives us the stop-aligned goal in canonical frame
        goal_stop_aligned = g.SE2.multiply(g_stop_pose, stop_T_goal)
        t_stop_aligned, theta_stop_aligned = g.translation_angle_from_SE2(goal_stop_aligned)
        
        # Blend translation and angle between canonical and stop-aligned versions
        t_blend = (1 - blend) * np.array(t_canonical) + blend * np.array(t_stop_aligned)
        theta_blend = self.angle_interp(theta_canonical, theta_stop_aligned, blend)
        
        return g.SE2_from_xytheta([t_blend[0], t_blend[1], theta_blend])

    @staticmethod
    def angle_interp(theta0, theta1, alpha):
        """Interpolate angles along the shortest arc."""
        # wrap difference to [-pi, pi]
        d = (theta1 - theta0 + np.pi) % (2 * np.pi) - np.pi
        return theta0 + alpha * d

    def interpolate_segment(self, start_pose, end_pose, num_points):
        """Interpolate SE(2) trajectory between two poses."""
        robot_frame_goal_pose = g.SE2.multiply(g.SE2.inverse(start_pose), end_pose)
        vel = g.SE2.algebra_from_group(robot_frame_goal_pose)
        alphas = [x/self.num_waypoints for x in range(1, self.num_waypoints+1)]
        waypoints = []
        directions = []
        for alpha in alphas:
            rel = g.SE2.group_from_algebra(vel * alpha)
            inter_pose = g.SE2.multiply(g_stop_pose, rel)
            position, direction = g.translation_angle_from_SE2(inter_pose)
            print(f"Adding waypoint:  position {position}, angle {direction}")
            waypoints.append(position)
            directions.append(direction)

        # Step 3 (optional): Publish the trajectory for visualization in RVIZ
        if self.visualization:
            self.visualize_trajectory(waypoints, directions)
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
            return

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
        car_control_msg.v = min(self.speed, gainV*(np.cos(self.yaw)*(wayPoint[0]-self.x)+np.sin(self.yaw)*(wayPoint[1])-self.y))
        car_control_msg.omega = self.compute_omega(self.reference_trajectory[self.iter_],self.x,self.y,self.yaw,dt)
        self.car_cmd.publish(car_control_msg)

        if self.check_point( np.array([self.x,self.y]),self.reference_trajectory[self.iter_] ):
            self.iter_ += 1
            if self.iter_ == self.num_waypoints:
                self.internal_state = "READY"
                self.stop_line_pose_received = False
                self.turn_type_received = False
                # Publish intersection done
                msg_done = BoolStamped()
                msg_done.data = True
                self.pub_int_done.publish(msg_done)
                self.reset_odometry()
                rospy.loginfo("[unicorn intersection node] intersection navigation complete")




    @staticmethod
    def dictionary_pose_to_geometry(dict_param):
        return g.SE2_from_xytheta([dict_param['x'], dict_param['y'], dict_param['theta']])

    @staticmethod
    def ros_pose_to_geometry(ros_pose):
        return g.SE2_from_xytheta([ros_pose.x, ros_pose.y, ros_pose.theta])

    def cbTurnType(self, msg):
        if self.turn_type_received:
            return

        self.turn_type = msg.turn_type
        # Store tag id if provided (used for Apriltag alignment)
        self.turn_tag_id = getattr(msg, "tag_id", None)
        self.turn_type_received = True
        rospy.loginfo(f"[unicorn_intersection_node] Received turn type: {self.turn_type} from tag {self.turn_tag_id}")

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

    def updateParams(self, event):
        pass

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo(f"[{self.node_name}] {param_name} = {value} ")
        return value

    @staticmethod
    def shortest_angle(theta):
        """Wrap angle to [-pi, pi]."""
        return (theta + math.pi) % (2 * math.pi) - math.pi

    def onShutdown(self):
        rospy.loginfo("[UnicornIntersectionNode] Shutdown.")

    @staticmethod
    def angle_clamp(theta):
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

    def check_point(self, current_point, target_point):
        threshold = 0.1
        threshold_x = 0.08
        dist_x = np.zeros((1,2))
        dist_x[0, 0] = (current_point[0] - self.alpha) - target_point[0]
        dist_x[0, 1] = (current_point[1]) - target_point[1]
        if self.iter_ == (self.num_waypoints - 1):
            if abs(dist_x[0, 1]) < threshold_x:
                return True

            return False

        else:
            dist = np.sqrt(((current_point[0]-self.alpha) - target_point[0])**2 + ((current_point[1]-self.alpha) - target_point[1])**2 )

            if (abs(dist_x[0,0])) > threshold_x or (dist) < threshold:
                return True

            return False

if __name__ == "__main__":
    unicorn_intersection_node = UnicornIntersectionNode(node_name="unicorn_intersection_node")
    rospy.on_shutdown(unicorn_intersection_node.onShutdown)
    rospy.spin()
