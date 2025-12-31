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
    FSMState
    )


from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
import math 
from geometry_msgs.msg import Quaternion, Twist, Pose2D, Point, Vector3, TransformStamped, Transform
from duckietown_msgs.srv import SetCustomLEDPattern, SetCustomLEDPatternRequest, ChangePattern

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


        ## Subscribers
        self.sub_turn_type = rospy.Subscriber("~turn_id_and_type", TurnIDandType, self.cbTurnType)
        self.sub_encoder_left = message_filters.Subscriber("~left_wheel_encoder_driver_node/tick", WheelEncoderStamped)
        self.sub_encoder_right = message_filters.Subscriber("~right_wheel_encoder_driver_node/tick", WheelEncoderStamped)
        self.sub_stop_line_reading = rospy.Subscriber("~stop_line_reading", StopLineReading, self.cbStopLineReading)

        ## Publisher
        self.pub_int_done = rospy.Publisher("~intersection_done", BoolStamped, queue_size=1)
        self.car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)
        self.reference_trajectory_pub = rospy.Publisher(
            "~reference_trajectory",
            Odometry,
            queue_size=self.num_waypoints,
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
            self.intersection_planning()
            rospy.loginfo(f"[unicorn_intersection_node] We don't have what we need yet: "
                      f"stop_line received: {self.stop_line_pose_received} " 
                      f"turn_type_received: {self.turn_type_received} "
                      f"internal_state:{self.internal_state} ")

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

        robot_frame_goal_pose = g.SE2.multiply( g.SE2.inverse(g_stop_pose), canonical_goal_pose)

        p, d = g.translation_angle_from_SE2(robot_frame_goal_pose)
        print(f"goal_pose in robot frame: position {p}, angle  {d}")

        # Step 2: Interpolate along the trajectory to generate waypoints
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
        if self.internal_state != "EXECUTING":
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
        if self.turn_type_received:
            return

        self.turn_type = msg.turn_type
        self.turn_type_received = True
        rospy.loginfo(f"[unicorn_intersection_node] Received turn type: {self.turn_type} ")
        self.check_if_go()

    def setupParams(self):
        self.use_stop_pose = self.setupParam("~use_stop_pose", False)
        self.num_waypoints = self.setupParam("~num_waypoints", 2)
        self.visualization = self.setupParam("~visualization", True)
        default_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0 }
        self.canonical_goal_pose_right = self.setupParam("~canonical_goal_pose_right", default_pose)
        self.canonical_goal_pose_left = self.setupParam("~canonical_goal_pose_left", default_pose)
        self.canonical_goal_pose_straight = self.setupParam("~canonical_goal_pose_straight", default_pose)
        self.speed = self.setupParam("~speed", 0.30)

    def updateParams(self, event):
        pass

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo(f"[{self.node_name}] {param_name} = {value} ")
        return value

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
            rospy.loginfo(f"[{self.node_name}] Set LED pattern to: {pattern_name}")
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
            rospy.loginfo(f"[{self.node_name}] Set custom LED pattern: {color_list}")
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
            Led_pattern = self.get_led_pattern(priority_level= self.priority_level, direction_index= 0)
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
