#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import numpy

import rospy
from duckietown_msgs.msg import AprilTagsWithInfos, AprilTagDetection, FSMState, TurnIDandType, BoolStamped, WheelsCmdStamped
from std_msgs.msg import Int16  # Imports msg
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
import tf


class RandomAprilTagTurnsNode(DTROS):
    def __init__(self, node_name):
        super(RandomAprilTagTurnsNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION,
            fsm_controlled=True)

        self.node_name = node_name
        self.turn_type = -1
        rospy.loginfo(f"[{self.node_name}] Initializing.")

        # AprilTag filtering parameters
        self.dis_max = self.setupParameter("~dis_max", 1.0)
        self.angle_min = self.setupParameter("~angle_min", 90)
        self.angle_max = self.setupParameter("~angle_max", 270)
        self.horizontal_angle_threshold = self.setupParameter("~horizontal_angle_threshold", -77)

        # Pivot scan behavior when no tag is detected
        self.scan_speed = 0.05  # Wheel speed for pivot rotation
        self.scan_duration = 0.5  # How long to rotate per scan cycle (seconds)
        self._scan_active = False
        self._scan_end_time = rospy.Time(0)

        # Setup publishers
        self.pub_turn_type = rospy.Publisher("~turn_type", Int16, queue_size=1, latch=True)
        self.pub_id_and_type = rospy.Publisher("~turn_id_and_type", TurnIDandType, queue_size=1, latch=True)
        self.pub_intersection_go = rospy.Publisher("~intersection_go", BoolStamped, queue_size=1)
        self.pub_validated_tag = rospy.Publisher("~validated_tag_detection", AprilTagDetection, queue_size=1, latch=True)
        self.pub_wheels = rospy.Publisher(
            "wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            queue_size=1
        )

        # Setup subscribers
        self.sub_topic_tag = rospy.Subscriber("~tag", AprilTagsWithInfos, self.cbTag, queue_size=1)

        rospy.loginfo(f"[{self.node_name}] Initialzed.")

    def _publish_wheels(self, v_left, v_right):
        """Publish a WheelsCmdStamped to the wheels driver."""
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = v_left
        msg.vel_right = v_right
        self.pub_wheels.publish(msg)

    def _start_scan(self):
        """Begin a pivot scan: rotate in place for scan_duration seconds."""
        self._scan_active = True
        self._scan_end_time = rospy.Time.now() + rospy.Duration(self.scan_duration)
        rospy.loginfo(f"[{self.node_name}] Starting pivot scan to search for intersection sign.")

    def _stop_scan(self):
        """Stop any ongoing scan and send zero-velocity command."""
        if self._scan_active:
            rospy.loginfo(f"[{self.node_name}] Stopping pivot scan.")
        self._scan_active = False
        # send zero wheel command to stop rotation
        self._publish_wheels(0.0, 0.0)

    def cbTag(self, tag_msgs):
            """Process AprilTag detections and select nearest valid intersection sign."""
            dis_min = 999
            idx_min = -1

            for idx, taginfo in enumerate(tag_msgs.infos):
                # Only process intersection topology signs
                if taginfo.tag_type == taginfo.SIGN:
                    if taginfo.traffic_sign_type in {
                        taginfo.NO_RIGHT_TURN,
                        taginfo.LEFT_T_INTERSECT,
                        taginfo.NO_LEFT_TURN,
                        taginfo.RIGHT_T_INTERSECT,
                        taginfo.T_INTERSECTION,
                        taginfo.FOUR_WAY
                    }:
                        tag_det = (tag_msgs.detections)[idx]
                        pos = tag_det.transform.translation
                        distance = math.sqrt(pos.x**2 + pos.y**2 + pos.z**2)

                        # Calculate tag orientation relative to camera
                        q = (
                            tag_det.transform.rotation.x,
                            tag_det.transform.rotation.y,
                            tag_det.transform.rotation.z,
                            tag_det.transform.rotation.w
                        )
                        R = tf.transformations.quaternion_matrix(q)[:3, :3]
                        tag_normal_vector = R[:, 2]
                        dot_product = tag_normal_vector[2]

                        # Angle between tag normal and camera z-axis
                        # ~180° means tag faces camera (perpendicular to robot)
                        angle_rad = numpy.arccos(numpy.clip(dot_product, -1.0, 1.0))
                        angle_deg = numpy.degrees(angle_rad)

                        # Horizontal viewing angle (lateral position in camera view)
                        horizontal_angle = numpy.degrees(numpy.arctan2(pos.y, pos.x))

                        # rospy.loginfo(f"[{self.node_name}] Tag {taginfo.id}: angle={angle_deg:.1f}°, distance={distance:.3f}m, horizontal={horizontal_angle:.1f}°")

                        # Filter 1: Perpendicularity check
                        # Reject tags not facing camera or at oblique angles
                        if angle_deg < self.angle_min or angle_deg > self.angle_max or abs(dot_product) < 0.707:
                            continue

                        # Filter 2: Horizontal angle check
                        # Reject tags on left side or already passed
                        if horizontal_angle < self.horizontal_angle_threshold:
                            continue

                        # Filter 3: Distance check
                        if distance > self.dis_max:
                            continue

                        # Select nearest valid tag
                        if distance < dis_min:
                            dis_min = distance
                            idx_min = idx

            # No valid tag found - initiate pivot scan
            if idx_min == -1:
                rospy.logwarn("[RANDOM_APRIL_TAG_TURNS_NODE]: No valid intersection sign detected, starting pivot scan")
                now = rospy.Time.now()

                if not self._scan_active:
                    self._start_scan()

                # Continue pivot rotation during scan window
                if self._scan_active and now < self._scan_end_time:
                    s = self.scan_speed
                    self._publish_wheels(s, -s)
                else:
                    self._stop_scan()

            # Valid tag found - determine available turns
            else:
                self._stop_scan()
                taginfo = (tag_msgs.infos)[idx_min]
                validated_tag_detection = (tag_msgs.detections)[idx_min]

                # Determine available turns based on sign type
                availableTurns = []
                signType = taginfo.traffic_sign_type
                if signType == taginfo.NO_RIGHT_TURN or signType == taginfo.LEFT_T_INTERSECT:
                    availableTurns = [0, 1]  # Left=0, Straight=1
                elif signType == taginfo.NO_LEFT_TURN or signType == taginfo.RIGHT_T_INTERSECT:
                    availableTurns = [1, 2]  # Straight=1, Right=2
                elif signType == taginfo.FOUR_WAY:
                    availableTurns = [0, 1, 2]  # All directions
                elif signType == taginfo.T_INTERSECTION:
                    availableTurns = [0, 2]  # Left=0, Right=2

                rospy.loginfo(f"[{self.node_name}] Available turns: {availableTurns} from tag {taginfo.id}")

                # Randomly select a turn direction
                if len(availableTurns) > 0:
                    randomIndex = numpy.random.randint(len(availableTurns))
                    chosenTurn = availableTurns[randomIndex]
                    self.turn_type = chosenTurn
                    self.pub_turn_type.publish(self.turn_type)

                    # Publish turn type with tag ID
                    id_and_type_msg = TurnIDandType()
                    id_and_type_msg.tag_id = taginfo.id
                    id_and_type_msg.turn_type = self.turn_type
                    self.pub_id_and_type.publish(id_and_type_msg)

                    # Publish validated tag for alignment
                    self.pub_validated_tag.publish(validated_tag_detection)

                    # Signal intersection navigation can begin
                    intersection_go_msg = BoolStamped()
                    intersection_go_msg.header = tag_msgs.header
                    intersection_go_msg.data = True
                    self.pub_intersection_go.publish(intersection_go_msg)

    def setupParameter(self, param_name, default_value):
        """Load parameter from ROS parameter server with default fallback."""
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        return value

    def on_shutdown(self):
        rospy.loginfo(f"[{self.node_name}] Shutting down.")


if __name__ == "__main__":
    # Initialize the node with rospy
    # rospy.init_node("random_april_tag_turns_node", anonymous=False)

    # Create the NodeName object
    node = RandomAprilTagTurnsNode(node_name="random_april_tag_turns_node")

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()