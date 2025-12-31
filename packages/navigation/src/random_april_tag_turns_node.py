#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import numpy

import rospy
from duckietown_msgs.msg import AprilTagsWithInfos, FSMState, TurnIDandType, BoolStamped, WheelsCmdStamped
from std_msgs.msg import Int16  # Imports msg
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
import tf


class RandomAprilTagTurnsNode(DTROS):
    def __init__(self, node_name):
        super(RandomAprilTagTurnsNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION,
            fsm_controlled=True)

        # Save the name of the node
        self.node_name = node_name
        self.turn_type = -1
        rospy.loginfo(f"[{self.node_name}] Initializing.")

        # Setup parameters
        self.dis_max = self.setupParameter("~dis_max", 1.0)
        self.angle_min = self.setupParameter("~angle_min", 90)
        self.angle_max = self.setupParameter("~angle_max", 270)
        self.horizontal_angle_threshold = self.setupParameter("~horizontal_angle_threshold", -77)

        # Parameters for pivot scan behavior ---
        # speed: wheel speed magnitude for pivot (left = +s, right = -s)
        self.scan_speed = 0.05
        # duration: how long to rotate (in seconds) when no tag is detected
        self.scan_duration = 0.5
        # Internal scan state
        self._scan_active = False
        self._scan_end_time = rospy.Time(0)

        # Setup publishers
        self.pub_turn_type = rospy.Publisher("~turn_type", Int16, queue_size=1, latch=True)
        self.pub_id_and_type = rospy.Publisher("~turn_id_and_type", TurnIDandType, queue_size=1, latch=True)
        self.pub_intersection_go = rospy.Publisher("~intersection_go", BoolStamped, queue_size=1)
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
            # loop through list of april tags to
            # find the nearest apriltag
            dis_min = 999
            idx_min = -1
            for idx, taginfo in enumerate(tag_msgs.infos):
                if taginfo.tag_type == taginfo.SIGN:
                    # we need to make sure it's a sign that tells us topology
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

                        #Finding the anglular difference between tag normal and camera
                        q = (
                            tag_det.transform.rotation.x,
                            tag_det.transform.rotation.y,
                            tag_det.transform.rotation.z,
                            tag_det.transform.rotation.w
                        )
                        R = tf.transformations.quaternion_matrix(q)[:3, :3]
                        tag_normal_vector = R[:, 2]
                        dot_product = tag_normal_vector[2]

                        # Calculate angle between tag normal and camera Z-axis
                        # dot_product close to -1 means tag is perpendicular (facing camera)
                        # dot_product close to 0 means tag is at 90 degrees (sideways)
                        angle_rad = numpy.arccos(numpy.clip(dot_product, -1.0, 1.0))
                        angle_deg = numpy.degrees(angle_rad)

                        # Calculate horizontal viewing angle (left/right position in camera view)
                        # pos.x is forward distance, pos.y is lateral offset
                        # Horizontal angle: negative = right, positive = left
                        horizontal_angle = numpy.degrees(numpy.arctan2(pos.y, pos.x))

                        #rospy.loginfo(f"[RANDOM_APRIL_TAG_TURNS_NODE] turn type: {taginfo.id}; angle: {angle_deg:.1f} deg; distance: {distance:.3f}; horizontal: {horizontal_angle:.1f} deg")

                        # Ignore tags that are more than 45 degrees from perpendicular
                        # We want tags facing roughly towards the camera (angle close to 180 degrees)
                        if angle_deg < self.angle_min or angle_deg > self.angle_max or abs(dot_product) < 0.707:
                            #rospy.loginfo(f"[RANDOM_APRIL_TAG_TURNS_NODE] Ignoring tag {taginfo.id} at {angle_deg:.1f} degrees (not perpendicular)")
                            continue

                        # Ignore tags on the left side of camera view
                        # Horizontal viewing angle threshold is configurable
                        # This is robust regardless of robot position
                        if horizontal_angle < self.horizontal_angle_threshold:
                            #rospy.loginfo(f"[RANDOM_APRIL_TAG_TURNS_NODE] Ignoring tag {taginfo.id} at horizontal angle {horizontal_angle:.1f} degrees (< {self.horizontal_angle_threshold} threshold)")
                            continue

                        if distance > self.dis_max:
                            #rospy.loginfo(f"[RANDOM_APRIL_TAG_TURNS_NODE] Ignoring tag {taginfo.id} at distance={distance:.3f} (too far)")
                            continue

                        if distance < dis_min:
                            dis_min = distance
                            idx_min = idx

            if idx_min == -1:
                rospy.logwarn("[RANDOM_APRIL_TAG_TURNS_NODE]: Unable to determine available turns at intersection, "
                              "no appropriate signs detected. Duckiebot will pivot until one is detected")
                now = rospy.Time.now()

                # If we are not already scanning, start a scan
                if not self._scan_active:
                    self._start_scan()

                # While scanning and within the scan window, keep publishing pivot command
                if self._scan_active and now < self._scan_end_time:
                    # Pivot in place: left wheel forward, right wheel backward
                    s = self.scan_speed
                    self._publish_wheels(s, -s)
                else:
                    # Scan window over: stop the scan and stop wheels
                    self._stop_scan()
            else:
                self._stop_scan()
                taginfo = (tag_msgs.infos)[idx_min]

                availableTurns = []
                # go through possible intersection types
                signType = taginfo.traffic_sign_type
                if signType == taginfo.NO_RIGHT_TURN or signType == taginfo.LEFT_T_INTERSECT:
                    availableTurns = [
                        0,
                        1,
                    ]  # these mystical numbers correspond to the array ordering in open_loop_intersection_control_node (very bad)
                elif signType == taginfo.NO_LEFT_TURN or signType == taginfo.RIGHT_T_INTERSECT:
                    availableTurns = [1, 2]
                elif signType == taginfo.FOUR_WAY:
                    availableTurns = [0, 1, 2]
                elif signType == taginfo.T_INTERSECTION:
                    availableTurns = [0, 2]
                rospy.loginfo(f"[{self.node_name}] reports Available turns are: [{availableTurns}] from tag {taginfo.id}")
                # now randomly choose a possible direction
                if len(availableTurns) > 0:
                    randomIndex = numpy.random.randint(len(availableTurns))
                    chosenTurn = availableTurns[randomIndex]
                    self.turn_type = chosenTurn
                    self.pub_turn_type.publish(self.turn_type)

                    id_and_type_msg = TurnIDandType()
                    id_and_type_msg.tag_id = taginfo.id
                    id_and_type_msg.turn_type = self.turn_type
                    self.pub_id_and_type.publish(id_and_type_msg)

                    intersection_go_msg = BoolStamped()
                    intersection_go_msg.header = tag_msgs.header
                    intersection_go_msg.data = True
                    self.pub_intersection_go.publish(intersection_go_msg)

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        # rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
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