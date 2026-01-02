#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Float32
from custom_msgs.msg import YoloDetectionArray, TargetCoordinatesArray
import threading
import time

class RosBridge:
    def __init__(self):
        # We don't init_node here immediately to avoid issues with import time
        # It's called in run_ros_spin
        self.state = {
            "status": "DISCONNECTED",
            "targets_found": 0,
            "drone_status": "UNKNOWN",
            "last_heartbeat": 0
        }
        self.mission_pub = None

    def run_ros_spin(self):
        rospy.init_node('unified_control_backend', anonymous=True, disable_signals=True)
        
        # Publishers
        self.mission_pub = rospy.Publisher('/mission/command', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/yolo/detections', YoloDetectionArray, self.detection_callback)
        rospy.Subscriber('/mission/target_coordinates', TargetCoordinatesArray, self.target_callback)
        # Add more subs as needed
        
        self.state["status"] = "CONNECTED"
        rospy.loginfo("Unified Control Backend ROS Node Started")
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.state["last_heartbeat"] = time.time()
            rate.sleep()

    def detection_callback(self, msg):
        pass # Optional: Process raw detections

    def target_callback(self, msg):
        self.state["targets_found"] = len(msg.diff_targets)

    def publish_mission_trigger(self, command: str):
        if self.mission_pub:
            rospy.loginfo(f"UI Triggered Command: {command}")
            self.mission_pub.publish(command)

    def get_state(self):
        return self.state
