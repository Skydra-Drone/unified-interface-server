#!/usr/bin/env python3
import roslibpy
import threading
import time
import logging

class RosBridge:
    def __init__(self):
        self.state = {
            "status": "CONNECTING...",
            "targets_found": 0,
            "drone_status": "UNKNOWN",
            "last_heartbeat": 0
        }
        self.client = None
        self.mission_pub = None
        self.talker = None

    def run_ros_spin(self):
        # Connect to ROS Bridge Daemon running on the Linux machine (or WSL)
        # Default: localhost:9090. Change IP if ROS is on another laptop.
        self.client = roslibpy.Ros(host='localhost', port=9090)
        
        try:
            self.client.run()
            self.state["status"] = "CONNECTED"
            print("Unified Control Backend: Connected to ROS Bridge!")
        except Exception as e:
            self.state["status"] = "CONNECTION FAILED"
            print(f"Failed to connect: {e}")
            return

        # Publishers
        self.talker = roslibpy.Topic(self.client, '/mission/command', 'std_msgs/String')
        self.talker.advertise()
        
        # Subscribers
        self.detection_listener = roslibpy.Topic(self.client, '/yolo/detections', 'custom_msgs/YoloDetectionArray')
        self.detection_listener.subscribe(self.detection_callback)
        
        self.target_listener = roslibpy.Topic(self.client, '/mission/target_coordinates', 'custom_msgs/TargetCoordinatesArray')
        self.target_listener.subscribe(self.target_callback)
        
        # Keep alive loop
        while self.client.is_connected:
            self.state["last_heartbeat"] = time.time()
            time.sleep(1)
        
        self.state["status"] = "DISCONNECTED"

    def detection_callback(self, msg):
        pass # Optional: Process raw detections

    def target_callback(self, msg):
        # 'msg' is a Dictionary in roslibpy
        targets = msg.get('diff_targets', [])
        self.state["targets_found"] = len(targets)

    def publish_mission_trigger(self, command: str):
        if self.talker and self.client.is_connected:
            print(f"UI Triggered Command: {command}")
            # roslibpy expects a dictionary matching the message definition
            self.talker.publish(roslibpy.Message({'data': command}))

    def get_state(self):
        return self.state
