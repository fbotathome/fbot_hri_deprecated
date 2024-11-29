#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import warnings
from std_msgs.msg import Empty
from scripts.detect_hotword import DetectHotWord
import rospkg
from termcolor import colored

class HotwordDetectorNode(Node):
    def __init__(self):
        super().__init__('detector_hotword_node')

        # Get package directory
        self.pack_dir = rospkg.RosPack().get_path('fbot_speech')

        # Parameters
        self.sensibility = self.get_parameter("/fbot_hotword_detection/sensibility").get_parameter_value().double_value
        self.keyword = [self.pack_dir + '/resources/Hello-Boris_en_linux_v3_0_0.ppn']
        self.sensibility = [self.sensibility] * len(self.keyword)

        # Topics for detection
        self.detector_publisher_param = self.get_parameter("publishers/fbot_hotword_detection/topic").get_parameter_value().string_value
        self.detector_subscriber_param = self.get_parameter("subscribers/fbot_hotword_detection/topic").get_parameter_value().string_value
        
        # Create publisher
        self.detector_publisher = self.create_publisher(Empty, self.detector_publisher_param, 10)

        # Create detector
        self.detector = DetectHotWord(self.keyword, self.sensibility)

        # Start hearing for hotwords
        self.detector.hear()

        # Timer to periodically check for hotword detection
        self.timer = self.create_timer(0.1, self.process_hotword)

    def process_hotword(self):
        result = self.detector.process()
        if result:
            self.detector_publisher.publish(Empty())
            self.get_logger().info(colored("Hotword Detected!", "green"))

def main(args=None):
    rclpy.init(args=args)

    # Create and spin the node
    node = HotwordDetectorNode()
    rclpy.spin(node)

    # Clean up and shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
