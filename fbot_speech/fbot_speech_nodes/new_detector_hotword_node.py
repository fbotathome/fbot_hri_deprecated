#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import warnings
from std_msgs.msg import Int16
from fbot_speech_scripts.new_detect_hotword import NewDetectHotWord
import rospkg
from termcolor import colored

class HotwordDetectorNode(Node):
    def __init__(self):
        super().__init__('detector_hotword_node')

        # Get the package directory
        self.pack_dir = rospkg.RosPack().get_path('fbot_speech')

        # Parameters
        self.sensibility = self.get_parameter_or("/fbot_hotword_detection/sensibility", 0.5)

        self.keyword = [
            self.pack_dir + '/resources/Hello-Boris_en_linux_v3_0_0.ppn',
            self.pack_dir + '/resources/It--s-right_en_linux_v3_0_0.ppn',
            self.pack_dir + '/resources/It--s-wrong_en_linux_v3_0_0.ppn'
        ]

        # Sensibility for each keyword
        self.sensibility = [self.sensibility] * len(self.keyword)

        # Topics
        self.detector_publisher_param = self.get_parameter_or("publishers/fbot_hotword_detection/topic", "/fbot_speech/bhd/detected")
        self.detector_subscriber_param = self.get_parameter_or("subscribers/fbot_hotword_detection/topic", "/fbot_speech/bhd/hot_word")
        
        # Create publisher
        self.detector_publisher = self.create_publisher(Int16, self.detector_publisher_param, 10)

        # Create hotword detector instance
        self.detector = NewDetectHotWord(self.keyword, self.sensibility)

        # Start hearing for hotwords
        self.detector.hear()

        # Timer to periodically process the hotword detection
        self.timer = self.create_timer(0.1, self.processHotword)

    def processHotword(self):
        result = self.detector.process()
        if result >= 0:
            data = Int16()
            data.data = result
            self.detector_publisher.publish(data)
            self.get_logger().info(f"Hotword Detected: {result}")


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
