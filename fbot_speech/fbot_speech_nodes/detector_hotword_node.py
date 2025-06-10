#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import re
from std_msgs.msg import String
from fbot_speech_scripts.detect_hotword import DetectHotWord
from ament_index_python.packages import get_package_share_directory


class HotwordDetectorNode(Node):
    def __init__(self):
        """
        @brief Constructor for the HotwordDetectorNode class.
        This class is responsible for detecting hotwords using the Porcupine library.
        """
        super().__init__('detector_hotword_node')

        ws_dir = os.path.abspath(os.path.join(get_package_share_directory('fbot_behavior'), '../../../..'))
        self.pack_dir = os.path.join(ws_dir, "src", "fbot_hri", "fbot_speech",'resources')

        # self.declareWordParameters()
        self.declareParameters()
        self.readParameters()
        self.initRosComm()

        # Sensibility for each keyword
        self.sensibility = [self.sensibility] * len(self.keyword)

        # Create hotword detector instance
        self.detector = DetectHotWord(self.keyword_path, self.sensibility)
        self.detector.hear()

        while rclpy.ok():
            result = self.detector.process()
            if result >= 0:
                keyword_output = String()
                keyword_output.data = self.keyword[result]
                self.get_logger().info(f"Hotword Detected: {keyword_output.data}")
                self.hotword_detector_publisher.publish(keyword_output)

    def initRosComm(self):
        """
        @brief Initialize ROS communication for the node.
        This function sets up the publisher and subscriber for hotword detection.
        """
        # Create publisher
        self.hotword_detector_publisher = self.create_publisher(String, self.detector_publisher_param, 10)

    def declareParameters(self):
        """
        @brief Declare parameters for the node.
        """
        self.declare_parameter('fbot_hotword_detection.sensibility', 0.5)
        self.declare_parameter('publishers.fbot_hotword_detection.topic', '/fbot_speech/bhd/detected')
        self.declare_parameter('fbot_hotword_detection.words', rclpy.Parameter.Type.STRING_ARRAY)

    def readParameters(self):
        """
        @brief Read parameters from the ROS parameter server.
        """
        self.detector_publisher_param = self.get_parameter('publishers.fbot_hotword_detection.topic').get_parameter_value().string_value
        self.sensibility = self.get_parameter('fbot_hotword_detection.sensibility').get_parameter_value().double_value
        self.words = self.get_parameter('fbot_hotword_detection.words').get_parameter_value().string_array_value
        self.keyword = []
        self.keyword_path = []
        for word in self.words:
            index = word.find(';')
            self.keyword.append(re.sub(r'[^a-z ]', '', word[:index].lower()))
            self.keyword_path.append(self.pack_dir+'/'+word[index+1:])
        
            
        
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
