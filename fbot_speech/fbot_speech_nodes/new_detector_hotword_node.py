#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
from std_msgs.msg import Int16
from fbot_speech_scripts.new_detect_hotword import NewDetectHotWord

PACK_DIR = os.path.join(os.path.expanduser("~"), 'fbot_ws', 'src', 'fbot_hri', 'fbot_speech')

class HotwordDetectorNode(Node):
    def __init__(self):
        """
        @brief Constructor for the HotwordDetectorNode class.
        This class is responsible for detecting hotwords using the Porcupine library.
        """
        super().__init__('detector_hotword_node')
        self.declareParameters()
        self.readParameters()
        self.initRosComm()

        # Get the package directory
        self.pack_dir = PACK_DIR

        self.keyword = [
            self.pack_dir + '/resources/Hello-Boris_en_linux_v3_0_0.ppn',
            self.pack_dir + '/resources/It--s-right_en_linux_v3_0_0.ppn',
            self.pack_dir + '/resources/It--s-wrong_en_linux_v3_0_0.ppn'
        ]

        # Sensibility for each keyword
        self.sensibility = [self.sensibility] * len(self.keyword)

        # Create hotword detector instance
        self.detector = NewDetectHotWord(self.keyword, self.sensibility)
        self.detector.hear()
        while rclpy.ok():
            result = self.detector.process()
            self.get_logger().info(f"Hotword Detected: {result}")
            if result>=0:
                data = Int16()
                data.data = result
                self.detector_publisher.publish(data)

    def initRosComm(self):
        """
        @brief Initialize ROS communication for the node.
        This function sets up the publisher and subscriber for hotword detection.
        """
        # Create publisher
        self.detector_publisher = self.create_publisher(Int16, self.detector_publisher_param, 10)


    def declareParameters(self):
        """
        @brief Declare parameters for the node.
        """
        self.declare_parameter('fbot_hotword_detection.sensibility', 0.5)
        self.declare_parameter('publishers.fbot_hotword_detection.topic', '/fbot_speech/bhd/detected')

    def readParameters(self):
        """
        @brief Read parameters from the ROS parameter server.
        """
        self.detector_publisher_param = self.get_parameter('publishers.fbot_hotword_detection.topic').get_parameter_value().string_value
        self.sensibility = self.get_parameter('fbot_hotword_detection.sensibility').get_parameter_value().double_value

        
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
