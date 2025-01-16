import rclpy
from rclpy.node import Node




class EmotionsBridge(Node):

    def __init__(self, pause=False):

        super().__init__('emotions_bridge')

        self.pause = pause

