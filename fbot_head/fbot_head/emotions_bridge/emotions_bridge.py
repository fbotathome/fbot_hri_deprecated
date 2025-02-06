import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class EmotionsBridge(Node):

    def __init__(self, pause=False):

        super().__init__('emotions_bridge')

        self.get_logger().info('teste')
        self.sub_emotion = self.create_subscription(String, 'fbot_face/emotion', self.emotion_callback, 10)
        


    def emotion_callback(self, msg):

        self.get_logger().info(msg.data)



def main(args=None):
    
    rclpy.init(args=args)
    node = EmotionsBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()