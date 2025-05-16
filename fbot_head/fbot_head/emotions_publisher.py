import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64MultiArray


class EmotionsPublisher(Node):

    def __init__(self):
        """
        @brief A Node for sending different emotions periodically, in order to test the bridge node.
        """
        super().__init__('emotions_publisher')
        self.publisher_ = self.create_publisher(String, 'fbot_face/emotion', 10)
        self.publisher_neck = self.create_publisher(Float64MultiArray, '/updateNeck', 10)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.emotions = ['happy', 'sad', 'neutral', 'surprised', 'angry', 'suspicious', 'sleepy']

    def timer_callback(self):
        msg = String()
        msg.data = self.emotions[self.i]
        self.publisher_.publish(msg)
        self.publisher_neck.publish(Float64MultiArray(data=[200.0-(self.i*5), 180.0]))
        self.i = self.i+1 if self.i<6 else 0


def main(args=None):
    rclpy.init(args=args)

    emotions_publisher = EmotionsPublisher()

    rclpy.spin(emotions_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    emotions_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()