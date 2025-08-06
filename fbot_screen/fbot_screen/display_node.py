import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time

class MediaDisplayNode(Node):
    
    # ROS2 node to display various media types (text, image, video, topic)
    #on a designated UI topic.
    
    def __init__(self):
        #Initializes the node, parameters, publishers, and subscribers.
        super().__init__('media_display_node')

        # Parameters
        self.declare_parameter('screen_width', 640)
        self.declare_parameter('screen_height', 480)
        self.screen_width = self.get_parameter('screen_width').value
        self.screen_height = self.get_parameter('screen_height').value

        # Publisher and Subscriber
        self.ui_publisher = self.create_publisher(Image, '/ui_display', 10)
        self.command_subscriber = self.create_subscription(
            String,
            '/display_command',
            self.command_callback,
            10)

        self.bridge = CvBridge()
        self.current_media_thread = None
        self.stop_media_flag = threading.Event()
        self.current_topic_subscriber = None

        self.get_logger().info('Media Display Node started. Waiting for commands on /display_command')

    def stop_current_media(self):
        #Stops any currently running media display (video thread or topic subscription).
        if self.current_media_thread is not None:
            self.stop_media_flag.set()
            self.current_media_thread.join()
            self.stop_media_flag.clear()
            self.current_media_thread = None

        if self.current_topic_subscriber is not None:
            self.destroy_subscription(self.current_topic_subscriber)
            self.current_topic_subscriber = None

    def command_callback(self, msg):
        #Callback function for the /display_command topic.
        #Parses the command and calls the appropriate handler.
        self.stop_current_media()

        parts = msg.data.split(':', 1)
        if len(parts) != 2:
            self.get_logger().error(f'Malformed command: "{msg.data}". Use "type:value".')
            return

        media_type, media_value = parts
        self.get_logger().info(f'Received command: type="{media_type}"')

        if media_type == 'sentence':
            self.handle_sentence(media_value)
        elif media_type == 'image':
            self.handle_image(media_value)
        elif media_type == 'video':
            self.current_media_thread = threading.Thread(target=self.handle_video, args=(media_value,))
            self.current_media_thread.start()
        elif media_type == 'topic':
            self.handle_topic(media_value)

    def handle_sentence(self, text):
        #Handles displaying a sentence by creating an image with the text.
        # Create a white background
        image = np.full((self.screen_height, self.screen_width, 3), 255, dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        thickness = 2
        color = (0, 0, 0) 
        y0, dy = 50, 40

        for i, line in enumerate(text.split('\\n')):
            textsize = cv2.getTextSize(line, font, font_scale, thickness)[0]
            x = (self.screen_width - textsize[0]) // 2
            y = y0 + i * dy
            cv2.putText(image, line, (x, y), font, font_scale, color, thickness)

        self.ui_publisher.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

    def handle_image(self, path):
        #Handles displaying an image from a given path.
        try:
            image = cv2.imread(path)
            if image is None:
                self.get_logger().error(f'Image not found or invalid: {path}')
                return
            resized_image = cv2.resize(image, (self.screen_width, self.screen_height))
            self.ui_publisher.publish(self.bridge.cv2_to_imgmsg(resized_image, "bgr8"))
            self.get_logger().info('Image published successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    def handle_video(self, path):
        #Handles displaying a video from a given path in a loop.
        cap = cv2.VideoCapture(path)
        while not self.stop_media_flag.is_set():
            ret, frame = cap.read()
            if not ret:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Loop the video
                continue

            resized_frame = cv2.resize(frame, (self.screen_width, self.screen_height))
            if self.stop_media_flag.is_set():
                break
            self.ui_publisher.publish(self.bridge.cv2_to_imgmsg(resized_frame, "bgr8"))
            time.sleep(1/30)  # Limit to ~30 FPS
        cap.release()

    def handle_topic(self, topic_name):
        #Handles mirroring an existing image topic.
        
        self.current_topic_subscriber = self.create_subscription(
            Image, topic_name, self.ui_publisher.publish, 10)

def main(args=None):
    #Main function to initialize and run the ROS2 node.
    rclpy.init(args=args)
    node = MediaDisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()