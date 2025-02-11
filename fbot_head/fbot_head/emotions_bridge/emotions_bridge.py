import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import yaml
from ament_index_python.packages import get_package_share_directory
import os
class EmotionsBridge(Node):

    def __init__(self, pause=False):

        super().__init__('emotions_bridge')

        self.motors = None
        #CARREGAR PARÂMETROS DO YAML    
        self.load_motors_params('motors.yaml')
        
        self.current_emotion = 'neutral'
        self.send_emotion(self.current_emotion)

        self.sub_emotion = self.create_subscription(String, 'fbot_face/emotion', self.emotion_callback, 10)
        

    def emotion_callback(self, msg):

        self.get_logger().info('Emotion received! ')
        self.current_emotion = msg.data
        self.send_emotion(self.current_emotion)

    def send_emotion(self, emotion):
        
        log_message = ''
        for motor in self.motors:
            # write_msg = [motor, self.get_parameter(self.get_name()+'.'+motor+'.'+emotion).value] #opção com namespace
            write_msg = [motor, self.get_parameter(motor+'.'+emotion).value] #opção sem namespace

            #REALIZAR O ENVIO NO PROTOCOLO DO MICROCONTROLADOR
            #ROSTOPIC ou SERIAL

            log_message = log_message+write_msg[0]+': '+str(write_msg[1])+'\n'

        self.get_logger().info('Writing: '+self.current_emotion+'\n'+log_message)

    
    def load_motors_params(self, filename):

        with open(os.path.join(get_package_share_directory('fbot_head'), 'config', filename)) as config_file:
            config = yaml.safe_load(config_file)[self.get_name()]['ros__parameters']

        self.motors = config
        for motor, value in config.items():
            # self.declare_parameters(namespace=, parameters=[(motor+'.'+prop, subvalue) for prop, subvalue in value.items()]) #opção com namespace
            for param, value in value.items():
                self.declare_parameter(motor+'.'+param, value) #opção sem namespace
            

        # self.declare_parameter('eyebrow_left_vert.pin', 0)
            

def main(args=None):
    rclpy.init(args=args)
    node = EmotionsBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()