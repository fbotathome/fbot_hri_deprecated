import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64MultiArray
from geometry_msgs.msg import PoseStamped, PointStamped

from .PyDynamixel import DxlCommProtocol1, DxlCommProtocol2, JointProtocol1, JointProtocol2
import numpy as np


class NeckController(Node):

    def __init__(self, pause=False):

        super().__init__('neck_controller')

        self.pause = False

        self.sub_emergency_button = self.create_subscription(Bool, 'fbot_face/emotion', self.emergencyButtonPushed, 10)

        self.sub_update_neck = self.create_subscription(Float64MultiArray, "updateNeck", self.updateNeck, 10)
        # self.sub_update_neck_by_point = self.create_subscription(PointStamped, "updateNeckByPoint", self.neckByPointUpdated, queue_size=10)

        self.vel_limit = 800

        self.motors_id = {
            'horizontal_neck_joint':{
                'current_angle': np.pi,
                'id': 61,
                'min_angle': 120,
                'max_angle': 240
            },
            'vertical_neck_joint':{
                'current_angle': np.pi,
                'id': 62,
                'min_angle': 150,
                'max_angle': 190
            },
            'head_pan_joint':{
                'current_angle': np.pi,
                'id': 8,
            },
            'head_tilt_joint': {
                'current_angle': np.pi,
                'id': 9,
            }
        }

        self.motors = {}

        self.setupMotors()


    def updateNeck(self, msg):

        if self.pause:
            return

        data = msg.data

        pos_horizontal = np.radians(min(self.motors_id['horizontal_neck_joint']['max_angle'], max(self.motors_id['horizontal_neck_joint']['min_angle'], data[0])))
        pos_vertical = np.radians(min(self.motors_id['vertical_neck_joint']['max_angle'], max(self.motors_id['vertical_neck_joint']['min_angle'], data[1])))

        self.motors['horizontal_neck_joint'].sendGoalAngle(pos_horizontal)

        self.motors_id['vertical_neck_joint'].sendGoalAngle(pos_vertical)

        self.motors['head_pan_joint'].sendGoalAngle(pos_horizontal)

        self.motors['head_tilt_joint'].sendGoalAngle(2*np.pi - pos_vertical)

    def emergencyButtonPushed(self, msg):
        self.pause = not msg.data

    def setupMotors(self):

        try:
            self.neck_comm = DxlCommProtocol2("/dev/ttyNECK")

            for motor_name, props in self.motors_id.items():
                
                self.motors[motor_name] = JointProtocol2(props['id'])

                self.neck_comm.attachJoint(self.motors[motor_name])

                self.motors[motor_name].enableTorque()

                self.motors[motor_name].setVelocityLimit(self.vel_limit)

        except Exception as e:
            print("Neck port failed to connect.")
        

        

def main(args=None):
    rclpy.init(args=args)
    node = NeckController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()