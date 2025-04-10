import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String, Bool, Float64MultiArray
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse

from fbot_vision_msgs.srv import LookAtDescription3D, LookAtDescription3DRequest, LookAtDescription3DResponse
from fbot_vision_msgs.msg import Detection3DArray, Detection3D

from .PyDynamixel import DxlCommProtocol1, DxlCommProtocol2, JointProtocol1, JointProtocol2
import numpy as np
import math

import tf2_ros
import tf2_geometry_msgs
from copy import deepcopy


class NeckController(Node):

    def __init__(self, pause=False):

        super().__init__('neck_controller')

        self.pause = False
        self.lock_updateNeck = False

        self.sub_emergency_button = self.create_subscription(Bool, 'emergency_button', self.emergencyButtonCallback)
        self.sub_update_neck = self.create_subscription(Float64MultiArray, "updateNeck", self.updateNeckCallback, queue_size=10)
        self.sub_update_neck_by_point = self.create_subscription(PointStamped, "updateNeckByPoint", self.updateNeckByPointCallback, queue_size=10)

        self.vel_limit = 800

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.neck_updated = None

        self.joints_dict = {    ## PRECISA DOS DOIS ULTIMOS ITENS? POIS SAO SEMPRE ZERO
            'horizontal_neck_joint': (0., 0., 0.),
            'vertical_neck_joint':   (0., 0., 0.),
            'head_pan_joint':        (0., 0., 0.),
            'head_tilt_joint':       (0., 0., 0.)
        }

        self.pub_joint_states = self.create_publisher(JointState, 'boris_head/joint_states', queue_size=10)
        self.seq = 0

        self.srv_start_lookat = self.create_service(LookAtDescription3D, 'lookat_start', self.lookAtStart)
        self.srv_stop_lookat = self.create_service(Empty, 'lookat_stop', self.lookAtStop)

        self.sub_lookat = None
        self.lookat_description_identifier: dict = None
        self.lookat_pose: PoseStamped = None
        self.last_stopped_time = None
        self.look_at_timeout = float("inf")
        self.lookat_timer = None  # Temporizador para o timeout
        self.lookat_timeout_callback = None  # Callback a ser chamado no timeout



        self.motors_config = {
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

        self.motors: dict[str, JointProtocol2] = {}

        self.setupMotors()

        self.initial_angle = Float64MultiArray()
        self.initial_angle.data = [180, 180]
        self.updateNeckByAngles(self.initial_angle)

    def setupMotors(self):

        try:
            self.neck_comm = DxlCommProtocol2("/dev/ttyNECK")

            for motor_name, props in self.motors_config.items():
                
                self.motors[motor_name] = JointProtocol2(props['id'])

                self.neck_comm.attachJoint(self.motors[motor_name])

                self.motors[motor_name].enableTorque()

                self.motors[motor_name].setVelocityLimit(self.vel_limit)

        except Exception as e:
            print("Neck port failed to connect.")

    def emergencyButtonCallback(self, msg):
        self.pause = not msg.data

    def updateNeckCallback(self, msg): 
        data = msg.data
        self.updateNeck(data=None, from_updateNeckCallback=True)

    def updateNeckByPointCallback(self, msg):

        transform = self.computeTFTransform(msg.header)
        ps = tf2_geometry_msgs.do_transform_point(msg, transform).point
        angle_msg = Float64MultiArray()
        angle_msg.data = self.computeNeckStateByPoint(ps)
        self.updateNeck(angle_msg, from_updateNeckCallback=True)

    def updateNeck(self, data:list[float], from_updateNeckCallback = False):

        if data:
            pos_horizontal = np.radians(min(self.motors_config['horizontal_neck_joint']['max_angle'], max(self.motors_config['horizontal_neck_joint']['min_angle'], data[0])))
            pos_vertical = np.radians(min(self.motors_config['vertical_neck_joint']['max_angle'], max(self.motors_config['vertical_neck_joint']['min_angle'], data[1])))

            self.motors_config['horizontal_neck_joint']['current_angle']=pos_horizontal
            self.motors_config['vertical_neck_joint']['current_angle']=pos_vertical
            self.motors_config['head_pan_joint']['current_angle']=pos_vertical
            self.motors_config['head_tilt_joint']['current_angle']=2*np.pi - pos_vertical

        if self.pause:
            return

        if not self.lock_updateNeck and (not from_updateNeckCallback or self.sub_lookat is None):

            for key in self.motors:
                self.motors[key].sendGoalAngle(self.motors_config[key]['current_angle'])

            self.updateJointsDict()


    def updateJointsDict(self):

        msg = JointState()

        # Nome da joint no URDF do Kinect
        msg.header.seq = self.seq
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        for key in self.joints_dict:
            position = self.motors[key].receiveCurrAngle() - np.pi
            if key=='vertical_neck_joint': position = -(position)
            self.joints_dict[key] = (position, 0., 0.)

            p, v, e = self.joints_dict[key]
            msg.name.append(key)
            msg.position.append(p)
            msg.velocity.append(v)
            msg.effort.append(e)

        self.pub_joint_states.publish(msg)

        ## PRA QUE SERVE ESSE SEQ?
        self.seq += 1

        if (self.joints_dict['head_pan_joint'][1]<=0.3) and (self.joints_dict['head_tilt_joint'][1]<=0.3):
            if self.last_stopped_time is None:
                self.last_stopped_time = msg.header.stamp
        else:
            self.last_stopped_time = None


    def computeTFTransform(self, target_frame='camera_link_static', source_header=None, lastest=False):

        if not source_header:
            self.get_logger().error('source_header parameter was not given.')
            return
        
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_header.frame_id, rclpy.time.Time() if lastest else source_header.stamp)
            
        except Exception as e:
            self.node.get_logger().error(f"Transform lookup failed: {str(e)}")
    
        return transform
    
    
    def computeNeckStateByPoint(self, point):

        horizontal = math.pi + math.atan2(point.y, point.x)
        dist = math.sqrt(point.x**2 + point.y**2)
        vertical = math.pi + math.atan2(point.z, dist)
        return [math.degrees(horizontal), math.degrees(vertical)]


    def lookAtStart(self, req : LookAtDescription3DRequest): ##PRECISO SALVAR OS ANGULOS ENVIADOS VIA UPDATENECK PARA SEREM UTILIZADOS QUANDO O LOOKATSERVICE TERMINAR?
        self.lookat_description_identifier = {'global_id': req.global_id, 'id': req.id, 'label': req.label}
        self.sub_lookat = self.create_subscription(Detection3DArray, req.recognitions3d_topic, self.lookAtRecogCallback, queue_size=1)
        self.last_pose  = None
        self.last_pose_time = 0.
        self.lookat_pose = None
        self.last_stopped_time = None
        self.look_at_timeout = float("inf") if req.timeout == 0 else req.timeout

        # temporizador pro timeout
        if self.lookat_timer:
            self.lookat_timer.cancel()  # Cancela o temporizador anterior, se existir
        self.lookat_timer = self.create_timer(self.look_at_timeout, self.lookAtTimeout)

        return LookAtDescription3DResponse()

    def lookAtRecogCallback(self, msg):
        selected_desc = self.selectDescription(msg.descriptions)

        if selected_desc is not None:

            header = selected_desc.poses_header
            if self.last_stopped_time is not None and header.stamp >= self.last_stopped_time:
                transform = self.computeTFTransform(header, to_frame_id=self.frame)

                lookat_pose = PoseStamped()
                lookat_pose.header = header
                lookat_pose.pose = selected_desc.bbox.center

                self.lookat_pose = tf2_geometry_msgs.do_transform_pose(lookat_pose, transform)

                if self.lookat_timer:
                    self.lookat_timer.reset()

                ##INCLUINDO ESCRITA NOS MOTORES
                transform = self.computeTFTransform(self.lookat_pose.header, lastest=True)
                ps = tf2_geometry_msgs.do_transform_pose(self.lookat_pose, transform).pose.position

                distance = 0.
                time = self.get_clock().now().nanoseconds / 1e9
                delta = float("inf")
                if self.last_pose != None:
                    new = np.array([ps.x, ps.y, ps.z])
                    previus = np.array([self.last_pose.x, self.last_pose.y, self.last_pose.z])
                    distance = np.linalg.norm(new - previus)
                    delta = time - self.last_pose_time
                
                if distance < max(1.5 * delta, 1.5):
                    lookat_neck = self.computeNeckStateByPoint(ps)
                    self.last_pose = deepcopy(ps)
                    self.updateNeck(lookat_neck)

                self.last_pose_time = time
                self.lookat_pose = None


    def lookAtTimeout(self):
        # Callback chamado quando o timeout é atingido
        self.get_logger().info("Timeout atingido no serviço lookAt!")
        self.updateNeck(data=self.initial_angle)

    
    def getCloserDescription(self, descriptions):
        min_desc = None
        min_dist = float('inf')
        for desc in descriptions:
            p = desc.bbox.center.position
            dist = math.sqrt(p.x**2 + p.y**2 + p.z**2)
            if dist < min_dist:
                min_desc = desc
                min_dist = dist
        
        return min_desc

    # TODO: implement for another parameters like global_id or local_id
    def selectDescription(self, descriptions):
        selected_descriptions = []

        desired_label = self.lookat_description_identifier['label']

        if desired_label != '':
            for desc in descriptions:
                if desc.label == desired_label:
                   selected_descriptions.append(desc)
        else:
            selected_descriptions = descriptions

        desc = self.getCloserDescription(selected_descriptions)

        return desc

    def lookAtStop(self, req):
        if self.sub_lookat is not None:
            self.sub_lookat.unregister()
            self.sub_lookat = None

        if self.lookat_timer is not None:
            self.lookat_timer.cancel()
            self.lookat_timer = None
        
        self.lookat_description_identifier = None

        return EmptyResponse()

        

def main(args=None):
    rclpy.init(args=args)
    node = NeckController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()