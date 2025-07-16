import math
import rclpy
import tf2_ros
import numpy as np
import tf2_geometry_msgs
from copy import deepcopy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from fbot_vision_msgs.msg import Detection3DArray
from fbot_vision_msgs.srv import LookAtDescription3D
from geometry_msgs.msg import PoseStamped, PointStamped
from .PyDynamixel import DxlCommProtocol2, JointProtocol2


class NeckController(Node):
    def __init__(self, pause=False):
        """
        @brief A node for controlling the neck and head joints. It provides functionality 
        for updating neck positions, looking at specific points, and handling emergency stops.
        @param pause: If True, the node will not send any data to the motors.
        """
        super().__init__('neck_controller')

        self.motors_config = {
            'horizontal_neck_joint':{
                'current_angle': np.pi,
                'id': 62,
                'min_angle': 120,
                'max_angle': 240
            },
            'vertical_neck_joint':{
                'current_angle': np.pi,
                'id': 61,
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

        self.vel_limit = 800
        self.neck_port = "/dev/ttyNECK"
        self.motors: dict[str, JointProtocol2] = {}
        self.neck_comm = None
        try:
            self.setupMotors()
        except RuntimeError as e:
            self.get_logger().error(str(e))
            return 

        self.pause = False
        self.lock_updateNeck = False

        self.sub_emergency_button = self.create_subscription(Bool, 'emergency_button', self.emergencyButtonCallback, 10)
        self.sub_update_neck = self.create_subscription(Float64MultiArray, "updateNeck", self.updateNeckCallback, 10)
        self.sub_update_neck_by_point = self.create_subscription(PointStamped, "updateNeckByPoint", self.updateNeckByPointCallback, 10)


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.neck_updated = None

        self.joints_dict = { 
            'horizontal_neck_joint': (0., 0., 0.),
            'vertical_neck_joint':   (0., 0., 0.),
            'head_pan_joint':        (0., 0., 0.),
            'head_tilt_joint':       (0., 0., 0.)
        }

        self.pub_joint_states = self.create_publisher(JointState, 'boris_head/joint_states', 10)
        self.seq = 0

        self.srv_start_lookat = self.create_service(LookAtDescription3D, 'lookat_start', self.lookAtStart)
        self.srv_stop_lookat = self.create_service(Empty, 'lookat_stop', self.lookAtStop)

        self.sub_lookat = None
        self.lookat_description_identifier: dict = None
        self.lookat_pose: PoseStamped = None
        self.last_stopped_time = None
        self.look_at_timeout = float("inf")
        self.lookat_timer = None 
        self.lookat_timeout_callback = None
        self.frame = 'map'

        self.current_angle = [0.0, 0.0]
        self.initial_angle = [180.0, 180.0]
        self.updateNeck(self.initial_angle)
        self.joints_publish_timer = self.create_timer(5, self.updateJointsDict)

    def setupMotors(self) -> None:
        """
        @brief Initializes the Dynamixel motors, sets up the communication with the motors and configures their torque and velocity limits.
        """
        try:
            self.neck_comm = DxlCommProtocol2(self.neck_port)

        except Exception as e:
            raise RuntimeError(f"Failed to initialize NeckController: Neck port {self.neck_port} failed to connect. See readme for more details.")

        for motor_name, props in self.motors_config.items():
            
            self.motors[motor_name] = JointProtocol2(props['id'])

            self.neck_comm.attachJoint(self.motors[motor_name])

            self.motors[motor_name].enableTorque()

            self.motors[motor_name].setVelocityLimit(self.vel_limit)


    def emergencyButtonCallback(self, msg) -> None:
        """
        @brief Sets the pause state based on the emergency button's state.
        @param msg: (std_msgs.msg.Bool) The message containing the emergency button state.
        """
        self.pause = not msg.data

    def updateNeckCallback(self, msg) -> None:
        """
        @brief Updates the neck's position based on the received angles.
        @param msg: (std_msgs.msg.Float64MultiArray) The message containing the new neck angles.
        """
        data = msg.data
        self.updateNeck(list(data), from_updateNeckCallback=True)

    def updateNeckByPointCallback(self, msg) -> None:
        """
        @brief Computes the neck angles required to look at the given point and updates the neck's position.
        @param msg: (geometry_msgs.msg.PointStamped) The message containing the target point.
        """
        transform = self.computeTFTransform(source_header = msg.header)
        if not transform:
            self.get_logger().error("Failed to compute transform for updateNeckByPointCallback.")
            return
        ps = tf2_geometry_msgs.do_transform_point(msg, transform).point
        angle_msg = self.computeNeckStateByPoint(ps)
        self.updateNeck(angle_msg, from_updateNeckCallback=True)

    def updateNeck(self, data:list[float], from_updateNeckCallback = False) -> None:
        """
        @brief Updates the neck's position based on the given angles.
        @param data: (list[float]) The list of angles for the neck joints.
        @param from_updateNeckCallback: (bool) Whether the update was triggered by a callback (default: False).
        """
        if data:
            pos_horizontal = np.radians(min(self.motors_config['horizontal_neck_joint']['max_angle'], max(self.motors_config['horizontal_neck_joint']['min_angle'], data[0])))
            pos_vertical = np.radians(min(self.motors_config['vertical_neck_joint']['max_angle'], max(self.motors_config['vertical_neck_joint']['min_angle'], data[1])))

            self.motors_config['horizontal_neck_joint']['current_angle']=pos_horizontal
            self.motors_config['vertical_neck_joint']['current_angle']=pos_vertical
            self.motors_config['head_pan_joint']['current_angle']=pos_horizontal
            self.motors_config['head_tilt_joint']['current_angle']=2*np.pi - pos_vertical

        if self.pause:
            return

        if not self.lock_updateNeck and (not from_updateNeckCallback or self.sub_lookat is None):

            for key in self.motors:
                self.motors[key].sendGoalAngle(self.motors_config[key]['current_angle'])
                self.current_angle = data

            self.updateJointsDict()


    def updateJointsDict(self) -> None:
        """
        @brief Updates the joint state dictionary and publishes the joint states.
        """
        msg = JointState()

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
            
        try:
            self.pub_joint_states.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish joint states: {str(e)}")

        if (self.joints_dict['head_pan_joint'][1]<=0.3) and (self.joints_dict['head_tilt_joint'][1]<=0.3):
            if self.last_stopped_time is None:
                self.last_stopped_time = msg.header.stamp
        else:
            self.last_stopped_time = None

    def computeTFTransform(self, source_header=None, target_frame='camera_link_static', lastest=True):
        """
        @brief Computes the transform between two frames.
        @param target_frame: (str) The target frame ID.
        @param source_header: (std_msgs.msg.Header) The source frame header.
        @param lastest: (bool) Whether to use the latest transform (default: False).
        """
        if not source_header:
            self.get_logger().error('source_header parameter was not given.')
            return
        transform= None    
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_header.frame_id, rclpy.time.Time() if lastest else source_header.stamp)
            return transform
            
        except Exception as e:
            self.get_logger().error(f"Transform lookup failed: {str(e)}")
    
        return transform
    
    def computeNeckStateByPoint(self, point):
        """
        @brief Computes the neck angles required to look at a given point.
        @param point: (geometry_msgs.msg.Point) The target point.
        """
        horizontal = math.pi + math.atan2(point.y, point.x)
        dist = np.hypot(point.x, point.y)
        vertical = math.pi + math.atan2(point.z, dist) #ajuste vertical 
        return [math.degrees(horizontal), math.degrees(vertical)]

    def lookAtStart(self, req : LookAtDescription3D.Request, res : LookAtDescription3D.Response): 
        """
        @brief Stops the "look at" service and resets the neck position.
        @param req: (std_srvs.srv.Empty.Request) The service request.
        """
        self.lookat_description_identifier = {'global_id': req.global_id, 'id': req.id, 'label': req.label}
        self.get_logger().info(f"Starting lookAt service with description: {self.lookat_description_identifier}")
        self.get_logger().info(f"Starting lookAt initial angle: {isinstance(req.initial_angle, list)} -> {req.initial_angle} -> {type(req.initial_angle)}")
        # if isinstance(req.initial_angle, list) and len(req.initial_angle) == 2:
        self.initial_angle = list(req.initial_angle)
        self.sub_lookat = self.create_subscription(Detection3DArray, req.recognitions3d_topic, self.lookAtRecogCallback, 10)
        self.last_pose  = None
        self.last_pose_time = 0.
        self.lookat_pose = None
        # self.last_stopped_time = None
        self.look_at_timeout = 10*60.0 if req.timeout == 0 else req.timeout

        if self.lookat_timer:
            self.lookat_timer.cancel() 
        self.lookat_timer = self.create_timer(self.look_at_timeout, self.lookAtTimeout)

        return res

    def lookAtRecogCallback(self, msg):
        """
        @brief Callback for processing recognition messages and updating the neck position.
        @param msg: (fbot_vision_msgs.msg.Detection3DArray) The message containing the detected objects.
        """
        selected_desc = self.selectDescription(msg.detections)

        if selected_desc is not None:
            header = selected_desc.header
            if self.last_stopped_time is not None and Time.from_msg(header.stamp) >= Time.from_msg(self.last_stopped_time):
                transform = self.computeTFTransform(header, self.frame)

                lookat_pose = PoseStamped()
                lookat_pose.header = header
                lookat_pose.pose = selected_desc.bbox3d.center

                self.lookat_pose = tf2_geometry_msgs.do_transform_pose_stamped(lookat_pose, transform)
                                    
                if self.lookat_timer:
                    self.lookat_timer.reset()

                transform = self.computeTFTransform(self.lookat_pose.header, lastest=True)
                if not transform:
                    self.get_logger().error("Transform not found, cannot compute look at point.")
                    self.lookat_pose = None
                    return
                
                ps = tf2_geometry_msgs.do_transform_pose_stamped(self.lookat_pose, transform).pose.position

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
                    if abs(lookat_neck[0] - self.current_angle[0] + lookat_neck[1] - self.current_angle[1]) > 1.5:
                        self.updateNeck(lookat_neck)

                self.last_pose_time = time
                self.lookat_pose = None


    def lookAtTimeout(self):
        """
        @brief Callback triggered when the "look at" service times out.
        """
        self.updateNeck(data=self.initial_angle)

    def getCloserDescription(self, descriptions):
        """
        @brief Finds the closest description from a list of detected objects.
        @param descriptions: (list[fbot_vision_msgs.msg.Detection3D]) The list of detected objects.
        """
        min_desc = None
        min_dist = float('inf')
        for desc in descriptions:
            p = desc.bbox3d.center.position
            dist = np.linalg.norm([p.x, p.y, p.z])
            if dist < min_dist:
                min_desc = desc
                min_dist = dist
        
        return min_desc

    # TODO: implement for another parameters like global_id or local_id
    def selectDescription(self, descriptions):
        """
        @brief Selects a description based on the target criteria.
        @param descriptions: (list[fbot_vision_msgs.msg.Detection3D]) The list of detected objects.
        """
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

    def lookAtStop(self, req: Empty.Request, res: Empty.Response):
        """
        @brief Stops the "look at" service and resets the neck position.
        @param req: (std_srvs.srv.Empty.Request) The service request.
        """
        if self.sub_lookat is not None:
            self.destroy_subscription(self.sub_lookat)
            self.sub_lookat = None
            self.updateNeck(self.initial_angle)

        if self.lookat_timer is not None:
            self.lookat_timer.cancel()
            self.lookat_timer = None
        
        self.lookat_description_identifier = None

        return res

def main(args=None):
    rclpy.init(args=args)
    node = NeckController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()