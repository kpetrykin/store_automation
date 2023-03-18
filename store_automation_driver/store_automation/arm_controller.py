import threading
from std_msgs.msg import Float32

ENCODER_TIMESTEP = 50  # ms

class ArmController:
    def __init__(self, robot, ros_node) -> None:
        self._robot = robot
        self._ros_node = ros_node
        
        self._back_pitch_encoder = self._robot.getDevice('back_pitch_encoder')
        self._back_pitch_encoder.enable(ENCODER_TIMESTEP)
        
        self._pitch_publisher = self._ros_node.create_publisher(
            Float32, '/cur_pitch_angle', 1)
        
        self._pitch_pub_timer = self._ros_node.create_timer(0.2, self._pitch_pub_timer_callback)
        
        self._motors = {
            "yaw": {
                "front": {
                    "device": self._robot.getDevice('front_arm_yaw_motor'),
                    "coeff": 1.0
                },
                "back": {
                    "device": self._robot.getDevice('back_arm_yaw_motor'),
                    "coeff": -1.0
                }
            },
            "pitch_and_shoulders": {
                "front_pitch": {
                    "device": self._robot.getDevice('front_arm_pitch_motor'),
                    "coeff": 1.0
                },
                "back_pitch": {
                    "device": self._robot.getDevice('back_arm_pitch_motor'),
                    "coeff": -1.0
                },
                "front_shoulder": {
                    "device": self._robot.getDevice('front_shoulder_motor'),
                    "coeff": -1.0
                },
                "back_shoulder": {
                    "device": self._robot.getDevice('back_shoulder_motor'),
                    "coeff": 1.0
                }
            },
            "sliders": {
                "front": {
                    "device": self._robot.getDevice('front_arm_slider_motor'),
                    "coeff": 1.0
                },
                "back": {
                    "device": self._robot.getDevice('back_arm_slider_motor'),
                    "coeff": 1.0
                }
            },
            "grippers": {
                "front": {
                    "device": self._robot.getDevice('front_gripper_motor'),
                    "coeff": -1.0
                },
                "back": {
                    "device": self._robot.getDevice('back_gripper_motor'),
                    "coeff": -1.0
                }
            }
        }
        
        self._init_motors()
        
        self._ros_node.create_subscription(
            Float32, '/arm_yaw', self._arm_yaw_position_callback, 1)
        
        self._ros_node.create_subscription(
            Float32, '/arm_pitch', self._arm_pitch_position_callback, 1)
        
        self._ros_node.create_subscription(
            Float32, '/arm_slider', self._arm_slider_position_callback, 1)
        
        self._ros_node.create_subscription(
            Float32, '/arm_shoulder', self._arm_shoulder_position_callback, 1)
        
        self._ros_node.create_subscription(
            Float32, '/gripper_pos', self._gripper_position_callback, 1)
        
    def _init_motors(self) -> None:
        for _, motor_group in self._motors.items():
            for _, device in motor_group.items():
                device["device"].setPosition(float('inf'))
                device["device"].setVelocity(0)
    
    def _arm_yaw_position_callback(self, msg) -> None:
        for _, dev in self._motors["yaw"].items():
            dev["device"].setPosition(msg.data * dev["coeff"])
            dev["device"].setVelocity(0.5)
    
    def _arm_pitch_position_callback(self, msg) -> None:
        for _, dev in self._motors["pitch_and_shoulders"].items():
            dev["device"].setPosition(msg.data * dev["coeff"])
            dev["device"].setVelocity(0.5)
    
    def _arm_slider_position_callback(self, msg) -> None:
        for _, dev in self._motors["sliders"].items():
            dev["device"].setPosition(msg.data * dev["coeff"])
            dev["device"].setVelocity(0.1)
    
    def _arm_shoulder_position_callback(self, msg) -> None:
        for _, dev in self._motors["pitch_and_shoulders"].items():
            dev["device"].setPosition(msg.data * dev["coeff"])
            dev["device"].setVelocity(0.5)
    
    def _gripper_position_callback(self, msg) -> None:
        for _, dev in self._motors["grippers"].items():
            dev["device"].setPosition(msg.data * dev["coeff"])
            dev["device"].setVelocity(0.5)
            
    def _pitch_pub_timer_callback(self):
        msg = Float32()
        
        cur_pitch = self._back_pitch_encoder.getValue()
        msg.data = cur_pitch
        
        self._pitch_publisher.publish(msg)
    
    