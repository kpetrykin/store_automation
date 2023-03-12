import rclpy
from geometry_msgs.msg import Twist
from .odom_handler import OdomHandler

HALF_DISTANCE_BETWEEN_WHEELS = 0.35
WHEEL_RADIUS = 0.11


class StorebotDriver:
    def init(self, webots_node, properties) -> None:
        self._robot = webots_node.robot

        self._left_motor = self._robot.getDevice('left_motor')
        self._right_motor = self._robot.getDevice('right_motor')

        self._left_motor.setPosition(float('inf'))
        self._left_motor.setVelocity(0)

        self._right_motor.setPosition(float('inf'))
        self._right_motor.setVelocity(0)

        rclpy.init(args=None)
        self._node = rclpy.create_node('storebot_driver')
        self._node.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_callback, 1)

        self._odom_handler = OdomHandler(
            self._robot, self._node, WHEEL_RADIUS, HALF_DISTANCE_BETWEEN_WHEELS)

    def step(self) -> None:
        rclpy.spin_once(self._node, timeout_sec=0)

        self._odom_handler.publish_odometry()

    def _cmd_vel_callback(self, twist: Twist) -> None:
        linear_vel = twist.linear.x
        angular_vel = twist.angular.z

        command_motor_left = self._calc_motor_velocity(
            linear_vel, angular_vel, motor='left')
        command_motor_right = self._calc_motor_velocity(
            linear_vel, angular_vel, motor='right')

        self._left_motor.setVelocity(command_motor_left)
        self._right_motor.setVelocity(command_motor_right)

    def _calc_motor_velocity(self, linear_vel: float, angular_vel: float, motor='left') -> float:
        if motor == 'left':
            angular_vel *= -1

        return (linear_vel + angular_vel * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
