import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.35
WHEEL_RADIUS = 0.11

class StorebotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left_motor')
        self.__right_motor = self.__robot.getDevice('right_motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        rclpy.init(args=None)
        self.__node = rclpy.create_node('storebot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

    def __cmd_vel_callback(self, twist):
        linear_vel = twist.linear.x
        angular_vel = twist.angular.z

        command_motor_left = self.__calc_motor_velocity(linear_vel, angular_vel, motor='left')
        command_motor_right = self.__calc_motor_velocity(linear_vel, angular_vel, motor='right')

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)

    def __calc_motor_velocity(self, linear_vel, angular_vel, motor='left'):
        if motor == 'left':
            angular_vel *= -1

        return (linear_vel + angular_vel * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        
        