from math import sin, cos, isnan
from typing import Tuple
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from rclpy import logging

import tf_transformations

ENCODER_TIMESTEP = 50  # ms


class OdomHandler:
    def __init__(self, robot, ros_node, wheel_radius: float, half_wheel_distance: float) -> None:
        self._robot = robot
        self._ros_node = ros_node
        self._wheel_radius = wheel_radius
        self._wheel_distance = half_wheel_distance * 2

        self._odometry_frame = 'odom'
        self._robot_base_frame = 'base_link'

        self._left_encoder = self._robot.getDevice('left_encoder')
        self._right_encoder = self._robot.getDevice('right_encoder')
        self._left_encoder.enable(ENCODER_TIMESTEP)
        self._right_encoder.enable(ENCODER_TIMESTEP)
        self._left_encoder_startup_offset = None
        self._right_encoder_startup_offset = None

        self._odometry_publisher = self._ros_node.create_publisher(
            Odometry, '/odom', 1)
        self._tf_broadcaster = TransformBroadcaster(self._ros_node)

        # Initialize "prev" variables
        self._last_odometry_sample_time = self._robot.getTime()
        self._prev_left_wheel_ticks = 0
        self._prev_right_wheel_ticks = 0
        self._prev_position = (0.0, 0.0)
        self._prev_angle = 0.0

    def publish_odometry(self) -> None:
        timestamp = Time(seconds=self._robot.getTime()).to_msg()

        time_diff_s = self._robot.getTime() - self._last_odometry_sample_time
        if time_diff_s == 0.0:
            return

        left_wheel_ticks = self._left_encoder.getValue()
        right_wheel_ticks = self._right_encoder.getValue()

        # For some reason we have 'nan' in several first encoder values
        if not isnan(left_wheel_ticks):
            if self._left_encoder_startup_offset is None:
                self._left_encoder_startup_offset = left_wheel_ticks
        else:
            return

        if not isnan(right_wheel_ticks):
            if self._right_encoder_startup_offset is None:
                self._right_encoder_startup_offset = right_wheel_ticks
        else:
            return

        left_wheel_ticks -= self._left_encoder_startup_offset
        right_wheel_ticks -= self._right_encoder_startup_offset

        v, omega = self._calculate_velocities(
            left_wheel_ticks, right_wheel_ticks, time_diff_s)

        if not isnan(v):
            position, orientation = self._calculate_position_and_orientation(
                v, omega, time_diff_s)

            # Update variables
            self._prev_position = position.copy()
            self._prev_angle = orientation
            self._prev_left_wheel_ticks = left_wheel_ticks
            self._prev_right_wheel_ticks = right_wheel_ticks
            self._last_odometry_sample_time = self._robot.getTime()

            self._publish_tf_odom_base_link(timestamp, position, orientation)
            self._publish_odom_navmsg(
                timestamp, v, omega, position, orientation)

    def _calculate_velocities(self, left_wheel_ticks: float,
                              right_wheel_ticks: float, time_diff_s: float) -> Tuple[float, float]:
        v_left_rad = (left_wheel_ticks -
                      self._prev_left_wheel_ticks) / time_diff_s
        v_right_rad = (right_wheel_ticks -
                       self._prev_right_wheel_ticks) / time_diff_s
        v_left = v_left_rad * self._wheel_radius
        v_right = v_right_rad * self._wheel_radius
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / self._wheel_distance

        return v, omega

    def _calculate_position_and_orientation(self, v: float, omega: float, time_diff_s: float) -> Tuple[float, float]:
        # Fourth order Runge - Kutta
        # Reference: https://www.cs.cmu.edu/~16311/s07/labs/NXTLabs/Lab%203.html
        k00 = v * cos(self._prev_angle)
        k01 = v * sin(self._prev_angle)
        k02 = omega
        k10 = v * cos(self._prev_angle + time_diff_s * k02 / 2)
        k11 = v * sin(self._prev_angle + time_diff_s * k02 / 2)
        k12 = omega
        k20 = v * cos(self._prev_angle + time_diff_s * k12 / 2)
        k21 = v * sin(self._prev_angle + time_diff_s * k12 / 2)
        k22 = omega
        k30 = v * cos(self._prev_angle + time_diff_s * k22 / 2)
        k31 = v * sin(self._prev_angle + time_diff_s * k22 / 2)
        k32 = omega
        position = [
            self._prev_position[0] + (time_diff_s / 6) *
            (k00 + 2 * (k10 + k20) + k30),
            self._prev_position[1] + (time_diff_s / 6) *
            (k01 + 2 * (k11 + k21) + k31)
        ]
        angle = self._prev_angle + \
            (time_diff_s / 6) * (k02 + 2 * (k12 + k22) + k32)

        return position, angle

    def _publish_tf_odom_base_link(self, timestamp: int, position: list, orientation: float) -> None:
        x, y, z, w = tf_transformations.quaternion_from_euler(
            0, 0, orientation)
        q = Quaternion(x=x, y=y, z=z, w=w)

        tf = TransformStamped()
        tf.header.stamp = timestamp
        tf.header.frame_id = self._odometry_frame
        tf.child_frame_id = self._robot_base_frame
        tf.transform.translation.x = position[0]
        tf.transform.translation.y = position[1]
        tf.transform.translation.z = 0.29
        tf.transform.rotation = q
        self._tf_broadcaster.sendTransform(tf)

    def _publish_odom_navmsg(self, timestamp: int, linear_vel: float, angular_vel: float, position: list,
                             orientation: float) -> None:
        x, y, z, w = tf_transformations.quaternion_from_euler(
            0, 0, orientation)
        q = Quaternion(x=x, y=y, z=z, w=w)

        msg = Odometry()
        msg.header.stamp = timestamp
        msg.header.frame_id = self._odometry_frame
        msg.child_frame_id = self._robot_base_frame
        msg.twist.twist.linear.x = linear_vel
        msg.twist.twist.angular.z = angular_vel
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.position.z = 0.29
        msg.pose.pose.orientation = q
        self._odometry_publisher.publish(msg)
