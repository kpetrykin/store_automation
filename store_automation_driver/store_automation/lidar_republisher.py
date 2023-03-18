from rclpy.time import Time
from sensor_msgs.msg import LaserScan


class LidarRepublisher:
    def __init__(self, robot, ros_node) -> None:
        self._robot = robot
        self._ros_node = ros_node
        
        self._laser_scan_publisher = self._ros_node.create_publisher(
            LaserScan, '/scan', 1)
        
        self._ros_node.create_subscription(
            LaserScan, '/storebot/lidar', self._laser_scan_callback, 1)
    
    def _laser_scan_callback(self, msg: LaserScan) -> None:
        timestamp = Time(seconds=self._robot.getTime()).to_msg()
        
        timed_msg = msg
        timed_msg.header.stamp = timestamp
        
        self._laser_scan_publisher.publish(timed_msg)
        
        
        
        