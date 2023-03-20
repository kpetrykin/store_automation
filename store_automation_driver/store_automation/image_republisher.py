import rclpy
from rclpy.node import Node

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CompressedImage


class ImageRepublisher(Node):
    def __init__(self):
        super().__init__('image_republisher')
        self._cv_bridge = CvBridge()
        
        self.create_subscription(
            Image, '/storebot/camera', self._camera_image_callback, 1)
        
        self._compressed_img_publisher = self.create_publisher(CompressedImage,
                                                               '/storebot/compressed_image', 1)
        
    def _camera_image_callback(self, msg):
        compressed_msg = self.convert_ros_msg_to_ros_compressed_msg(msg)
        self._compressed_img_publisher.publish(compressed_msg)
        
    def convert_ros_msg_to_cv2(self, ros_data, image_encoding='bgr8'):
        """
        Convert from a ROS Image message to a cv2 image.
        """
        try:
            img = self._cv_bridge.imgmsg_to_cv2(ros_data, image_encoding)
            
            # cv2.imshow("Display window", img)
            # # # print(type(message))
            # cv2.waitKey(1)
            return img
        except CvBridgeError as e:
            if "[16UC1] is not a color format" in str(e):
                raise CvBridgeError(
                    "You may be trying to use a Image method " +
                    "(Subscriber, Publisher, conversion) on a depth image" +
                    " message. Original exception: " + str(e))
            raise e
        
    def convert_ros_msg_to_ros_compressed_msg(self, image,
                                              image_encoding='bgr8',
                                              compressed_format="jpg"):
        """
        Convert from ROS Image message to ROS CompressedImage.
        """
        cv2_img = self.convert_ros_msg_to_cv2(image, image_encoding)
        cimg_msg = self._cv_bridge.cv2_to_compressed_imgmsg(cv2_img,
                                                            dst_format=compressed_format)
        cimg_msg.header = image.header
        return cimg_msg
    

def main(args=None):
    rclpy.init(args=args)
    ir = ImageRepublisher()
    
    rclpy.spin(ir)
    
    ir.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()