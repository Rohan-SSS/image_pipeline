import array
import sys

import cv2

import numpy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

class StereoImagePublisher(Node):
    def __init__(
        self,
        left_image: numpy.ndarray,
        right_image: numpy.ndarray,
        *,
        timer_period: float = 0.1
    ):

        super().__init__('image_publisher')
        self.left_image_and_info = self._create_image_and_info_messages(left_image)
        self.right_image_and_info = self._create_image_and_info_messages(right_image)

        self.left_image_pub = self.create_publisher(Image, 'left/image_rect', 1)
        self.left_camera_info_pub = self.create_publisher(CameraInfo, 'left/camera_info', 1)
        self.right_image_pub = self.create_publisher(Image, 'right/image_rect', 1)
        self.right_camera_info_pub = self.create_publisher(CameraInfo, 'right/camera_info', 1)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def _create_image_and_info_messages(self, image):
        image_msg = Image()
        image_msg.height = image.shape[0]
        image_msg.width = image.shape[1]
        image_msg.encoding = 'bgr8'
        image_msg.step = image_msg.width * 3
        image_msg.data = array.array('B', image.tobytes())

        camera_info_msg = CameraInfo()
        camera_info_msg.height = image.shape[0]
        camera_info_msg.width = image.shape[1]
        camera_info_msg.p = [
            1.0, 0.0, 1.0, 0.0,
            0.0, 1.0, 1.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        return (image_msg, camera_info_msg)

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        self.left_image_and_info[0].header.stamp = now
        self.left_image_and_info[1].header.stamp = now
        self.right_image_and_info[0].header.stamp = now
        self.right_image_and_info[1].header.stamp = now

        self.left_image_pub.publish(self.left_image_and_info[0])
        self.left_camera_info_pub.publish(self.left_image_and_info[1])
        self.right_image_pub.publish(self.right_image_and_info[0])
        self.right_camera_info_pub.publish(self.right_image_and_info[1])


if __name__ == '__main__':
    rclpy.init()
    left_image = cv2.imread(sys.argv[1])
    right_image = cv2.imread(sys.argv[2])
    publisher = StereoImagePublisher(left_image, right_image)
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

