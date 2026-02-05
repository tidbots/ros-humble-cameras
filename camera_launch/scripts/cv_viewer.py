#!/usr/bin/env python3
"""OpenCV-based image viewer for ROS 2."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CvViewer(Node):
    def __init__(self):
        super().__init__('cv_viewer')

        self.declare_parameter('image', '/camera/image_raw')
        self.declare_parameter('window_name', 'Camera View')

        self.bridge = CvBridge()
        self.window_name = self.get_parameter('window_name').value
        image_topic = self.get_parameter('image').value

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f'Subscribing to: {image_topic}')

    def image_callback(self, msg):
        try:
            encoding = msg.encoding.lower()

            if encoding == 'rgb8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            elif encoding == 'bgr8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            elif encoding in ['yuyv', 'yuv422_yuy2']:
                # YUV422 YUYV format - manually convert
                yuv_data = np.frombuffer(msg.data, dtype=np.uint8)
                yuv_image = yuv_data.reshape((msg.height, msg.width, 2))
                cv_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_YUYV)
            elif encoding == 'uyvy':
                yuv_data = np.frombuffer(msg.data, dtype=np.uint8)
                yuv_image = yuv_data.reshape((msg.height, msg.width, 2))
                cv_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_UYVY)
            elif encoding == 'mono8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            elif encoding == 'mono16':
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono16')
                cv_image = (cv_image / 256).astype(np.uint8)
            else:
                # Try passthrough for unknown encodings
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

            cv2.imshow(self.window_name, cv_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # q or ESC
                self.get_logger().info('User requested shutdown')
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f'Error converting image (encoding={msg.encoding}): {e}')


def main(args=None):
    rclpy.init(args=args)
    viewer = CvViewer()
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()


if __name__ == '__main__':
    main()
