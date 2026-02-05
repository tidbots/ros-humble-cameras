#!/usr/bin/env python3
"""OpenCV-based image viewer for ROS 2."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


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
            if msg.encoding == 'rgb8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)

            cv2.imshow(self.window_name, cv_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # q or ESC
                self.get_logger().info('User requested shutdown')
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')


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
