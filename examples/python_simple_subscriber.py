#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np


class CompressedSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_subscriber')

        self.subscription = self.create_subscription(CompressedImage, '/video_source/compressed', self.callback, 10)

    def callback(self, msg: CompressedImage):
        # Convert compressed image data to OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image is None:
            self.get_logger().warn("Failed to decode compressed image")
            return

        # Display image
        cv2.imshow("Compressed Image", image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CompressedSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt")

    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
