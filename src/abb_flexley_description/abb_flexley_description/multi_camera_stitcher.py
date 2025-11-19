#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber


class MultiCameraStitcher(Node):
    def __init__(self):
        super().__init__('multi_camera_stitcher')

        self.bridge = CvBridge()

        # --- Subscriptions to 6 camera feeds ---
        self.sub_front = Subscriber(self, Image, '/camera_front')
        self.sub_back = Subscriber(self, Image, '/camera_back')
        self.sub_front_right = Subscriber(self, Image, '/camera_front_right')
        self.sub_back_right = Subscriber(self, Image, '/camera_back_right')
        self.sub_front_left = Subscriber(self, Image, '/camera_front_left')
        self.sub_back_left = Subscriber(self, Image, '/camera_back_left')

        # --- Synchronize all 6 streams ---
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_front, self.sub_back,
             self.sub_front_right, self.sub_back_right,
             self.sub_front_left, self.sub_back_left],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.callback)

        # --- Publishers ---
        self.pub_image = self.create_publisher(Image, '/camera_360/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/camera_360/camera_info', 10)

        self.get_logger().info('Multi-camera stitcher node started.')

    def callback(self, front, back, fr, br, fl, bl):
        try:
            # Convert ROS images to OpenCV
            imgs = [
                self.bridge.imgmsg_to_cv2(front, 'bgr8'),
                self.bridge.imgmsg_to_cv2(fr, 'bgr8'),
                self.bridge.imgmsg_to_cv2(br, 'bgr8'),
                self.bridge.imgmsg_to_cv2(back, 'bgr8'),
                self.bridge.imgmsg_to_cv2(bl, 'bgr8'),
                self.bridge.imgmsg_to_cv2(fl, 'bgr8')
            ]

            # Resize for consistency (optional)
            imgs = [cv2.resize(img, (320, 240)) for img in imgs]

            # Simple horizontal merge (side-by-side 360° strip)
            merged = cv2.hconcat(imgs)

            # Create ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(merged, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "base_link"  # Important for RViz

            # --- Create CameraInfo message ---
            cam_info = CameraInfo()
            cam_info.header = img_msg.header
            cam_info.width = merged.shape[1]
            cam_info.height = merged.shape[0]
            cam_info.distortion_model = "plumb_bob"
            cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            cam_info.k = [
                1.0, 0.0, cam_info.width / 2.0,
                0.0, 1.0, cam_info.height / 2.0,
                0.0, 0.0, 1.0
            ]
            cam_info.p = [
                1.0, 0.0, cam_info.width / 2.0, 0.0,
                0.0, 1.0, cam_info.height / 2.0, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]

            # --- Publish both ---
            self.pub_image.publish(img_msg)
            self.pub_info.publish(cam_info)

        except Exception as e:
            self.get_logger().error(f'Error processing images: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraStitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
