#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
import math
import random

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu_scan', 10)
        self.front_lidar_pub = self.create_publisher(LaserScan, '/lidar_front', 10)
        self.back_lidar_pub = self.create_publisher(LaserScan, '/lidar_back', 10)

        
        self.timer = self.create_timer(0.1, self.publish_data)  # 10 Hz

    def publish_data(self):
        now = self.get_clock().now().to_msg()

        # --- IMU ---
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.linear_acceleration.x = random.uniform(-0.1, 0.1)
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = random.uniform(-0.1, 0.1)
        self.imu_pub.publish(imu_msg)

        # --- LIDAR parameters ---
        angle_min = -1.396263
        angle_max = 1.396263
        angle_increment = math.radians(0.5)
        range_min = 0.08
        range_max = 10.0
        num_readings = int((angle_max - angle_min) / angle_increment)

        # --- Front LIDAR ---
        front_scan = LaserScan()
        front_scan.header.stamp = now
        front_scan.header.frame_id = 'lidar_front'
        front_scan.angle_min = angle_min
        front_scan.angle_max = angle_max
        front_scan.angle_increment = angle_increment
        front_scan.range_min = range_min
        front_scan.range_max = range_max
        front_scan.ranges = [random.uniform(1.0, 5.0) for _ in range(num_readings)]
        self.front_lidar_pub.publish(front_scan)

        # --- Back LIDAR ---
        back_scan = LaserScan()
        back_scan.header.stamp = now
        back_scan.header.frame_id = 'lidar_back'
        back_scan.angle_min = angle_min
        back_scan.angle_max = angle_max
        back_scan.angle_increment = angle_increment
        back_scan.range_min = range_min
        back_scan.range_max = range_max
        back_scan.ranges = [random.uniform(1.0, 5.0) for _ in range(num_readings)]
        self.back_lidar_pub.publish(back_scan)

        self.get_logger().info(
        f"IMU → Acc: ({imu_msg.linear_acceleration.x:.2f}, "
        f"{imu_msg.linear_acceleration.y:.2f}, {imu_msg.linear_acceleration.z:.2f}) "
        f"Gyro: (0.0, 0.0, {imu_msg.angular_velocity.z:.2f})\n"
        f"LIDAR Front Ranges (sample): {front_scan.ranges[:5]}\n"
        f"LIDAR Back Ranges (sample): {back_scan.ranges[:5]}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
