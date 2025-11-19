import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

class LidarFrameMerger(Node):
    def __init__(self):
        super().__init__('lidar_frame_merger')

        # Subscribers
        self.sub_front = self.create_subscription(
            LaserScan, '/lidar_front', self.front_cb, 10)
        self.sub_back = self.create_subscription(
            LaserScan, '/lidar_back', self.back_cb, 10)

        # Publishers for fixed frames
        self.pub_front_fixed = self.create_publisher(LaserScan, '/lidar_front_fixed', 10)
        self.pub_back_fixed = self.create_publisher(LaserScan, '/lidar_back_fixed', 10)

        # Publisher for merged 360° scan
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)

        # Store latest scans
        self.front_scan = None
        self.back_scan = None

        self.get_logger().info('LidarFrameMerger node started')

    def front_cb(self, msg):
        fixed_msg = self.copy_scan_with_frame(msg, 'base_link')
        self.pub_front_fixed.publish(fixed_msg)
        self.front_scan = fixed_msg
        self.try_merge()

    def back_cb(self, msg):
        fixed_msg = self.copy_scan_with_frame(msg, 'base_link')
        self.pub_back_fixed.publish(fixed_msg)
        self.back_scan = fixed_msg
        self.try_merge()

    def copy_scan_with_frame(self, msg, frame_id):
        """Creates a copy of a LaserScan message with a fixed frame."""
        fixed = LaserScan()
        fixed.header = Header()
        fixed.header.stamp = self.get_clock().now().to_msg()
        fixed.header.frame_id = frame_id
        fixed.angle_min = msg.angle_min
        fixed.angle_max = msg.angle_max
        fixed.angle_increment = msg.angle_increment
        fixed.time_increment = msg.time_increment
        fixed.scan_time = msg.scan_time
        fixed.range_min = msg.range_min
        fixed.range_max = msg.range_max
        fixed.ranges = list(msg.ranges)
        fixed.intensities = list(msg.intensities)
        return fixed

    def try_merge(self):
        if self.front_scan is None or self.back_scan is None:
            return

        merged = LaserScan()
        merged.header = Header()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = 'base_link'  # single merged frame

        # We'll keep min/max of the scans
        merged.angle_min = -3.14159  # -π
        merged.angle_max = 3.14159   # +π
        merged.angle_increment = self.front_scan.angle_increment
        merged.time_increment = self.front_scan.time_increment
        merged.scan_time = self.front_scan.scan_time
        merged.range_min = min(self.front_scan.range_min, self.back_scan.range_min)
        merged.range_max = max(self.front_scan.range_max, self.back_scan.range_max)

        # Concatenate the ranges and intensities
        merged.ranges = list(self.front_scan.ranges) + list(self.back_scan.ranges)
        if self.front_scan.intensities and self.back_scan.intensities:
            merged.intensities = list(self.front_scan.intensities) + list(self.back_scan.intensities)
        else:
            merged.intensities = []

        self.pub_scan.publish(merged)
        self.get_logger().debug(f'Published merged scan in frame: {merged.header.frame_id}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarFrameMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

