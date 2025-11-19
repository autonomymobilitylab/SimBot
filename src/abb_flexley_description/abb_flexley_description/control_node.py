import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading
import select

def get_key(timeout=0.1):
    """Non-blocking keyboard input using select."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # Obstacle detection
        self.front_range = float('inf')
        self.back_range = float('inf')
        self.obstacle_threshold = 0.5
        self.obstacle_blocked = False  # To prevent repeated logging

        # Speed settings
        self.linear_speed = 7.0
        self.angular_speed = 32.0

        # State
        self.pressed_keys = set()
        self.running = True

        # ROS interfaces
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.front_sub = self.create_subscription(LaserScan, '/lidar_front', self.front_callback, 10)
        self.back_sub = self.create_subscription(LaserScan, '/lidar_back', self.back_callback, 10)
        self.create_timer(0.1, self.control_loop)

        # Keyboard input thread
        self.key_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.key_thread.start()

    def front_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if 0.01 < r < float('inf')]
        self.front_range = min(valid_ranges) if valid_ranges else float('inf')

    def back_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if 0.01 < r < float('inf')]
        self.back_range = min(valid_ranges) if valid_ranges else float('inf')

    def keyboard_listener(self):
        while self.running:
            key = get_key(timeout=0.1)
            if not self.running:
                break
            if key is None:
                continue
            if key == 'q':
                self.get_logger().info("Quit key pressed — shutting down.")
                self.running = False
                rclpy.shutdown()
                break
            elif key in [' ', 'x']:
                self.get_logger().info("Emergency stop triggered.")
                self.pressed_keys.clear()
                self.publish_stop()
            elif key in ['w', 'a', 's', 'd']:
                self.pressed_keys.add(key)
            elif key in ['W', 'A', 'S', 'D']:
                self.pressed_keys.discard(key.lower())

    def control_loop(self):
        twist = Twist()

        wants_forward = 'w' in self.pressed_keys
        wants_backward = 's' in self.pressed_keys
        wants_left = 'a' in self.pressed_keys
        wants_right = 'd' in self.pressed_keys

        # Detect if the current command would hit an obstacle
        forward_blocked = wants_forward and self.front_range < self.obstacle_threshold
        backward_blocked = wants_backward and self.back_range < self.obstacle_threshold
        movement_blocked = forward_blocked or backward_blocked

        if movement_blocked:
            if not self.obstacle_blocked:
                self.get_logger().warn(" Obstacle detected — stopping all motion.")
                self.publish_stop()
                self.obstacle_blocked = True
            return  # Do not continue motion
        else:
            # Clear the block state once movement becomes safe
            self.obstacle_blocked = False

        # Set linear velocity
        if wants_forward:
            twist.linear.x = self.linear_speed
        elif wants_backward:
            twist.linear.x = -self.linear_speed

        # Set angular velocity
        if wants_left:
            twist.angular.z = self.angular_speed
        elif wants_right:
            twist.angular.z = -self.angular_speed

        self.cmd_pub.publish(twist)

    def publish_stop(self):
        """Publishes zero velocity to stop the robot."""
        stop_twist = Twist()
        self.cmd_pub.publish(stop_twist)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt — exiting cleanly.")
    finally:
        node.running = False
        node.key_thread.join(timeout=1.0)
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

