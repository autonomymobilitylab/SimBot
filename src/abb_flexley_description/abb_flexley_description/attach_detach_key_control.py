#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import sys
import termios
import tty
import subprocess
import re
import math
import time
import threading

class AttachDetachNode(Node):
    def __init__(self):
        super().__init__('attach_detach_key_control')
        self.attach_pub = self.create_publisher(Empty, '/attach', 10)
        self.detach_pub = self.create_publisher(Empty, '/detach', 10)
        self.get_logger().info("Attach/Detach node started. Press 't' to attach, 'r' to detach, 'q' to quit.")

        # XY proximity threshold (meters)
        self.proximity_threshold_xy = 0.55

        # Debounce: require this many consecutive checks inside threshold
        self.sustain_count_required = 3
        self._sustain_counter = 0
        self.models_close = False

        # Optional yaw alignment (set to None to ignore)
        self.yaw_tolerance = None

        # Flag to stop threads on exit
        self._running = True

    def get_key(self):
        """Blocking single-key read from stdin."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def get_model_pose(self, model_name):
        """Return (x, y, z, roll, pitch, yaw) from `ign model -m <name> --pose`."""
        try:
            cmd = ["ign", "model", "-m", model_name, "--pose"]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=3.0)
            lines = result.stdout.splitlines()
            xyz, rpy = None, None
            for line in lines:
                line = line.strip()
                if line.startswith('[') and xyz is None:
                    xyz = [float(v) for v in line.strip('[]').split()]
                elif line.startswith('[') and xyz is not None:
                    rpy = [float(v) for v in line.strip('[]').split()]
                    break
            if xyz is not None and rpy is not None:
                return (xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2])
        except Exception as e:
            self.get_logger().warn(f"Failed to get pose for {model_name}: {e}")
        return None

    def update_proximity(self):
        """Update planar proximity using only X and Y."""
        tpose = self.get_model_pose("trolley")
        upose = self.get_model_pose("flexley_tug")

        if not tpose or not upose:
            # couldn't read poses -> treat as FAR
            if self.models_close:
                self.get_logger().info("Models are now FAR (pose read failed).")
            self.models_close = False
            self._sustain_counter = 0
            return

        dx = tpose[0] - upose[0]
        dy = tpose[1] - upose[1]
        dist_sq = dx*dx + dy*dy
        thresh_sq = self.proximity_threshold_xy ** 2

        planar_ok = dist_sq <= thresh_sq

        yaw_ok = True
        if self.yaw_tolerance is not None:
            tyaw = tpose[5]
            uyaw = upose[5]
            diff = (tyaw - uyaw + math.pi) % (2*math.pi) - math.pi
            yaw_ok = abs(diff) <= self.yaw_tolerance

        if planar_ok and yaw_ok:
            self._sustain_counter += 1
        else:
            self._sustain_counter = 0

        prev_state = self.models_close
        if self._sustain_counter >= self.sustain_count_required:
            self.models_close = True
        else:
            self.models_close = False

        # Only print message when state changes
        if prev_state != self.models_close:
            state_str = "NEAR" if self.models_close else "FAR"
            planar_dist = math.sqrt(dist_sq)
            self.get_logger().info(f"Models are {state_str} (XY distance = {planar_dist:.3f} m).")

    def proximity_loop(self):
        """Thread to continuously check model proximity."""
        while self._running and rclpy.ok():
            self.update_proximity()
            time.sleep(0.2)

    def key_loop(self):
        """Thread to handle key input."""
        while self._running and rclpy.ok():
            key = self.get_key()
            if key == 't':
                if self.models_close:
                    self.attach_pub.publish(Empty())
                    self.get_logger().info(" Sent /attach message.")
                else:
                    self.get_logger().warning(" Models too far apart — cannot attach.")
            elif key == 'r':
                self.detach_pub.publish(Empty())
                self.get_logger().info(" Sent /detach message.")
            elif key == 'q':
                self.get_logger().info("Exiting...")
                self._running = False
                break

    def run(self):
        # Start threads for proximity checking and key handling
        prox_thread = threading.Thread(target=self.proximity_loop, daemon=True)
        key_thread = threading.Thread(target=self.key_loop, daemon=True)
        prox_thread.start()
        key_thread.start()
        # Wait for key thread to finish
        key_thread.join()
        self._running = False  # stop proximity thread

def main(args=None):
    rclpy.init(args=args)
    node = AttachDetachNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
