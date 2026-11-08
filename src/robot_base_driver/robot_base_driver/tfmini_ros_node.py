import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import RPi.GPIO as GPIO
import serial
import time
import math

class TFMiniROSNode(Node):
    def __init__(self):
        super().__init__('tfmini_ros_node')
        
        # ROS 2 Publisher for standard 2D laser scans
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # ---------------- Stepper setup ----------------
        self.IN1, self.IN2, self.IN3, self.IN4 = 17, 27, 22, 24
        self.PINS = [self.IN1, self.IN2, self.IN3, self.IN4]
        
        self.SEQ = [
            [1,0,0,0], [1,1,0,0], [0,1,0,0], [0,1,1,0],
            [0,0,1,0], [0,0,1,1], [0,0,0,1], [1,0,0,1],
        ]
        
        self.STEPS_PER_REV = 4096
        self.STEPS_180DEG = self.STEPS_PER_REV // 2  # 2048
        self.STEP_DELAY = 0.0015
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in self.PINS:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)
            
        self.seq_index = 0
        self.position = 0
        self.direction = 1
        self.sample_every = 16
        self.step_count = 0

        # ---------------- TFmini-S setup ----------------
        self.LIDAR_PORT = "/dev/serial0"
        self.LIDAR_BAUD = 115200
        try:
            self.lidar = serial.Serial(self.LIDAR_PORT, self.LIDAR_BAUD, timeout=0)
        except Exception as e:
            self.get_logger().error(f"Failed to open LiDAR serial port: {e}")
            return

        # High-frequency timer to drive the sweep loop step-by-step
        self.timer = self.create_timer(0.0015, self.sweep_step_callback)
        self.get_logger().info("TFmini ROS 2 Sweep Node Initialized.")

    def do_step(self, direction):
        self.seq_index = (self.seq_index + direction) % 8
        for pin, val in zip(self.PINS, self.SEQ[self.seq_index]):
            GPIO.output(pin, val)

    def read_distance_nonblocking(self):
        latest = None
        while self.lidar.in_waiting >= 9:
            if self.lidar.read(1) != b'\x59':
                continue
            if self.lidar.read(1) != b'\x59':
                continue
            frame = self.lidar.read(7)
            if len(frame) != 7:
                break
            checksum = (0x59 + 0x59 + sum(frame[:6])) & 0xFF
            if checksum == frame[6]:
                dist = frame[0] + (frame[1] << 8)
                strength = frame[2] + (frame[3] << 8)
                if strength >= 100:
                    latest = dist
        return latest

    def sweep_step_callback(self):
        # Execute single step
        self.do_step(self.direction)
        self.position += self.direction
        self.step_count += 1

        # Check sample interval
        if self.step_count % self.sample_every == 0:
            d_cm = self.read_distance_nonblocking()
            if d_cm is not None and d_cm > 0:
                angle_deg = (self.position / self.STEPS_180DEG) * 180.0
                
                # Convert angle to radians relative to center (90 deg = straight ahead)
                # Matches your polar_to_cartesian offset logic
                theta = math.radians(angle_deg)
                angle_rad = theta - math.pi / 2.0
                distance_m = d_cm / 100.0

                # Publish standard LaserScan message
                scan_msg = LaserScan()
                scan_msg.header.stamp = self.get_clock().now().to_msg()
                scan_msg.header.frame_id = 'laser_frame'
                scan_msg.angle_min = angle_rad
                scan_msg.angle_max = angle_rad
                scan_msg.angle_increment = 0.01745
                scan_msg.time_increment = 0.0
                scan_msg.range_min = 0.05
                scan_msg.range_max = 12.0
                scan_msg.ranges = [distance_m]
                
                self.scan_pub.publish(scan_msg)

        # Reverse direction at limits
        if self.position >= self.STEPS_180DEG:
            self.direction = -1
        elif self.position <= 0:
            self.direction = 1

    def destroy_node(self):
        for pin in self.PINS:
            GPIO.output(pin, 0)
        if hasattr(self, 'lidar') and self.lidar.is_open:
            self.lidar.close()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TFMiniROSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
