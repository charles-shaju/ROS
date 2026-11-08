import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import serial
import math

class BaseDriver(Node):
    def __init__(self):
        super().__init__('base_driver')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value

        # Robot Physical Constants (from your setup)
        self.wheel_radius = 0.035  # meters
        self.track_width = 0.17    # meters
        self.ticks_per_rev = 43.2
        # Odometry state variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vth = 0.0
        self.theta = 0.0
        self.last_left_ticks = None   # <--- Add this line
        self.last_right_ticks = None
        
        self.last_time = self.get_clock().now()

        # Serial connection to ESP32
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to ESP32 on {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            self.serial_conn = None

        # Publishers and Subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Timer for reading serial data and publishing odometry (e.g., 20 Hz)
        self.timer = self.create_timer(0.05, self.update_odometry)

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Calculate target velocities in m/s
        left_speed = linear - (angular * self.track_width / 2.0)
        right_speed = linear + (angular * self.track_width / 2.0)

        # Convert m/s to PWM (Assuming 1.0 m/s max speed roughly equals 255 PWM)
        # You can increase this multiplier if the robot is too slow!
        pwm_multiplier = 255.0  
        
        left_pwm = int(left_speed * pwm_multiplier)
        right_pwm = int(right_speed * pwm_multiplier)

        # Constrain values to strictly fall within -255 and 255
        left_pwm = max(min(left_pwm, 255), -255)
        right_pwm = max(min(right_pwm, 255), -255)

        self.get_logger().info(f"Sending to ESP32 -> L_PWM:{left_pwm}, R_PWM:{right_pwm}")

        if self.serial_conn and self.serial_conn.is_open:
            cmd_str = f"L:{left_pwm},R:{right_pwm}\n"
            self.serial_conn.write(cmd_str.encode('utf-8'))

    def update_odometry(self):
        if not self.serial_conn or not self.serial_conn.is_open:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        try:
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if line.startswith("ENC:"):
                    parts = line[4:].split(',')
                    left_ticks = -int(parts[0])
                    right_ticks = int(parts[1])
                    
                    if self.last_left_ticks is None:
                        self.last_left_ticks = left_ticks
                        self.last_right_ticks = right_ticks
                        self.last_time = current_time
                        return

                    d_left_ticks = left_ticks - self.last_left_ticks
                    d_right_ticks = right_ticks - self.last_right_ticks
                    
                    self.last_left_ticks = left_ticks
                    self.last_right_ticks = right_ticks
                    
                    d_left_wheel = 2.0 * math.pi * self.wheel_radius * (d_left_ticks / self.ticks_per_rev)
                    d_right_wheel = 2.0 * math.pi * self.wheel_radius * (d_right_ticks / self.ticks_per_rev)
                    
                    d_center = (d_left_wheel + d_right_wheel) / 2.0
                    d_theta = (d_right_wheel - d_left_wheel) / self.track_width
                    
                    self.x += d_center * math.cos(self.th + d_theta / 2.0)
                    self.y += d_center * math.sin(self.th + d_theta / 2.0)
                    self.th += d_theta
                    
                    if dt > 0:
                        self.vx = d_center / dt
                        self.vth = d_theta / dt
                        
                    self.last_time = current_time
                    
                    # DEBUG PRINT: Verify math is calculating!
                    self.get_logger().info(f"Odom -> X: {self.x:.3f}m | Y: {self.y:.3f}m | Yaw: {self.th:.3f}rad")

        except Exception as e:
            self.get_logger().error(f"Serial parsing error: {e}")

        # Publish the Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth

        # EKF REQUIRES COVARIANCE: Set small diagonal values (0.01) so EKF trusts it
        odom.pose.covariance[0] = 0.01  # x
        odom.pose.covariance[7] = 0.01  # y
        odom.pose.covariance[35] = 0.01 # yaw
        
        odom.twist.covariance[0] = 0.01 # vx
        odom.twist.covariance[7] = 0.01 # vy
        odom.twist.covariance[35] = 0.01 # vyaw

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = BaseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
