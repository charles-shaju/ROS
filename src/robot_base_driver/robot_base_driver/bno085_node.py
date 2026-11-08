import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

class BNO085Node(Node):
    def __init__(self):
        super().__init__('bno085_ros_node')
        
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        
        # Initialize I2C and BNO085
        try:
            i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
            self.bno = BNO08X_I2C(i2c)
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.get_logger().info("BNO085 IMU Initialized Successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize BNO085: {e}")
            return

        # Publish at 30Hz to match EKF frequency
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        try:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            # Quaternion orientation
            qi, qj, qk, qr = self.bno.quaternion
            imu_msg.orientation.x = qi
            imu_msg.orientation.y = qj
            imu_msg.orientation.z = qk
            imu_msg.orientation.w = qr

            # Gyroscope angular velocity (rad/s)
            gx, gy, gz = self.bno.gyro
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz

            # Accelerometer linear acceleration (m/s^2)
            ax, ay, az = self.bno.acceleration
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az

            self.publisher_.publish(imu_msg)
        except Exception as e:
            self.get_logger().warn(f"Error reading IMU data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BNO085Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
