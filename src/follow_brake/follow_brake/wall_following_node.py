import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math
import numpy as np

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # PID parameters
        self.Kp = 1.0
        self.Ki = 0.0
        self.Kd = 0.1
        self.prev_error = 0.0
        self.integral = 0.0

        # Desired parameters
        self.desired_distance = 1.0  # meters from wall
        self.lookahead = 0.5         # meters
        self.theta_deg = 50          # angle for second LiDAR beam
        self.get_logger().info("Wall follower node started")

    def get_range(self, data, angle):
        # Convert angle to index in scan
        angle_rad = math.radians(angle)
        index = int((angle_rad - data.angle_min) / data.angle_increment)
        index = np.clip(index, 0, len(data.ranges) - 1)
        return data.ranges[index]

    def scan_callback(self, data):
        theta = math.radians(self.theta_deg)
        a = self.get_range(data, self.theta_deg)
        b = self.get_range(data, 0)

        # Handle invalid readings
        if math.isinf(a) or math.isinf(b):
            self.get_logger().warn("Invalid LiDAR readings")
            return

        alpha = math.atan((a * math.cos(theta) - b) / (a * math.sin(theta)))
        Dt = b * math.cos(alpha)
        Dt1 = Dt + self.lookahead * math.sin(alpha)

        error = self.desired_distance - Dt1
        steering_angle = self.pid_control(error)

        speed = self.speed_control(steering_angle)
        self.publish_drive(steering_angle, speed)

    def pid_control(self, error):
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        control = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return np.clip(control, -0.4, 0.4)  # limit steering range

    def speed_control(self, steering_angle):
        angle_deg = abs(math.degrees(steering_angle))
        if angle_deg < 10:
            return 1.5
        elif angle_deg < 20:
            return 1.0
        else:
            return 0.5

    def publish_drive(self, steering, speed):
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
