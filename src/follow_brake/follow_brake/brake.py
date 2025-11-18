import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math

class Brake(Node):
    def __init__(self):
        super().__init__('ebrake')

        self.odom_subscriber = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback,10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.vx = None
        self.scan = None

        self.impact_threshold = 1.5
        self.foo = 0

    def odom_callback(self, msg):
        self.vx = msg.twist.twist.linear.x

    def scan_callback(self, msg):
        self.scan = msg
    
    def brake_callback(self):
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
            self.publisher.publish(msg)
            self.get_logger().info('e-brake engaged')

    def timer_callback(self):
        if self.foo == 1:
            self.timer2 = self.create_timer(0.1, self.brake_callback)
            self.timer.cancel()
        ranges, min_idx = self.get_ranged_scans([-45,45])
        shortest_distance = min(ranges)
        idx = ranges.index(shortest_distance)

        full_idx = idx + min_idx
        theta = self.scan.angle_min + (full_idx * self.scan.angle_increment)

        ittc = None
        r_dot = self.vx * math.cos(theta)
        try:
            ittc = shortest_distance / max(0,r_dot)
            self.get_logger().info(str(ittc))
        except:
            ittc = 100
            self.get_logger().info('div by 0')

        if r_dot <= 0 : ittc = math.inf
        
        if ittc < self.impact_threshold:
            self.get_logger().info('!!!!!!!!!!!!!!!!')
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
            self.foo = 1
            self.publisher.publish(msg)


    
    # range is [from,to] in degrees
    # degrees go from -135 to 135
    def get_ranged_scans(self,range):
        range_from = self.rad_to_index(math.radians(range[0]))
        range_to = self.rad_to_index(math.radians(range[1]))
        return self.scan.ranges[range_from:range_to+1], range_from


    def rad_to_index(self, rad):
        return int((rad - self.scan.angle_min)/self.scan.angle_increment)

def main(args=None):
    rclpy.init(args=args)
    node = Brake()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
        


        


