import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from tf_transformations import quaternion_from_euler

class FakeOdometryPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.publish_odom)  # Publish at 10Hz

        # Initial values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0  # m/s
        self.angular_velocity = 0.0  # rad/s

    def publish_odom(self):
        # Create a new Odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Update position based on constant velocity
        self.x += self.linear_velocity * 0.1  # 0.1s time step
        self.y += 0.0  # No movement in Y for simplicity
        self.theta += self.angular_velocity * 0.1  # 0.1s time step

        # Set position and orientation (in quaternion)
        odom.pose.pose = Pose()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Publish the Odometry message
        self.odom_pub.publish(odom)
        self.get_logger().info(f'Publishing fake odom: x={self.x}, y={self.y}, theta={self.theta}')

def main(args=None):
    rclpy.init(args=args)
    node = FakeOdometryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

