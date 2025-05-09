import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.ser = serial.Serial('/dev/mcu', 115200, timeout=1)
        time.sleep(2)

        self.last_cmd_time = self.get_clock().now()
        self.timeout_sec = 2.0  # Timeout in seconds

        # Create a timer that runs at 10Hz to check for timeout
        self.timer = self.create_timer(0.1, self.check_timeout)

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        if abs(msg.angular.z) > abs(msg.linear.x):
            self.turn_left() if msg.angular.z > 0 else self.turn_right()
        elif msg.linear.x != 0:
            self.forward() if msg.linear.x > 0 else self.backward()
        else:
            self.stop()

    def check_timeout(self):
        time_since_last = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_since_last > self.timeout_sec:
            self.stop()

    def forward(self):
        self.send_serial(12, 12, 11, 11)

    def turn_left(self):
        self.send_serial(-25, 25, -25, 29)

    def backward(self):
        self.send_serial(-11, -11, -12, -12)

    def turn_right(self):
        self.send_serial(25, -25, 25, -25)

    def stop(self):
        self.send_serial(0, 0, 0, 0)

    def send_serial(self, front_left, front_right, back_left, back_right):
        self.ser.write(bytes([64 + front_left, 192 - front_right, 64 + back_left, 192 - back_right]))

def main():
    rclpy.init()
    node = CmdVelToSerial()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

