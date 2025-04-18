import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Set up serial communication
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Adjust the port as needed
        time.sleep(2)  # Wait for the serial connection to be established

    def cmd_vel_callback(self, msg: Twist):
        if abs(msg.angular.z) > 0.1:
            self.turn_left() if msg.angular.z > 0 else self.turn_right()
        elif msg.linear.x > 0.1:
            self.forward()
        else:
            self.stop()

    def forward(self):
        # Send '1' to move forward via serial
        self.ser.write(b'1\n')

    def turn_left(self):
        # Send '2' to turn left via serial
        self.ser.write(b'2\n')

    def turn_right(self):
        # Send '3' to turn right via serial
        self.ser.write(b'3\n')

    def stop(self):
        # Send '0' to stop via serial
        self.ser.write(b'0\n')

def main():
    rclpy.init()
    node = CmdVelToSerial()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

