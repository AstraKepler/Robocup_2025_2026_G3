#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.callback, 10
        )
        self.pub = self.create_publisher(
            Twist, '/mecanum_drive_controller/reference_unstamped', 10
        )

    def callback(self, msg):
        out = Twist()

        # Invert velocities
        
        out.linear.x  = -msg.linear.x
        out.linear.y  = -msg.linear.y
        out.linear.z  = -msg.linear.z
        out.angular.x = -msg.angular.x
        out.angular.y = -msg.angular.y
        out.angular.z = -msg.angular.z
        
        self.pub.publish(out)

def main():
    rclpy.init()
    node = CmdVelRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()