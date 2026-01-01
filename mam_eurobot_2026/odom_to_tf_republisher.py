#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class OdomTfRepublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_republisher')

        # Subscribe to the controller's odometry TF messages
        self.create_subscription(
            TFMessage,
            "/mecanum_drive_controller/tf_odometry",
            self.cb_republish,
            10
        )

        # Publisher to /tf
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)

    def cb_republish(self, msg: TFMessage):
        # Forward the entire TFMessage
        self.tf_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()