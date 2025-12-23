#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64MultiArray

class GripperControl(Node):
    def __init__(self):
        super().__init__('gripper_control')

        self.declare_parameter('close_distance', 0.1) # 0.1 is just a default value of the travelling distance of the gripper joint, However we set the joint limits in .urdf file


        self.D = self.get_parameter('close_distance').value

        self.cmd_sub = self.create_subscription(
            Int32,
            '/gripper/cmd',
            self.cmd_callback,
            10)

        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10
        )

        self.status_pub = self.create_publisher(
            Int32,
            '/gripper/status',
            10
        )

        self.current_state = 0

    def cmd_callback(self, msg):
        cmd = msg.data
        out = Float64MultiArray()

        if cmd == 1:
            # Close
            out.data = [ self.D, self.D ]
            self.current_state = 1
        else:
            # Open
            out.data = [ 0.0, 0.0 ]
            self.current_state = 0

        self.cmd_pub.publish(out)

        status = Int32()
        status.data = self.current_state
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = GripperControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()