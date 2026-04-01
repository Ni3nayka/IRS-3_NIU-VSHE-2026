#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelTestNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_test_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Последовательность тестовых команд (x, y, rotation)
        self.sequence = [
            (0.3, 0.0, 0.0),
            (-0.3, 0.0, 0.0),
            (0.0, 0.3, 0.0),
            (0.0, -0.3, 0.0),
            (0.0, 0.0, 0.3),
            (0.0, 0.0, -0.3),
            (0.0, 0.0, 0.0),
        ]
        self.index = 0

        self.timer = self.create_timer(3.0, self.publish_next)
        self.get_logger().info('cmd_vel test node started, publishing every 3 seconds')

        # Отправим первую команду сразу при старте
        self.publish_next()

    def publish_next(self):
        x, y, rotation = self.sequence[self.index]

        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = rotation

        self.publisher.publish(msg)
        self.get_logger().info(f'Published cmd_vel: x={x:.1f}, y={y:.1f}, rotation={rotation:.1f}')

        self.index = (self.index + 1) % len(self.sequence)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
