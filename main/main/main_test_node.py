#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Int32MultiArray


class CmdVelTestNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_test_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gy25_sub = self.create_subscription(Int32, 'gy25', self.gy25_callback, 10)
        self.encoders_sub = self.create_subscription(Int32MultiArray, 'encoders', self.encoders_callback, 10)
        self.last_gy25 = None
        self.last_enc_abs_sum = None

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

    def gy25_callback(self, msg):
        self.last_gy25 = msg.data
        self.log_sensor_state()

    def encoders_callback(self, msg):
        self.last_enc_abs_sum = sum(abs(value) for value in msg.data)
        self.log_sensor_state()

    def log_sensor_state(self):
        gy25_value = self.last_gy25 if self.last_gy25 is not None else 'N/A'
        enc_sum_value = self.last_enc_abs_sum if self.last_enc_abs_sum is not None else 'N/A'
        self.get_logger().info(f'GY25 angle: {gy25_value} | Encoders abs sum: {enc_sum_value}')


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
