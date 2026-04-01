#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class MotorTestNode(Node):
    def __init__(self):
        super().__init__('motor_test_node')
        self.pub_left = self.create_publisher(Int32, 'motor_speed_left', 10)
        self.pub_right = self.create_publisher(Int32, 'motor_speed_right', 10)

        self.ramp_time = 3.0
        self.max_speed = 100
        self.update_rate = 20.0

        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.total_time = 2 * self.ramp_time
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
        self.get_logger().info('Motor test node started')

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self.start_time

        if t >= self.total_time:
            # Останавливаем моторы
            self.pub_left.publish(Int32(data=0))
            self.pub_right.publish(Int32(data=0))
            self.get_logger().info('Test finished, stopping motors.')
            self.timer.cancel()
            # Корректно завершаем ROS2
            self.destroy_node()
            rclpy.shutdown()
            return

        if t <= self.ramp_time:
            speed = (t / self.ramp_time) * self.max_speed
        else:
            t_brake = t - self.ramp_time
            speed = self.max_speed * (1 - t_brake / self.ramp_time)

        speed_int = int(round(speed))
        self.pub_left.publish(Int32(data=speed_int))
        self.pub_right.publish(Int32(data=speed_int))

def main(args=None):
    rclpy.init(args=args)
    node = MotorTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()