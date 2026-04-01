#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class MotorTestEncoder(Node):
    def __init__(self):
        super().__init__('motor_test_encoder')
        self.pub_left = self.create_publisher(Int32, 'motor_speed_left', 10)
        self.pub_right = self.create_publisher(Int32, 'motor_speed_right', 10)
        self.sub_enc_left = self.create_subscription(Int32, 'encoder_left', self.enc_left_callback, 10)
        self.sub_enc_right = self.create_subscription(Int32, 'encoder_right', self.enc_right_callback, 10)

        self.speed = 20
        self.target_sum = 10000
        self.check_rate = 20.0

        self.enc_left = 0
        self.enc_right = 0
        self.enc_left_start = None
        self.enc_right_start = None
        self.moving = False
        self.test_complete = False
        self.initialized = False

        self.timer = self.create_timer(1.0 / self.check_rate, self.timer_callback)
        self.get_logger().info('Motor test encoder node started, waiting for encoder data...')

    def enc_left_callback(self, msg):
        self.enc_left = msg.data
        if self.enc_left_start is None:
            self.enc_left_start = msg.data
            self.get_logger().info(f'Initial left encoder: {self.enc_left_start}')
            self.check_initialized()

    def enc_right_callback(self, msg):
        self.enc_right = msg.data
        if self.enc_right_start is None:
            self.enc_right_start = msg.data
            self.get_logger().info(f'Initial right encoder: {self.enc_right_start}')
            self.check_initialized()

    def check_initialized(self):
        if not self.initialized and self.enc_left_start is not None and self.enc_right_start is not None:
            self.initialized = True
            self.get_logger().info('Both encoders initialized, starting motors...')
            self.start_motors()

    def start_motors(self):
        self.pub_left.publish(Int32(data=self.speed))
        self.pub_right.publish(Int32(data=self.speed))
        self.get_logger().info(f'Motors started at speed {self.speed}')
        self.moving = True

    def stop_motors(self):
        self.pub_left.publish(Int32(data=0))
        self.pub_right.publish(Int32(data=0))
        self.get_logger().info('Motors stopped')
        self.moving = False

    def timer_callback(self):
        if self.test_complete:
            return

        # Если ещё не инициализированы, просто ждём
        if not self.initialized:
            return

        # Если моторы ещё не запущены (хотя инициализированы), запускаем
        if not self.moving:
            self.start_motors()
            return

        # Вычисляем пройденное расстояние
        traveled_left = self.enc_left - self.enc_left_start
        traveled_right = self.enc_right - self.enc_right_start
        total_traveled = traveled_left + traveled_right
        self.get_logger().info(
            f'Traveled: left={traveled_left}, right={traveled_right}, total={total_traveled}'
        )

        if total_traveled >= self.target_sum:
            self.stop_motors()
            self.get_logger().info(f'Target reached: {total_traveled} >= {self.target_sum}. Test finished.')
            self.test_complete = True
            self.timer.cancel()
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MotorTestEncoder()
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