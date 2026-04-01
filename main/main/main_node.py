#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray
from geometry_msgs.msg import Twist

class MotorTestEncoder(Node):
    def __init__(self):
        super().__init__('motor_test_encoder')
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_encoders = self.create_subscription(Int32MultiArray, 'encoders', self.encoders_callback, 10)
        self.sub_gy25 = self.create_subscription(Int32, 'gy25', self.gy25_callback, 10)

        self.encoders = None
        self.encoders_start = None
        self.gy25_angle = None
        self.target_sum = 0

    def encoders_callback(self, msg):
        if len(msg.data) < 4:
            self.get_logger().warn(f'Expected 4 encoders, got: {list(msg.data)}')
            return

        self.encoders = list(msg.data[:4])
        if self.encoders_start is None:
            self.encoders_start = self.encoders.copy()
            self.get_logger().info(f'Initial encoders: {self.encoders_start}')

    def gy25_callback(self, msg):
        self.gy25_angle = msg.data

    def _check_target(self):
        if self.encoders_start is None or self.encoders is None:
            return False

        traveled = [
            abs(current - start)
            for current, start in zip(self.encoders, self.encoders_start)
        ]
        total = sum(traveled)
        gy25_info = self.gy25_angle if self.gy25_angle is not None else 'N/A'
        self.get_logger().info(f'Traveled total={total} | GY25 angle={gy25_info}')
        return total >= self.target_sum

    def normalize_angle(self, angle):
        """Нормализует угол в диапазон [-180, 180)."""
        return ((angle + 180) % 360) - 180

    def angle_delta(self, current_angle, start_angle):
        """Возвращает signed delta от стартового угла с учетом перехода через +/-180."""
        return self.normalize_angle(current_angle - start_angle)

    def start_motors(self, left_speed, right_speed):
        msg = Twist()
        msg.linear.x = (left_speed + right_speed) / 200.0
        msg.angular.z = (right_speed - left_speed) / 200.0
        self.pub_cmd_vel.publish(msg)
        self.get_logger().info(f'Motors started: left={left_speed}, right={right_speed}')

    def stop_motors(self):
        self.pub_cmd_vel.publish(Twist())
        self.get_logger().info('Motors stopped')

    def wait(self, duration):
        """Ждать duration секунд, обрабатывая входящие сообщения ROS2."""
        end_time = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().debug(f'Wait of {duration}s finished')

    def wait_for_encoder_initialization(self):
        """Ожидать первое сообщение encoders, продолжая крутить ROS callbacks."""
        last_log_time = 0.0
        while rclpy.ok() and self.encoders is None:
            rclpy.spin_once(self, timeout_sec=0.5)
            now = time.monotonic()
            if now - last_log_time >= 2.0:
                self.get_logger().warn('Waiting for messages in topic encoders...')
                last_log_time = now

    def wait_for_gy25_initialization(self):
        """Ожидать первое сообщение gy25, продолжая крутить ROS callbacks."""
        last_log_time = 0.0
        while rclpy.ok() and self.gy25_angle is None:
            rclpy.spin_once(self, timeout_sec=0.5)
            now = time.monotonic()
            if now - last_log_time >= 2.0:
                self.get_logger().warn('Waiting for messages in topic gy25...')
                last_log_time = now

    def move(self, left_speed, right_speed, enc_delta, delay_after=1.0):
        """Выполнить движение на заданное расстояние (в импульсах энкодеров)."""
        if self.encoders is None:
            self.get_logger().warn('Encoders are not initialized yet, waiting...')
            self.wait_for_encoder_initialization()

        self.encoders_start = self.encoders.copy()
        self.target_sum = enc_delta

        self.get_logger().info('Robot move - start')
        self.start_motors(left_speed, right_speed)
        while rclpy.ok() and not self._check_target():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Robot move - end')
        self.stop_motors()

        if delay_after > 0:
            self.get_logger().info(f'Waiting {delay_after} seconds before next action...')
            self.wait(delay_after)

    def rotate(self, angle, turn_speed=20, delay_after=1.0):
        """Повернуть робот на заданный угол по данным gy25."""
        if self.gy25_angle is None:
            self.get_logger().warn('GY25 angle is not initialized yet, waiting...')
            self.wait_for_gy25_initialization()

        if angle == 0:
            self.get_logger().info('Rotate skipped: target angle is 0')
            return

        start_angle = self.gy25_angle
        target_angle = self.normalize_angle(start_angle + angle)
        self.get_logger().info(
            f'Rotate start: start={start_angle} | delta={angle} | target={target_angle}'
        )

        if angle > 0:
            self.start_motors(-turn_speed, turn_speed)
        else:
            self.start_motors(turn_speed, -turn_speed)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.gy25_angle is None:
                continue

            current_delta = self.angle_delta(self.gy25_angle, start_angle)
            self.get_logger().info(
                f'Rotate progress: current={self.gy25_angle} | delta={current_delta} | target_delta={angle}'
            )

            if angle > 0 and current_delta >= angle:
                break
            if angle < 0 and current_delta <= angle:
                break

        self.stop_motors()
        self.get_logger().info(
            f'Rotate end: current={self.gy25_angle} | target={target_angle}'
        )

        if delay_after > 0:
            self.get_logger().info(f'Waiting {delay_after} seconds before next action...')
            self.wait(delay_after)

    def algorithm(self):

        def forward(distance):
            self.move(20, 20, distance, delay_after=2.0)

        def left(angle):
            self.rotate(angle, turn_speed=20, delay_after=2.0)

        def delay(time_sec):
            self.wait(time_sec)

        """Основной алгоритм: последовательные шаги с ожиданием."""
        self.get_logger().info('Algorithm: waiting for encoder initialization...')
        self.wait_for_encoder_initialization()
        self.get_logger().info('Algorithm: initialization done, starting motors')

        forward(5000)
        left(90)

        # self.move(20, 20, 100, delay_after=2.0)
        # self.move(20, -20, 1150, delay_after=2.0)
        # self.move(20, 20, 1100, delay_after=2.0)
        # self.move(-20, 20, 1150, delay_after=2.0)
        # self.move(-20, -20, 8000, delay_after=2.0)
        # self.move(20, 20, 13000, delay_after=2.0)
        # self.start_motors(10, 10)
        # self.wait(1.0)

        # self.start_motors(0, 0)
        # self.wait(2.0)
        # self.move(-20, -20, 21000, delay_after=2.0)
        # self.move(20, 20, 100, delay_after=2.0)
        # self.move(20, -20, 1450, delay_after=2.0)
        # self.move(20, 20, 6000, delay_after=2.0)
        # self.move(-20, -20, 3000, delay_after=2.0)

        self.get_logger().info('Algorithm: finishing node')
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MotorTestEncoder()
    node.algorithm()

if __name__ == '__main__':
    main()
