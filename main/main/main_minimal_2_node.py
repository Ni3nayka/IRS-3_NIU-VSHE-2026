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

        self.declare_parameter('target_way', 'fflfrfsf')

        self.encoders = None
        self.encoders_start = None
        self.gy25_angle = None
        self.gy25_start_angle = None
        self.target_relative_angle = 0
        self.target_sum = 0
        self.encoder_sum_per_cm = 211 #115000.0 / 510.0
        self.target_way = self.get_parameter('target_way').get_parameter_value().string_value

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
        if self.gy25_start_angle is None:
            self.gy25_start_angle = msg.data
            self.get_logger().info(f'GY25 start angle saved: {self.gy25_start_angle}')

    def _check_target(self):
        if self.encoders_start is None or self.encoders is None:
            return False

        total = self.traveled_encoder_sum()
        gy25_info = self.gy25_angle if self.gy25_angle is not None else 'N/A'
        self.get_logger().info(f'Traveled total={total} | GY25 angle={gy25_info}')
        return total >= self.target_sum

    def angle_delta(self, current_angle, start_angle):
        """Возвращает signed delta от стартового угла для накопленного угла gy25."""
        return current_angle - start_angle

    def traveled_encoder_sum(self):
        """Возвращает сумму модулей изменений по 4 энкодерам от стартовой точки."""
        if self.encoders_start is None or self.encoders is None:
            return 0

        return sum(
            abs(current - start)
            for current, start in zip(self.encoders, self.encoders_start)
        )

    def cm_to_encoder_sum(self, distance_cm):
        """Переводит сантиметры в целевую сумму энкодеров."""
        return abs(distance_cm) * self.encoder_sum_per_cm

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

    def forward(self, distance_cm, max_speed=60, min_speed=10, kp=0.005, distance_tolerance_cm=2.0, delay_after=1.0):
        """Движение вперед/назад на заданное расстояние в сантиметрах с P-регулятором."""
        if self.encoders is None:
            self.get_logger().warn('Encoders are not initialized yet, waiting...')
            self.wait_for_encoder_initialization()

        if distance_cm == 0:
            self.get_logger().info('Forward skipped: target distance is 0 cm')
            return

        self.encoders_start = self.encoders.copy()
        target_sum = self.cm_to_encoder_sum(distance_cm)
        tolerance_sum = self.cm_to_encoder_sum(distance_tolerance_cm)
        direction = 1 if distance_cm > 0 else -1
        last_command_speed = None

        self.get_logger().info(
            f'Forward start: distance_cm={distance_cm} | target_sum={target_sum:.1f} | tolerance_sum={tolerance_sum:.1f}'
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            traveled_sum = self.traveled_encoder_sum()
            remaining_sum = target_sum - traveled_sum
            gy25_info = self.gy25_angle if self.gy25_angle is not None else 'N/A'
            self.get_logger().info(
                f'Forward progress: traveled_sum={traveled_sum:.1f} | remaining_sum={remaining_sum:.1f} | GY25 angle={gy25_info}'
            )

            if abs(remaining_sum) <= tolerance_sum:
                break

            command_speed = int(kp * abs(remaining_sum))
            command_speed = max(min_speed, min(max_speed, command_speed))
            signed_speed = direction * command_speed

            if last_command_speed != signed_speed:
                self.get_logger().info(
                    f'Forward control: remaining_sum={remaining_sum:.1f} | speed={signed_speed}'
                )
                self.start_motors(signed_speed, signed_speed)
                last_command_speed = signed_speed

        self.stop_motors()
        self.get_logger().info(
            f'Forward end: traveled_sum={self.traveled_encoder_sum():.1f} | target_sum={target_sum:.1f}'
        )

        if delay_after > 0:
            self.get_logger().info(f'Waiting {delay_after} seconds before next action...')
            self.wait(delay_after)

    def rotate(self, angle, max_turn_speed=20, min_turn_speed=5, kp=0.35, angle_tolerance=5, delay_after=1.0):
        """Повернуть робот на заданный угол по данным gy25 с P-регулятором."""
        if self.gy25_angle is None or self.gy25_start_angle is None:
            self.get_logger().warn('GY25 angle is not initialized yet, waiting...')
            self.wait_for_gy25_initialization()

        if angle == 0:
            self.get_logger().info('Rotate skipped: target angle is 0')
            return

        self.target_relative_angle += angle
        target_angle = self.gy25_start_angle + self.target_relative_angle
        self.get_logger().info(
            f'Rotate start: start={self.gy25_start_angle} | delta={angle} | target={target_angle}'
        )

        last_command_speed = None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.gy25_angle is None:
                continue

            remaining = target_angle - self.gy25_angle
            self.get_logger().info(
                f'Rotate progress: current={self.gy25_angle} | target={target_angle} | remaining={remaining}'
            )

            if abs(remaining) <= angle_tolerance:
                break

            command_speed = int(kp * abs(remaining))
            command_speed = max(min_turn_speed, min(max_turn_speed, command_speed))
            direction = 1 if remaining > 0 else -1

            if last_command_speed != direction * command_speed:
                self.get_logger().info(
                    f'Rotate control: remaining={remaining} | speed={command_speed} | direction={direction}'
                )
                if direction > 0:
                    self.start_motors(-command_speed, command_speed)
                else:
                    self.start_motors(command_speed, -command_speed)
                last_command_speed = direction * command_speed

        self.stop_motors()
        self.get_logger().info(
            f'Rotate end: current={self.gy25_angle} | target={target_angle}'
        )

        if delay_after > 0:
            self.get_logger().info(f'Waiting {delay_after} seconds before next action...')
            self.wait(delay_after)

    def algorithm(self):

        def left(angle):
            self.rotate(angle, max_turn_speed=20, delay_after=1.0)

        def delay(time_sec):
            self.wait(time_sec)

        """Основной алгоритм: последовательные шаги с ожиданием."""
        self.get_logger().info('Algorithm: waiting for encoder initialization...')
        self.wait_for_encoder_initialization()
        self.get_logger().info('Algorithm: initialization done, starting motors')

        for step in self.target_way:
            if step == 'f':
                self.forward(80)
            elif step == 'l':
                left(90)
            elif step == 'r':
                left(-90)
            elif step == 's':
                delay(3)
            else:
                self.get_logger().warn(f'Unknown step in target_way: {step}')

        # delay(4)
        # self.forward(320)
        # left(-180)
        # self.forward(80)
        # self.forward(160)
        # self.forward(80)
        # left(180)

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
