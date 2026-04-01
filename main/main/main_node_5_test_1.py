#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from concurrent.futures import Future

class MotorTestEncoder(Node):
    def __init__(self):
        super().__init__('motor_test_encoder')
        self.pub_left = self.create_publisher(Int32, 'motor_speed_left', 10)
        self.pub_right = self.create_publisher(Int32, 'motor_speed_right', 10)
        self.pub_servo_1 = self.create_publisher(Int32, 'servo_angle_1', 10)
        self.pub_servo_2 = self.create_publisher(Int32, 'servo_angle_2', 10)
        self.sub_enc_left = self.create_subscription(Int32, 'encoder_left', self.enc_left_callback, 10)
        self.sub_enc_right = self.create_subscription(Int32, 'encoder_right', self.enc_right_callback, 10)

        self.enc_left = 0
        self.enc_right = 0
        self.enc_left_start = None
        self.enc_right_start = None
        self.init_future = Future()
        self.target_future = Future()
        self.target_sum = 0

    def enc_left_callback(self, msg):
        self.enc_left = msg.data
        if self.enc_left_start is None:
            self.enc_left_start = msg.data
            self.get_logger().info(f'Initial left encoder: {self.enc_left_start}')
            self._check_init()
        self._check_target()

    def enc_right_callback(self, msg):
        self.enc_right = msg.data
        if self.enc_right_start is None:
            self.enc_right_start = msg.data
            self.get_logger().info(f'Initial right encoder: {self.enc_right_start}')
            self._check_init()
        self._check_target()

    def _check_init(self):
        if (self.enc_left_start is not None and self.enc_right_start is not None and
                not self.init_future.done()):
            self.init_future.set_result(True)

    def _check_target(self):
        if (self.enc_left_start is None or self.enc_right_start is None or
                self.target_future.done()):
            return
        traveled_left = abs(self.enc_left - self.enc_left_start)
        traveled_right = abs(self.enc_right - self.enc_right_start)
        total = traveled_left + traveled_right
        self.get_logger().info(f'Traveled: left={traveled_left}, right={traveled_right}, total={total}')
        if total >= self.target_sum:
            self.target_future.set_result(True)

    def start_motors(self, left_speed, right_speed):
        self.pub_left.publish(Int32(data=left_speed))
        self.pub_right.publish(Int32(data=right_speed))
        self.get_logger().info(f'Motors started: left={left_speed}, right={right_speed}')

    def stop_motors(self):
        self.pub_left.publish(Int32(data=0))
        self.pub_right.publish(Int32(data=0))
        self.get_logger().info('Motors stopped')

    def wait(self, duration):
        """Ждать duration секунд, обрабатывая входящие сообщения ROS2."""
        future = Future()
        timer = self.create_timer(duration, lambda: future.set_result(True))
        rclpy.spin_until_future_complete(self, future)
        timer.cancel()
        self.get_logger().debug(f'Wait of {duration}s finished')

    def move(self, left_speed, right_speed, enc_delta, delay_after=1.0):
        """Выполнить движение на заданное расстояние (в импульсах энкодеров)."""
        # Сначала создаём новый future, чтобы избежать состояния гонки
        self.target_future = Future()
        # Запоминаем текущие значения энкодеров как стартовые для этого движения
        self.enc_left_start = self.enc_left
        self.enc_right_start = self.enc_right
        self.target_sum = enc_delta

        self.get_logger().info('Robot move - start')
        self.start_motors(left_speed, right_speed)
        rclpy.spin_until_future_complete(self, self.target_future)
        self.get_logger().info('Robot move - end')
        self.stop_motors()

        if delay_after > 0:
            self.get_logger().info(f'Waiting {delay_after} seconds before next action...')
            self.wait(delay_after)

    def algorithm(self):
        """Основной алгоритм: последовательные шаги с ожиданием."""
        self.get_logger().info('Algorithm: waiting for encoder initialization...')
        rclpy.spin_until_future_complete(self, self.init_future)
        self.get_logger().info('Algorithm: initialization done, starting motors')

        # START
        self.pub_servo_1.publish(Int32(data=80))
        self.pub_servo_2.publish(Int32(data=180))
        self.wait(30.0)
        self.pub_servo_1.publish(Int32(data=470))
        self.wait(2.0)
        self.pub_servo_2.publish(Int32(data=180))

        # MAIN
        self.move(20, 20, 100, delay_after=2.0)
        self.move(-20, 20, 1150, delay_after=2.0)
        self.move(20, 20, 1000, delay_after=2.0)
        self.move(20, -20, 1150, delay_after=2.0)
        self.move(20, 20, 10000, delay_after=2.0)
        # self.move(20, 20, 1000, delay_after=2.0)
        # self.move(20, -20, 1000, delay_after=2.0)
        self.start_motors(10, 10)
        # for i in range (180,90,10):
        #     self.pub_servo_2.publish(Int32(data=i))
        #     self.wait(0.2)
        self.pub_servo_2.publish(Int32(data=85))
        self.wait(2.0)
        self.start_motors(0, 0)
        self.pub_servo_1.publish(Int32(data=300))
        self.wait(2.0)
        self.move(-20, -20, 19000, delay_after=2.0)
        self.move(20, 20, 100, delay_after=2.0)
        self.move(-20, 20, 1400, delay_after=2.0)
        self.move(20, 20, 6000, delay_after=2.0)
        self.pub_servo_1.publish(Int32(data=470))
        self.wait(2.0)
        self.pub_servo_2.publish(Int32(data=180))
        self.wait(2.0)
        self.move(-20, -20, 3000, delay_after=2.0)
        
        # END
        self.wait(2.0)
        self.pub_servo_1.publish(Int32(data=470))
        self.pub_servo_2.publish(Int32(data=180))

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