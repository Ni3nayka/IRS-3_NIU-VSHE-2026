#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray
from geometry_msgs.msg import Twist
import serial
import serial.tools.list_ports
import time

class ArduinoController(Node):
    def __init__(self):
        super().__init__('arduino_controller')

        # Параметры
        self.declare_parameter('port', '')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('read_rate', 20.0)
        self.declare_parameter('enc_topic', 'encoders')
        self.declare_parameter('reconnect_delay', 2.0)  # задержка перед повторным подключением (сек)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.read_rate = self.get_parameter('read_rate').get_parameter_value().double_value
        self.enc_topic = self.get_parameter('enc_topic').get_parameter_value().string_value
        self.reconnect_delay = self.get_parameter('reconnect_delay').get_parameter_value().double_value

        # Переменные для хранения последних полученных значений движения и серв
        self.forward_speed = 0
        self.left_speed = 0
        self.rotation_speed = 0
        self.servo_angle_1 = 470
        self.servo_angle_2 = 180

        # Подписки
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.sub_servo1 = self.create_subscription(Int32, 'servo_angle_1', self.servo_1_callback, 10)
        self.sub_servo2 = self.create_subscription(Int32, 'servo_angle_2', self.servo_2_callback, 10)

        # Публикатор для энкодеров (массив из 4 int)
        self.pub_encoders = self.create_publisher(Int32MultiArray, self.enc_topic, 10)

        # Состояние подключения
        self.serial_conn = None
        self.last_reconnect_attempt = 0.0
        self.connect_serial()

        # Таймеры
        self.timer_send = self.create_timer(1.0 / self.rate, self.send_commands)
        self.timer_read = self.create_timer(1.0 / self.read_rate, self.read_serial)

        self.get_logger().info('Arduino controller node started')

    def connect_serial(self):
        """Устанавливает соединение с Arduino."""
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except:
                pass
            self.serial_conn = None

        port = self.port
        if not port:
            # Автоматическое определение
            ports = serial.tools.list_ports.comports()
            self.get_logger().info(f'Found ports: {[p.device for p in ports]}')
            arduino_ports = [p.device for p in ports if 'Arduino' in p.description or 'usb' in p.device]
            if not arduino_ports:
                self.get_logger().error('Arduino not found')
                return
            port = arduino_ports[0]
            self.get_logger().info(f'Using automatically detected port: {port}')

        try:
            self.serial_conn = serial.Serial(port, self.baudrate, timeout=0.1)
            time.sleep(2)  # даём Arduino время на инициализацию
            self.get_logger().info(f'Connected to {port} at {self.baudrate} baud')
            self.last_reconnect_attempt = self.get_clock().now().nanoseconds / 1e9
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.serial_conn = None

    def reconnect_serial(self):
        """Пытается переподключиться, но не чаще чем раз в reconnect_delay секунд."""
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_reconnect_attempt >= self.reconnect_delay:
            self.get_logger().warn('Attempting to reconnect to Arduino...')
            self.connect_serial()
            self.last_reconnect_attempt = now
        else:
            self.get_logger().debug('Reconnect attempt too soon, skipping')

    def cmd_vel_callback(self, msg):
        self.forward_speed = int(round(msg.linear.x))
        self.left_speed = int(round(msg.linear.y))
        self.rotation_speed = int(round(msg.angular.z))
        self.get_logger().info(f'{self.forward_speed} {self.left_speed} {self.rotation_speed}')

    def servo_1_callback(self, msg):
        self.servo_angle_1 = msg.data

    def servo_2_callback(self, msg):
        self.servo_angle_2 = msg.data

    def send_commands(self):
        """Отправляет команды на Arduino, если соединение активно."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            return

        motor_cmd = f"v {int(self.forward_speed*100)} {int(self.left_speed*100)} {int(self.rotation_speed*100)}\n"
        # servo_cmd = f"A {self.servo_angle_1} {self.servo_angle_2}\n"

        try:
            self.serial_conn.write(motor_cmd.encode())
            self.get_logger().info(motor_cmd)
            # self.serial_conn.write(servo_cmd.encode())
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.reconnect_serial()

    def read_serial(self):
        """Читает данные из последовательного порта, обрабатывает ошибки."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            return

        try:
            while self.serial_conn.in_waiting > 0:
                try:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.process_line(line)
                except UnicodeDecodeError:
                    self.get_logger().debug('Unicode decode error, skipping line')
                except Exception as e:
                    self.get_logger().error(f'Error reading line: {e}')
                    # Не выходим, продолжаем чтение других строк
        except OSError as e:
            self.get_logger().error(f'Serial I/O error: {e}')
            self.reconnect_serial()
        except Exception as e:
            self.get_logger().error(f'Unexpected error in read_serial: {e}')
            self.reconnect_serial()

    def process_line(self, line):
        """Обрабатывает одну строку, полученную от Arduino."""
        if line.startswith('ENC:'):
            parts = line.replace('ENC:', '', 1).split()
            if len(parts) >= 4:
                try:
                    encoders = [int(parts[i]) for i in range(4)]
                    self.pub_encoders.publish(Int32MultiArray(data=encoders))
                    self.get_logger().debug(f'Published encoders: {encoders}')
                except ValueError:
                    self.get_logger().warn(f'Invalid encoder values: {parts}')
            else:
                self.get_logger().warn(f'Malformed ENC line: {line}')
        # Здесь можно добавить обработку других возможных форматов от Arduino

    def destroy_node(self):
        """Закрывает последовательный порт при завершении узла."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoController()
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
