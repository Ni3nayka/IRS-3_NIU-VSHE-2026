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
        self.declare_parameter('gy25_topic', 'gy25')
        self.declare_parameter('reconnect_delay', 2.0)  # задержка перед повторным подключением (сек)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.read_rate = self.get_parameter('read_rate').get_parameter_value().double_value
        self.enc_topic = self.get_parameter('enc_topic').get_parameter_value().string_value
        self.gy25_topic = self.get_parameter('gy25_topic').get_parameter_value().string_value
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
        self.pub_gy25 = self.create_publisher(Int32, self.gy25_topic, 10)

        # Состояние подключения
        self.serial_conn = None
        self.last_reconnect_attempt = 0.0
        self.connect_serial()

        # Таймеры
        self.timer_send = self.create_timer(1.0 / self.rate, self.send_commands)
        self.timer_read = self.create_timer(1.0 / self.read_rate, self.read_serial)

        self.get_logger().info('Arduino controller node started')

    def port_matches_keywords(self, port_info, keywords):
        values = [
            getattr(port_info, 'device', '') or '',
            getattr(port_info, 'description', '') or '',
            getattr(port_info, 'manufacturer', '') or '',
            getattr(port_info, 'product', '') or '',
            getattr(port_info, 'hwid', '') or '',
        ]
        haystack = ' '.join(values).lower()
        return any(keyword in haystack for keyword in keywords)

    def score_arduino_port(self, port_info):
        """Оценивает, насколько порт похож на Arduino, а не на лидар."""
        score = 0

        arduino_keywords = [
            'arduino',
            'ch340',
            'ch341',
            'usb2.0-serial',
            '1a86:7523',
        ]
        lidar_keywords = [
            'silicon labs',
            'cp210',
            '10c4:ea60',
            'rplidar',
            'slamtec',
        ]

        if self.port_matches_keywords(port_info, arduino_keywords):
            score += 10
        if self.port_matches_keywords(port_info, lidar_keywords):
            score -= 20

        return score

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
            self.get_logger().info(
                'Found ports: ' +
                str([
                    {
                        'device': p.device,
                        'description': p.description,
                        'manufacturer': getattr(p, 'manufacturer', None),
                        'product': getattr(p, 'product', None),
                        'hwid': p.hwid,
                    }
                    for p in ports
                ])
            )

            scored_ports = sorted(
                ((self.score_arduino_port(p), p) for p in ports if p.device.startswith('/dev/ttyUSB') or p.device.startswith('/dev/ttyACM')),
                key=lambda item: item[0],
                reverse=True,
            )

            if not scored_ports or scored_ports[0][0] < 0:
                self.get_logger().error('Arduino not found')
                return

            best_score, best_port = scored_ports[0]
            port = best_port.device
            self.get_logger().info(
                f'Using automatically detected Arduino port: {port} '
                f'(description={best_port.description}, score={best_score})'
            )

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
        self.forward_speed = msg.linear.x
        self.left_speed = msg.linear.y
        self.rotation_speed = msg.angular.z
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
            gy25_section = None
            enc_section = line
            if 'GY25:' in line:
                enc_section, gy25_section = line.split('GY25:', 1)

            parts = enc_section.replace('ENC:', '', 1).split()
            if len(parts) >= 4:
                try:
                    encoders = [int(parts[i]) for i in range(4)]
                    self.pub_encoders.publish(Int32MultiArray(data=encoders))
                    self.get_logger().debug(f'Published encoders: {encoders}')
                except ValueError:
                    self.get_logger().warn(f'Invalid encoder values: {parts}')
            else:
                self.get_logger().warn(f'Malformed ENC line: {line}')

            if gy25_section is not None:
                self.publish_gy25(gy25_section.strip(), line)
        elif line.startswith('GY25:'):
            self.publish_gy25(line.replace('GY25:', '', 1).strip(), line)
        # Здесь можно добавить обработку других возможных форматов от Arduino

    def publish_gy25(self, gy25_data, raw_line):
        """Публикует угол GY25 в отдельный топик."""
        parts = gy25_data.split()
        if not parts:
            self.get_logger().warn(f'Malformed GY25 line: {raw_line}')
            return

        try:
            angle = int(parts[0])
            self.pub_gy25.publish(Int32(data=angle))
            self.get_logger().debug(f'Published GY25 angle: {angle}')
        except ValueError:
            self.get_logger().warn(f'Invalid GY25 angle: {gy25_data}')

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
