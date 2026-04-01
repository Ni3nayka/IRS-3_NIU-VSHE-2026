# код для управления роботом с помощью клавитатуры (работает без графической оболочки, но костыли имеются)
 
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import threading
import time

class KeyboardTeleopPI(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_pi')
        
        # Топик управления
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Параметры скоростей
        self.speed_linear = 0.3
        self.speed_angular = 0.2
        
        # Текущие скорости
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
        # Флаг работы
        self.running = True
        
        # Время последнего нажатия (для таймера безопасности)
        self.last_key_time = time.time()
        self.timeout = 0.3  # Секунды без нажатия -> стоп
        
        # Сохраняем настройки терминала для восстановления
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        
        # Таймер публикации (10 Гц)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Таймер проверки безопасности (проверяет, не пора ли остановиться)
        self.safety_timer = self.create_timer(0.05, self.safety_callback)
        
        self.get_logger().info('PI Teleop started. WASDQE to move. SPACE=Stop. CTRL+C=Exit.')
        self.print_help()

    def print_help(self):
        print("\n--- Управление ---")
        print("W : Вперёд (X+)")
        print("S : Назад (X-)")
        print("Q : Влево боком (Y+)")
        print("E : Вправо боком (Y-)")
        print("A : Поворот влево (Z+)")
        print("D : Поворот вправо (Z-)")
        print("SPACE : Экстренный стоп")
        print("CTRL+C : Выход")
        print("----------------\n")

    def timer_callback(self):
        """Публикация текущей скорости"""
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.linear.y = self.linear_y
        msg.angular.z = self.angular_z
        self.cmd_pub.publish(msg)

    def safety_callback(self):
        """
        Проверка тайм-аута. 
        Если клавиши не нажимались дольше self.timeout секунд -> стоп.
        """
        if time.time() - self.last_key_time > self.timeout:
            if self.linear_x != 0.0 or self.linear_y != 0.0 or self.angular_z != 0.0:
                self.linear_x = 0.0
                self.linear_y = 0.0
                self.angular_z = 0.0
                # self.get_logger().debug('Safety timeout: stopping')

    def get_key(self):
        """Чтение одного символа без буферизации (RAW режим)"""
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        return ch

    def process_key(self, key):
        """Обработка нажатой клавиши"""
        key = key.lower()
        self.last_key_time = time.time()  # Сброс таймера безопасности

        if key == 'w':
            self.linear_x = self.speed_linear
            self.linear_y = 0.0
            self.angular_z = 0.0
        elif key == 's':
            self.linear_x = -self.speed_linear
            self.linear_y = 0.0
            self.angular_z = 0.0
        elif key == 'q':
            # Движение влево боком (положительный Y в системе координат робота)
            self.linear_x = 0.0
            self.linear_y = self.speed_linear
            self.angular_z = 0.0
        elif key == 'e':
            # Движение вправо боком (отрицательный Y в системе координат робота)
            self.linear_x = 0.0
            self.linear_y = -self.speed_linear
            self.angular_z = 0.0
        elif key == 'a':
            # Поворот влево (положительный Z - против часовой стрелки)
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = self.speed_angular
        elif key == 'd':
            # Поворот вправо (отрицательный Z - по часовой стрелке)
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = -self.speed_angular
        elif key == ' ':
            self.emergency_stop()
        elif key == '\x03':  # Ctrl+C
            self.running = False
        else:
            # Неизвестная клавиша - не сбрасываем движение, просто игнорируем
            pass

    def emergency_stop(self):
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.last_key_time = time.time()
        self.get_logger().warn('EMERGENCY STOP!')

    def input_loop(self):
        """Поток для чтения ввода с клавиатуры"""
        try:
            while self.running and rclpy.ok():
                try:
                    key = self.get_key()
                    if key:
                        self.process_key(key)
                except Exception as e:
                    # Игнорируем ошибки чтения, если нода закрывается
                    break
        finally:
            # Восстанавливаем настройки терминала при выходе
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def destroy_node(self):
        """Очистка при выходе"""
        self.emergency_stop()
        # Гарантированная отправка нуля
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Восстанавливаем терминал
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopPI()
    
    # Запускаем поток ввода
    input_thread = threading.Thread(target=node.input_loop)
    input_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        input_thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()