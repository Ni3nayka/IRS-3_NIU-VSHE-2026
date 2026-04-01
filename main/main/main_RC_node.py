#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard
import sys

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Создаем издатель в стандартный топик управления скоростью
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Текущее состояние скоростей (по умолчанию 0)
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
        # Набор активных клавиш (чтобы поддерживать нажатие нескольких кнопок)
        self.active_keys = set()
        
        # Параметры скоростей согласно заданию
        self.speed_linear = 0.3
        self.speed_angular = 0.2
        
        # Таймер для периодической отправки команды (10 Гц)
        # Это надежнее, чем отправка только в момент нажатия
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Запускаем слушатель клавиатуры в отдельном потоке
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()
        
        self.get_logger().info('Teleop node started. Use WASDQE to move, SPACE to stop, ESC to exit.')

    def timer_callback(self):
        """Периодически публикует текущую скорость в топик /cmd_vel"""
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.linear.y = self.linear_y
        msg.angular.z = self.angular_z
        self.cmd_pub.publish(msg)

    def update_velocity(self):
        """Пересчитывает скорость на основе набора активных клавиш"""
        # Сбрасываем скорости перед пересчетом
        vx = 0.0
        vy = 0.0
        vz = 0.0
        
        # Обработка оси X (Вперед/Назад)
        # Если нажаты обе, приоритет не задан, но обычно они взаимоисключающие.
        # Здесь логика: если есть 'w' и нет 's' -> вперед. Если 's' и нет 'w' -> назад.
        if 'w' in self.active_keys and 's' not in self.active_keys:
            vx = self.speed_linear
        elif 's' in self.active_keys and 'w' not in self.active_keys:
            vx = -self.speed_linear
        # Если нажаты обе одновременно или ни одной -> 0 (безопасное поведение)

        # Обработка оси Y (Стрейф влево/вправо)
        if 'q' in self.active_keys and 'e' not in self.active_keys:
            vy = -self.speed_linear # q - влево (отрицательный Y в базе робота обычно)
        elif 'e' in self.active_keys and 'q' not in self.active_keys:
            vy = self.speed_linear  # e - вправо

        # Обработка оси Z (Поворот)
        if 'a' in self.active_keys and 'd' not in self.active_keys:
            vz = -self.speed_angular # a - влево
        elif 'd' in self.active_keys and 'a' not in self.active_keys:
            vz = self.speed_angular  # d - вправо

        # Применяем новые значения
        self.linear_x = vx
        self.linear_y = vy
        self.angular_z = vz

    def on_press(self, key):
        """Обработчик нажатия клавиши"""
        try:
            # Получаем символ клавиши
            k = key.char.lower()
        except AttributeError:
            # Для специальных клавиш (пробел, esc)
            if key == keyboard.Key.space:
                self.emergency_stop()
                return
            elif key == keyboard.Key.esc:
                self.get_logger().info('Exit requested via ESC')
                self.listener.stop()
                rclpy.shutdown()
                return
            else:
                return

        # Добавляем клавишу в активные и пересчитываем скорость
        if k in ['w', 's', 'a', 'd', 'q', 'e']:
            self.active_keys.add(k)
            self.update_velocity()
            self.get_logger().debug(f'Key pressed: {k}, Active: {self.active_keys}')

    def on_release(self, key):
        """Обработчик отжатия клавиши"""
        try:
            k = key.char.lower()
        except AttributeError:
            return

        # Удаляем клавишу из активных и пересчитываем скорость
        # Это реализует требование: "кнопка отжата -- скорость 0"
        if k in self.active_keys:
            self.active_keys.remove(k)
            self.update_velocity()
            self.get_logger().debug(f'Key released: {k}, Active: {self.active_keys}')

    def emergency_stop(self):
        """Экстренная остановка по пробелу"""
        self.active_keys.clear()
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.get_logger().warn('EMERGENCY STOP!')

    def destroy_node(self):
        """Гарантированная остановка при закрытии ноды"""
        self.emergency_stop()
        # Отправляем команду остановки несколько раз, чтобы убедиться, что она дошла
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.1)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    
    try:
        # Запускаем цикл обработки событий ROS 2
        # Слушатель клавиатуры работает в фоновом потоке
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Очистка ресурсов
        node.listener.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()