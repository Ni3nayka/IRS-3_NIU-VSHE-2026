'''
sudo apt update
sudo apt install -y ros-jazzy-rplidar-ros
'''

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan

class LidarForwarder(Node):
    def __init__(self):
        super().__init__('lidar_forwarder')
        
        # Топик для публикации (по заданию)
        self.custom_topic_name = 'RplidarC1'
        
        # Топик от драйвера RPLIDAR (стандартный)
        self.driver_topic_name = '/scan'
        
        # Создаем издатель в кастомный топик
        self.lidar_pub = self.create_publisher(LaserScan, self.custom_topic_name, 10)
        
        # Подписываемся на стандартный топик драйвера
        self.lidar_sub = self.create_subscription(
            LaserScan,
            self.driver_topic_name,
            self.lidar_callback,
            10
        )
        
        self.frame_count = 0
        self.get_logger().info(f'Lidar Forwarder started')
        self.get_logger().info(f'Listening on: {self.driver_topic_name}')
        self.get_logger().info(f'Publishing to: {self.custom_topic_name}')

    def lidar_callback(self, msg):
        """Получение данных от драйвера и публикация в кастомный топик"""
        # Просто перепубликуем сообщение как есть
        self.lidar_pub.publish(msg)
        self.frame_count += 1
        
        # Логирование каждые 10 сканов
        if self.frame_count % 10 == 0:
            self.get_logger().info(f'Forwarded {self.frame_count} scans')

    def destroy_node(self):
        if rclpy.ok():
            self.get_logger().info(f'Lidar Forwarder stopped. Total scans: {self.frame_count}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarForwarder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
