#!/usr/bin/env python3
'''
Wall Detector Node
Простая нода: подписка на лидар → анализ → публикация результатов

Подписка: /RplidarC1 (sensor_msgs/msg/LaserScan)
Публикация: /lidarAnalyzer (custom формат в LaserScan)
'''

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class WallDetectorNode(Node):
    def __init__(self):
        super().__init__('wall_detector_node')
        
        # === НАСТРОЙКИ СЕКТОРОВ (в радианах) ===
        # (центральный_угол, полуширина_сектора)
        self.sectors = {
            'front':  (0.0, math.radians(30)),      # ±30° от носа
            'left':   (math.pi/2, math.radians(30)), # ±30° от левого борта
            'right':  (-math.pi/2, math.radians(30)),# ±30° от правого борта
            'back':   (math.pi, math.radians(30))    # ±30° от кормы
        }
        
        # === ПАРАМЕТРЫ ===
        self.min_valid_points = 5       # Мин. точек в секторе для валидного результата
        self.max_distance_m = 10.0      # Макс. дистанция для "стены"
        
        # === ПОДПИСКА И ПУБЛИКАЦИЯ ===
        self.lidar_sub = self.create_subscription(
            LaserScan,
            'RplidarC1',
            self.lidar_callback,
            10
        )
        
        self.analyzer_pub = self.create_publisher(
            LaserScan,
            'lidarAnalyzer',
            10
        )
        
        self.get_logger().info('Wall Detector started')
        self.get_logger().info('  Subscribe: /RplidarC1')
        self.get_logger().info('  Publish:   /lidarAnalyzer')

    def _normalize_angle(self, angle):
        """Нормализация угла к диапазону [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _get_sector_indices(self, scan, center_angle, half_width):
        """Получение индексов точек, попадающих в сектор"""
        indices = []
        for i in range(len(scan.ranges)):
            angle = scan.angle_min + i * scan.angle_increment
            norm_angle = self._normalize_angle(angle)
            norm_center = self._normalize_angle(center_angle)
            diff = abs(self._normalize_angle(norm_angle - norm_center))
            
            if diff <= half_width:
                indices.append(i)
        return indices

    def _analyze_sector(self, scan, center_angle, half_width):
        """Анализ одного сектора: поиск ближайшей стены"""
        indices = self._get_sector_indices(scan, center_angle, half_width)
        
        if len(indices) < self.min_valid_points:
            return {'distance': float('inf'), 'angle': center_angle, 'valid': False}
        
        # Собираем валидные дистанции
        valid_dists = []
        valid_angles = []
        
        for idx in indices:
            dist = scan.ranges[idx]
            if 0 < dist < self.max_distance_m and not math.isinf(dist):
                valid_dists.append(dist)
                angle = scan.angle_min + idx * scan.angle_increment
                valid_angles.append(self._normalize_angle(angle))
        
        if len(valid_dists) < self.min_valid_points:
            return {'distance': float('inf'), 'angle': center_angle, 'valid': False}
        
        # Берём минимальную дистанцию (ближайшее препятствие)
        min_idx = np.argmin(valid_dists)
        return {
            'distance': valid_dists[min_idx],
            'angle': valid_angles[min_idx],
            'valid': True
        }

    def lidar_callback(self, msg: LaserScan):
        """Обработка нового скана с лидара"""
        results = {}
        
        # Анализируем все 4 направления
        for direction, (center, width) in self.sectors.items():
            results[direction] = self._analyze_sector(msg, center, width)
        
        # Публикуем результат в топик lidarAnalyzer
        self._publish_results(results, msg.header.stamp)

    def _publish_results(self, results, stamp):
        """Публикация результатов в топик"""
        # Используем LaserScan для простоты (совместимость с ROS 2)
        # Формат ranges: [dist_front, angle_front, dist_left, angle_left, ...]
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = 'lidar_analyzer_frame'
        
        # Кодируем 4 направления в массив
        msg.ranges = [
            results['front']['distance'], results['front']['angle'],
            results['left']['distance'],  results['left']['angle'],
            results['right']['distance'], results['right']['angle'],
            results['back']['distance'],  results['back']['angle']
        ]
        
        # Флаги валидности в intensities
        msg.intensities = [
            1.0 if results['front']['valid'] else 0.0,
            1.0 if results['left']['valid'] else 0.0,
            1.0 if results['right']['valid'] else 0.0,
            1.0 if results['back']['valid'] else 0.0
        ]
        
        self.analyzer_pub.publish(msg)
        
        # Логирование для отладки
        self.get_logger().debug(
            f'F:{results["front"]["distance"]:.2f}m '
            f'L:{results["left"]["distance"]:.2f}m '
            f'R:{results["right"]["distance"]:.2f}m '
            f'B:{results["back"]["distance"]:.2f}m'
        )

def main(args=None):
    rclpy.init(args=args)
    node = WallDetectorNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        if rclpy.ok():
            node.get_logger().info('Stopped by user')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
