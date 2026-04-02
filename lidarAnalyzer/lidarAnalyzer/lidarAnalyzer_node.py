#!/usr/bin/env python3
"""
Wall Detector Node
Подписывается на данные лидара и постоянно публикует результаты анализа по 4 секторам.
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SimpleFilter:
    """Простой экспоненциальный фильтр (EMA)."""

    def __init__(self, alpha=0.6):
        self.alpha = alpha
        self.value = None
        self.initialized = False

    def update(self, new_value):
        if not self.initialized:
            self.value = new_value
            self.initialized = True
            return new_value

        if math.isinf(new_value) or new_value < 0:
            self.initialized = False
            return new_value

        self.value = self.alpha * self.value + (1 - self.alpha) * new_value
        return self.value

    def reset(self):
        self.value = None
        self.initialized = False


class WallDetectorNode(Node):
    def __init__(self):
        super().__init__('wall_detector_node')

        self.sectors = {
            'front': (0.0, math.radians(30)),
            'left': (math.pi / 2, math.radians(30)),
            'right': (-math.pi / 2, math.radians(30)),
            'back': (math.pi, math.radians(30)),
        }

        self.declare_parameter('min_valid_points', 5)
        self.declare_parameter('max_distance_m', 10.0)
        self.declare_parameter('ema_alpha', 0.6)
        self.declare_parameter('use_filter', True)

        self.min_valid_points = self.get_parameter('min_valid_points').value
        self.max_distance_m = self.get_parameter('max_distance_m').value
        self.alpha = self.get_parameter('ema_alpha').value
        self.use_filter = self.get_parameter('use_filter').value

        self.filters = {
            'front': SimpleFilter(self.alpha),
            'left': SimpleFilter(self.alpha),
            'right': SimpleFilter(self.alpha),
            'back': SimpleFilter(self.alpha),
        }

        self.lidar_sub = self.create_subscription(
            LaserScan,
            'RplidarC1',
            self.lidar_callback,
            10,
        )

        self.analyzer_pub = self.create_publisher(
            LaserScan,
            'lidarAnalyzer',
            10,
        )

        self.get_logger().info('Wall Detector started')
        self.get_logger().info('  Subscribe: /RplidarC1')
        self.get_logger().info('  Publish:   /lidarAnalyzer')
        self.get_logger().info(f'  Filter enabled: {self.use_filter}')
        self.get_logger().info(f'  EMA alpha: {self.alpha}')

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _get_sector_indices(self, scan, center_angle, half_width):
        indices = []
        norm_center = self._normalize_angle(center_angle)

        for i in range(len(scan.ranges)):
            angle = scan.angle_min + i * scan.angle_increment
            norm_angle = self._normalize_angle(angle)
            diff = abs(self._normalize_angle(norm_angle - norm_center))

            if diff <= half_width:
                indices.append(i)

        return indices

    def _analyze_sector(self, scan, center_angle, half_width):
        indices = self._get_sector_indices(scan, center_angle, half_width)

        if len(indices) < self.min_valid_points:
            return {'distance': float('inf'), 'angle': center_angle, 'valid': False}

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

        min_idx = int(np.argmin(valid_dists))
        return {
            'distance': valid_dists[min_idx],
            'angle': valid_angles[min_idx],
            'valid': True,
        }

    def _apply_filter(self, direction, distance):
        if not self.use_filter:
            return distance

        if math.isinf(distance) or distance < 0:
            self.filters[direction].reset()
            return distance

        return self.filters[direction].update(distance)

    def lidar_callback(self, msg):
        results = {}

        for direction, (center, width) in self.sectors.items():
            results[direction] = self._analyze_sector(msg, center, width)
            raw_distance = results[direction]['distance']
            results[direction]['distance'] = self._apply_filter(direction, raw_distance)

        self._publish_results(results, msg.header.stamp)

    def _publish_results(self, results, stamp):
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = 'lidar_analyzer_frame'
        msg.ranges = [
            results['front']['distance'], results['front']['angle'],
            results['left']['distance'], results['left']['angle'],
            results['right']['distance'], results['right']['angle'],
            results['back']['distance'], results['back']['angle'],
        ]
        msg.intensities = [
            1.0 if results['front']['valid'] else 0.0,
            1.0 if results['left']['valid'] else 0.0,
            1.0 if results['right']['valid'] else 0.0,
            1.0 if results['back']['valid'] else 0.0,
        ]

        self.analyzer_pub.publish(msg)
        self.get_logger().debug(
            f'F:{results["front"]["distance"]:.2f}m '
            f'L:{results["left"]["distance"]:.2f}m '
            f'R:{results["right"]["distance"]:.2f}m '
            f'B:{results["back"]["distance"]:.2f}m'
        )

    def destroy_node(self):
        self.get_logger().info('Wall Detector stopped')
        super().destroy_node()


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
