#!/usr/bin/env python3
'''
RPLidar C1 ROS 2 Node
Прямое подключение через rplidarc1 библиотеку (как в LIdar.py)

Установка:
pip3 install rplidarc1 numpy
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rplidarc1.scanner import RPLidar
import threading
import time
import math
import asyncio
import queue as pyqueue
import numpy as np

class RPLidarNode(Node):
    def __init__(self):
        super().__init__('rplidar_node')
        
        # === НАСТРОЙКИ ===
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 460800)
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('max_distance_mm', 12000)
        self.declare_parameter('min_quality', 0)
        self.declare_parameter('max_points', 1800)
        self.declare_parameter('queue_drop_threshold', 8000)
        self.declare_parameter('drain_max', 5000)
        
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.publish_hz = self.get_parameter('publish_hz').value
        self.max_distance_mm = self.get_parameter('max_distance_mm').value
        self.min_quality = self.get_parameter('min_quality').value
        self.max_points = self.get_parameter('max_points').value
        self.queue_drop_threshold = self.get_parameter('queue_drop_threshold').value
        self.drain_max = self.get_parameter('drain_max').value
        
        # === ПУБЛИШЕР ===
        self.lidar_pub = self.create_publisher(LaserScan, 'RplidarC1', 10)
        
        # === СОСТОЯНИЕ ===
        self.running = True
        self.latest_scan = None
        self.scan_lock = threading.Lock()
        self.publish_count = 0
        
        # === ИНИЦИАЛИЗАЦИЯ ЛИДАРА ===
        self._init_lidar()
        
        # === ТАЙМЕР ПУБЛИКАЦИИ ===
        self.publish_timer = self.create_timer(1.0 / self.publish_hz, self.publish_callback)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('RPLIDAR C1 NODE')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Port: {self.port}')
        self.get_logger().info(f'Baudrate: {self.baudrate}')
        self.get_logger().info(f'Publish Hz: {self.publish_hz}')
        self.get_logger().info(f'Topic: RplidarC1')
        self.get_logger().info('=' * 50)

    def _init_lidar(self):
        """Инициализация лидара в отдельном потоке"""
        try:
            self.get_logger().info(f'🔍 Connecting to LiDAR: {self.port}...')
            
            # Проверяем доступность порта
            import os
            if not os.path.exists(self.port):
                raise FileNotFoundError(f'Port {self.port} does not exist!')
            
            # Проверяем права доступа
            if not os.access(self.port, os.R_OK | os.W_OK):
                raise PermissionError(f'No read/write access to {self.port}!')
            
            self.lidar = RPLidar(self.port, self.baudrate)
            self.get_logger().info(f'✅ LiDAR connected: {self.port}')
            
            # Запускаем поток сканирования
            self.scan_thread = threading.Thread(target=self._scan_runner, daemon=True)
            self.scan_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'❌ LiDAR initialization failed: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            self.running = False

    def _scan_runner(self):
        """Запуск асинхронного цикла сканирования"""
        try:
            asyncio.run(self._c1_scan_loop())
        except Exception as e:
            self.get_logger().error(f'Scan runner error: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')

    async def _c1_scan_loop(self):
        """Асинхронный цикл сканирования (как в LIdar.py)"""
        publish_period = 1.0 / max(self.publish_hz, 1e-3)
        last_publish = time.time()
        
        try:
            # Запускаем сканирование
            self.get_logger().info('🎬 Starting scan...')
            _scan_task = asyncio.create_task(self.lidar.simple_scan(make_return_dict=True))
            
            frame_angles = []
            frame_dists = []
            
            while self.running and rclpy.ok():
                # 1) Если отстаём — выбрасываем старые данные
                try:
                    qsize = self.lidar.output_queue.qsize()
                except:
                    qsize = 0
                
                if qsize and qsize > self.queue_drop_threshold:
                    drop_n = qsize - self.max_points
                    dropped = 0
                    while dropped < drop_n:
                        try:
                            self.lidar.output_queue.get_nowait()
                            dropped += 1
                        except (pyqueue.Empty, Exception):
                            break
                    self.get_logger().debug(f'Dropped {dropped} old packets')
                
                # 2) Дренируем очередь пачкой
                drained = 0
                while drained < self.drain_max:
                    try:
                        data = self.lidar.output_queue.get_nowait()
                    except (pyqueue.Empty, Exception):
                        break
                    
                    drained += 1
                    a_deg = data.get("a_deg", None)
                    d_mm = data.get("d_mm", None)
                    q = data.get("q", 0)
                    
                    if a_deg is None or d_mm is None:
                        continue
                    
                    try:
                        q = int(q) if q is not None else 0
                        a_deg = float(a_deg)
                        d = float(d_mm)
                    except (TypeError, ValueError):
                        continue
                    
                    if q >= self.min_quality and 0 < d <= self.max_distance_mm:
                        frame_angles.append(math.radians(a_deg))
                        frame_dists.append(d)
                
                # 3) Публикуем "кадр" с фиксированной частотой
                now = time.time()
                if now - last_publish >= publish_period:
                    if frame_angles:
                        # Ограничим количество точек (downsample)
                        if len(frame_angles) > self.max_points:
                            step = max(1, len(frame_angles) // self.max_points)
                            a = np.array(frame_angles[::step], dtype=np.float32)
                            d = np.array(frame_dists[::step], dtype=np.float32)
                        else:
                            a = np.array(frame_angles, dtype=np.float32)
                            d = np.array(frame_dists, dtype=np.float32)
                        
                        # Сохраняем в shared state
                        with self.scan_lock:
                            self.latest_scan = {
                                'angles_rad': a,
                                'dists_mm': d,
                                'timestamp': now,
                                'n_points': int(a.size)
                            }
                        
                        frame_angles.clear()
                        frame_dists.clear()
                        last_publish = now
                
                await asyncio.sleep(0.001)
                
        except Exception as e:
            self.get_logger().error(f'Scan loop error: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
        finally:
            try:
                self.lidar.reset()
                self.get_logger().info('LiDAR reset')
            except:
                pass

    def publish_callback(self):
        """Публикация данных в топик с заданной частотой"""
        with self.scan_lock:
            if self.latest_scan is None:
                return
            
            angles = self.latest_scan['angles_rad']
            dists_mm = self.latest_scan['dists_mm']
            timestamp = self.latest_scan['timestamp']
            n_points = self.latest_scan['n_points']
        
        if len(angles) == 0 or len(dists_mm) == 0:
            return
        
        # Создаём сообщение LaserScan
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        
        # Угловые параметры
        msg.angle_min = float(np.min(angles))
        msg.angle_max = float(np.max(angles))
        msg.angle_increment = (msg.angle_max - msg.angle_min) / max(len(angles) - 1, 1)
        
        # Временные параметры
        msg.time_increment = 0.0001
        msg.scan_time = 1.0 / self.publish_hz
        msg.range_min = 0.01  # 10 мм
        msg.range_max = self.max_distance_mm / 1000.0  # Конвертируем в метры
        
        # Создаём массив ranges фиксированного размера
        num_ranges = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = [float('inf')] * num_ranges
        
        # Заполняем реальными данными (конвертируем мм → м)
        for i, (angle, dist_mm) in enumerate(zip(angles, dists_mm)):
            idx = int((angle - msg.angle_min) / msg.angle_increment)
            if 0 <= idx < num_ranges:
                msg.ranges[idx] = dist_mm / 1000.0  # мм → м
        
        msg.intensities = [0.0] * len(msg.ranges)
        
        # Публикуем
        self.lidar_pub.publish(msg)
        self.publish_count += 1
        
        # Логирование каждые 10 публикаций
        if self.publish_count % 10 == 0:
            lag_ms = (time.time() - timestamp) * 1000.0
            self.get_logger().info(f'Published {self.publish_count} scans | Points: {n_points} | Lag: {lag_ms:.0f}ms')

    def destroy_node(self):
        """Корректное завершение работы"""
        self.get_logger().info('Stopping LiDAR node...')
        self.running = False
        
        try:
            self.lidar.reset()
            self.get_logger().info('LiDAR stopped')
        except:
            pass
        
        self.get_logger().info(f'Total scans published: {self.publish_count}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RPLidarNode()
    
    if not node.running:
        node.destroy_node()
        rclpy.shutdown()
        return
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user (Ctrl+C)')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()