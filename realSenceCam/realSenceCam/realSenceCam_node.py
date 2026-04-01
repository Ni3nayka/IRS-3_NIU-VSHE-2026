#!/usr/bin/env python3
'''
sudo apt update
sudo apt install -y ros-jazzy-realsense2-camera python3-opencv
pip3 install numpy
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
from datetime import datetime

class RGBRecorder(Node):
    def __init__(self):
        super().__init__('rgb_recorder')
        
        # === НАСТРОЙКИ ===
        self.base_dir = '/home/pi/recordings'
        self.recording_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.save_dir = os.path.join(self.base_dir, self.recording_name)
        
        # Создаем директорию
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Путь к видеофайлу
        self.video_path = os.path.join(self.save_dir, 'video.mp4')
        
        # Параметры видео
        self.fps = 30.0
        self.frame_size = (640, 480)  # Будет обновлено при получении первого кадра
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        
        # Видеописатель (инициализируется после первого кадра)
        self.video_writer = None
        
        # Счётчик кадров
        self.frame_count = 0
        self.start_time = time.time()
        
        # Мост для конвертации ROS Image -> OpenCV
        self.bridge = CvBridge()
        
        # === ИЗМЕНЁННЫЙ ТОПИК ===
        # Подписка на кастомный топик камеры
        self.rgb_sub = self.create_subscription(
            Image,
            'realSenceCam',  # Топик изменён по запросу
            self.rgb_callback,
            10
        )
        
        self.get_logger().info(f'RGB Recorder started. Listening on topic: realSenceCam')
        self.get_logger().info(f'Saving video to: {self.video_path}')

    def rgb_callback(self, msg):
        """Обработка каждого кадра с RGB камеры"""
        try:
            # Конвертируем ROS сообщение в изображение OpenCV
            rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Инициализируем VideoWriter при получении первого кадра
            if self.video_writer is None:
                height, width = rgb_image.shape[:2]
                self.frame_size = (width, height)
                self.video_writer = cv2.VideoWriter(
                    self.video_path,
                    self.fourcc,
                    self.fps,
                    self.frame_size
                )
                self.get_logger().info(f'Video initialized: {width}x{height} @ {self.fps} FPS')
            
            # Записываем кадр в видеофайл
            self.video_writer.write(rgb_image)
            self.frame_count += 1
            
            # Логирование прогресса каждые 30 кадров
            if self.frame_count % 30 == 0:
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed if elapsed > 0 else 0
                self.get_logger().info(f'Recorded {self.frame_count} frames ({fps:.1f} FPS)')
                
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')

    def destroy_node(self):
        """Корректное завершение записи при остановке ноды"""
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f'Video saved to: {self.video_path}')
        
        elapsed = time.time() - self.start_time
        self.get_logger().info(f'Recording finished. Total frames: {self.frame_count}, Duration: {elapsed:.1f}s')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RGBRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()