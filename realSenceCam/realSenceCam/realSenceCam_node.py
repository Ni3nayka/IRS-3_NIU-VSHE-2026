#!/usr/bin/env python3
'''
sudo apt update
sudo apt install -y python3-opencv
pip3 install numpy pyrealsense2
'''

import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import cv2
import os
import sys
import tty
import termios
import threading
import time
from datetime import datetime

class RGBRecorderDirect(Node):
    def __init__(self):
        super().__init__('rgb_recorder_direct')
        
        # === НАСТРОЙКИ ===
        self.base_dir = os.path.join(os.path.expanduser('~'), 'recordings')
        self.recording_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.save_dir = os.path.join(self.base_dir, self.recording_name)
        
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.video_path = os.path.join(self.save_dir, 'video.mp4')
        self.log_path = os.path.join(self.save_dir, 'recording.log')
        
        # Параметры видео
        self.fps = 30.0
        self.frame_size = (640, 480)
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        
        # Состояние
        self.video_writer = None
        self.frame_count = 0
        self.start_time = 0.0
        self.is_recording = False
        self.last_log_frame = 0
        self.last_log_time = 0.0
        self.running = True
        
        # Камера
        self.pipeline = None
        self.camera_name = "Unknown"
        
        # Терминал
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        
        # === ИНИЦИАЛИЗАЦИЯ КАМЕРЫ С ПОВТОРНЫМИ ПОПЫТКАМИ ===
        if not self._init_camera_with_retry():
            self.get_logger().error('❌ Failed to initialize camera after retries. Exiting.')
            self.running = False
            return
        
        self._init_log_file()
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('RGB RECORDER - DIRECT REALSENSE')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Camera: {self.camera_name}')
        self.get_logger().info('T : Start/Stop recording')
        self.get_logger().info('Q : Quit')
        self.get_logger().info('=' * 50)

    def _init_camera_with_retry(self, max_attempts=10, delay=1.0):
        """Попытка инициализации камеры с повторами"""
        for attempt in range(1, max_attempts + 1):
            try:
                self.ctx = rs.context()
                devices = self.ctx.query_devices()
                
                # ✅ ПРАВИЛЬНАЯ ПРОВЕРКА: итерируемся по device_list
                device_count = 0
                for dev in devices:
                    device_count += 1
                    self.get_logger().info(f'📷 Found device: {dev.get_info(rs.camera_info.name)}')
                
                if device_count == 0:
                    self.get_logger().warn(f'Attempt {attempt}/{max_attempts}: No devices found. Waiting...')
                    time.sleep(delay)
                    continue
                
                # Берём первое устройство
                self.device = devices[0]
                self.camera_name = self.device.get_info(rs.camera_info.name)
                
                self.pipeline = rs.pipeline(ctx=self.ctx)
                self.config = rs.config()
                self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                
                self.profile = self.pipeline.start(self.config)
                
                self.get_logger().info(f'✅ Camera initialized: {self.camera_name}')
                return True
                
            except Exception as e:
                self.get_logger().warn(f'Attempt {attempt}/{max_attempts}: Error - {str(e)}')
                time.sleep(delay)
        
        return False

    def _init_log_file(self):
        try:
            with open(self.log_path, 'w') as f:
                f.write("=" * 60 + "\n")
                f.write("RGB CAMERA RECORDING LOG\n")
                f.write("=" * 60 + "\n\n")
                f.write(f"Start Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Node Name: {self.get_name()}\n")
                f.write(f"Camera: {self.camera_name}\n")
                f.write(f"Target FPS: {self.fps}\n")
                f.write(f"Save Directory: {self.save_dir}\n")
                f.write(f"Video Path: {self.video_path}\n")
                f.write(f"Recording Status: WAITING\n")
                f.write("\n" + "-" * 60 + "\n")
                f.write("RECORDING PROGRESS\n")
                f.write("-" * 60 + "\n")
                f.write(f"{'Timestamp':<20} | {'Frames':<10} | {'FPS':<10} | {'Status'}\n")
                f.write("-" * 60 + "\n")
        except Exception as e:
            self.get_logger().error(f'Failed to initialize log file: {str(e)}')

    def _write_log_entry(self, frames, fps, status):
        try:
            with open(self.log_path, 'a') as f:
                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                f.write(f"{timestamp:<20} | {frames:<10} | {fps:<10.1f} | {status}\n")
        except Exception as e:
            self.get_logger().error(f'Failed to write log entry: {str(e)}')

    def _finalize_log(self):
        try:
            elapsed = time.time() - self.start_time if self.start_time > 0 else 0
            actual_fps = self.frame_count / elapsed if elapsed > 0 else 0
            
            with open(self.log_path, 'a') as f:
                f.write("-" * 60 + "\n")
                f.write("\nRECORDING COMPLETE\n")
                f.write("=" * 60 + "\n\n")
                f.write(f"End Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Total Duration: {elapsed:.2f} seconds\n")
                f.write(f"Total Frames: {self.frame_count}\n")
                f.write(f"Average FPS: {actual_fps:.2f}\n")
                f.write(f"Target FPS: {self.fps}\n")
                f.write(f"Frame Size: {self.frame_size[0]}x{self.frame_size[1]}\n")
                f.write(f"Video File: {self.video_path}\n")
                f.write(f"Log File: {self.log_path}\n")
                f.write("\n" + "=" * 60 + "\n")
                f.write("END OF LOG\n")
                f.write("=" * 60 + "\n")
        except Exception as e:
            self.get_logger().error(f'Failed to finalize log file: {str(e)}')

    def _update_log_status(self, status):
        try:
            with open(self.log_path, 'r+') as f:
                lines = f.readlines()
                for i, line in enumerate(lines):
                    if line.startswith('Recording Status:'):
                        lines[i] = f"Recording Status: {status}\n"
                        break
                f.seek(0)
                f.writelines(lines)
                f.truncate()
        except Exception as e:
            self.get_logger().error(f'Failed to update log status: {str(e)}')

    def capture_and_record(self):
        try:
            while self.running and rclpy.ok():
                try:
                    frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                    color_frame = frames.get_color_frame()
                    
                    if not color_frame:
                        continue
                    
                    # Конвертация в numpy
                    import numpy as np
                    color_image = np.asanyarray(color_frame.get_data())
                    
                    if self.is_recording:
                        self._record_frame(color_image)
                    
                except Exception as e:
                    if self.running:
                        self.get_logger().error(f'Frame capture error: {str(e)}')
                    time.sleep(0.1)
                    
        except Exception as e:
            self.get_logger().error(f'Capture loop error: {str(e)}')

    def _record_frame(self, frame):
        try:
            if self.video_writer is None:
                height, width = frame.shape[:2]
                self.frame_size = (width, height)
                self.video_writer = cv2.VideoWriter(
                    self.video_path,
                    self.fourcc,
                    self.fps,
                    self.frame_size
                )
                self.start_time = time.time()
                self.last_log_time = time.time()
                self.get_logger().info(f'✗ RECORDING STARTED: {width}x{height} @ {self.fps} FPS')
                self._write_log_entry(1, 0.0, 'FIRST FRAME')
            
            self.video_writer.write(frame)
            self.frame_count += 1
            
            if self.frame_count % 30 == 0:
                current_time = time.time()
                elapsed = current_time - self.start_time
                
                delta_frames = self.frame_count - self.last_log_frame
                delta_time = current_time - self.last_log_time
                current_fps = delta_frames / delta_time if delta_time > 0 else 0
                
                self.last_log_frame = self.frame_count
                self.last_log_time = current_time
                
                avg_fps = self.frame_count / elapsed if elapsed > 0 else 0
                
                self.get_logger().info(f'✗ Recorded {self.frame_count} frames ({avg_fps:.1f} avg FPS)')
                self._write_log_entry(self.frame_count, current_fps, 'RECORDING')
                
        except Exception as e:
            self.get_logger().error(f'Record frame error: {str(e)}')

    def toggle_recording(self):
        self.is_recording = not self.is_recording
        
        if self.is_recording:
            self.frame_count = 0
            self.video_writer = None
            self._update_log_status('RECORDING')
            self.get_logger().info('=' * 50)
            self.get_logger().info('✗✗✗ RECORDING STARTED ✗✗✗')
            self.get_logger().info('=' * 50)
        else:
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
            
            self._update_log_status('STOPPED')
            self._finalize_log()
            
            elapsed = time.time() - self.start_time if self.start_time > 0 else 0
            self.get_logger().info('=' * 50)
            self.get_logger().info('✓✓✓ RECORDING STOPPED ✓✓✓')
            self.get_logger().info(f'Total frames: {self.frame_count}')
            self.get_logger().info(f'Duration: {elapsed:.1f}s')
            self.get_logger().info(f'Saved to: {self.video_path}')
            self.get_logger().info('=' * 50)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        return ch

    def input_loop(self):
        try:
            while self.running and rclpy.ok():
                try:
                    key = self.get_key()
                    if key:
                        key = key.lower()
                        if key == 't':
                            self.toggle_recording()
                        elif key == 'q':
                            self.get_logger().info('Quit requested')
                            self.running = False
                            rclpy.shutdown()
                        else:
                            self.get_logger().info(f'Key pressed: {key} (T=record, Q=quit)')
                except Exception as e:
                    break
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def destroy_node(self):
        self.get_logger().info('Stopping recorder...')
        
        if self.is_recording and self.video_writer is not None:
            self.video_writer.release()
            self._finalize_log()
            self.get_logger().info(f'Video saved to: {self.video_path}')
        
        if self.pipeline:
            try:
                self.pipeline.stop()
                self.get_logger().info('Camera pipeline stopped')
            except:
                pass
        
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        except:
            pass
        
        elapsed = time.time() - self.start_time if self.start_time > 0 else 0
        self.get_logger().info(f'Node stopped. Total frames: {self.frame_count}, Duration: {elapsed:.1f}s')
        self.get_logger().info(f'All data saved to: {self.save_dir}')
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RGBRecorderDirect()
    
    if not node.running:
        node.destroy_node()
        rclpy.shutdown()
        return
    
    capture_thread = threading.Thread(target=node.capture_and_record)
    capture_thread.start()
    
    input_thread = threading.Thread(target=node.input_loop)
    input_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user (Ctrl+C)')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        node.running = False
        capture_thread.join()
        input_thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()