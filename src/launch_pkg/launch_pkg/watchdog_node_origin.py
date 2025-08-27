#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
DATE: 10/07/2025
AUTHOR: HOANG VAN QUANG, Archie Phung

Mô tả: Giám sát topic và tự động khởi động lại node nếu không nhận được tin nhắn trong một khoảng thời gian nhất định.

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Hoặc đúng loại message bạn cần
import subprocess
import time
import signal
import os
import psutil
from message_pkg.msg import *

class TopicWatchdog(Node):
    def __init__(self):
        super().__init__('topic_watchdog')

        # Cấu hình giám sát
        self.topic_name = '/can_send'
        self.timeout_seconds = 5.0
        self.last_msg_time = self.get_clock().now()

        # Không khởi động chương trình bị giám sát từ đây
        self.proc = None  # Chỉ gán khi cần restart

        # Tên tiến trình cần giám sát (phải khớp đúng với tên node bạn chạy)
        self.process_name = 'convert_canRos'

        # Subscriber để giám sát topic
        self.create_subscription(Cansend, self.topic_name, self.topic_callback, 10)

        # Timer định kỳ kiểm tra
        self.create_timer(1.0, self.check_timeout)

    def topic_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        # self.get_logger().info('Received message.')

    def check_timeout(self):
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        self.get_logger().info(f'Time elapsed since last message: {elapsed:.1f}s')
        if elapsed > self.timeout_seconds:
            self.get_logger().warn(f'No message from {self.topic_name} for {elapsed:.1f}s. Restarting...')
            self.restart_process()

    def find_and_kill_process(self):
        """Tìm và kill tiến trình theo tên node"""
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if self.process_name in ' '.join(proc.info['cmdline']):
                    self.get_logger().warn(f"Killing process PID {proc.pid} ({proc.info['name']})...")
                    os.kill(proc.pid, signal.SIGTERM)
                    time.sleep(2)
                    return True
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        self.get_logger().warn(f"No process found with name: {self.process_name}")
        return False

    def restart_process(self):
        self.find_and_kill_process()
        # Chạy lại node bị giám sát bằng lệnh ros2 launch
        self.get_logger().info("Launching monitored node...")
        self.proc = subprocess.Popen(['ros2', 'launch', 'ros_canBus', 'convertCAN_ROS.launch.py'])

def main(args=None):
    rclpy.init(args=args)
    node = TopicWatchdog()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.proc:
            node.proc.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
