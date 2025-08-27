#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
DATE: 10/07/2025
AUTHOR: HOANG VAN QUANG, Archie Phung

Mô tả: Giám sát topic và tự động khởi động lại node nếu không nhận được tin nhắn trong một khoảng thời gian nhất định.

"""

import rclpy
from rclpy.node import Node
import subprocess
import time
import signal
import os
import psutil
from message_pkg.msg import *

class MultiNodeWatchdog(Node):
	def __init__(self):
		super().__init__('multi_node_watchdog')

		# Danh sách các node cần theo dõi
		self.monitored_nodes = {
			# 'convert_canRos': {
			# 	'topics': ['/can_send'],
			# 	'msg_type': Cansend,
			# 	'timeout': 5.0,
			# 	'last_received': {},
			# 	'launch_cmd': ['ros2', 'launch', 'ros_canBus', 'convertCAN_ROS.launch.py']
			# },
			'ros_serial_bridge': {
				'topics': ['/can_received'],
				'msg_type': Canreceived,
				'timeout': 5.0,
				'last_received': {},
				'launch_cmd': ['ros2', 'launch', 'ros_canBus', 'RTC_board_vs2.launch.py'],
				'count': 0
			},
			'sti_rfid': {
				'topics': ['/rfid_respond'],
				'msg_type': RFID,
				'timeout': 5.0,
				'last_received': {},
				'launch_cmd': ['ros2', 'launch', 'sti_module', 'stiRFID.launch.py'],
				'count': 0
			},
			'sti_magnetic': {
				'topics': ['/magneticLine_front', '/magneticLine_behind'],
				'msg_type': MagneticLine,
				'timeout': 5.0,
				'last_received': {},
				'launch_cmd': ['ros2', 'launch', 'sti_module', 'stiMagline.launch.py'],
				'count': 0
			}
		}

		# Đăng ký subscriber cho mỗi topic
		for node_name, config in self.monitored_nodes.items():
			msg = config['msg_type']
			for topic in config['topics']:
				self.create_subscription(msg, topic, self.generate_callback(node_name, topic), 10)
				config['last_received'][topic] = self.get_clock().now()

		self.status_pub = self.create_publisher(StatusReconnect, '/watchdog_status', 10)
		self.component_status = {
			'rfid': True,
			'driver_all': True,
			'rtc': True,
			'magline': True
		}
		self.component_count = {
			'rfid': 0,
			'driver_all': 0,
			'rtc': 0,
			'magline': 0
		}
		# Kiểm tra mỗi 1 giây
		self.create_timer(5.0, self.check_all_nodes)

	def publish_status(self):
		msg = StatusReconnect()
		msg.rfid.sts = self.component_status['rfid']
		msg.rfid.times = self.component_count['rfid']

		msg.driver_all.sts = self.component_status['driver_all']
		msg.driver_all.times = self.component_count['driver_all']

		msg.rtc.sts = self.component_status['rtc']
		msg.rtc.times = self.component_count['rtc']

		msg.magline.sts = self.component_status['magline']
		msg.magline.times = self.component_count['magline']
		
		self.status_pub.publish(msg)

	def generate_callback(self, node_name, topic):
		def callback(msg):
			self.monitored_nodes[node_name]['last_received'][topic] = self.get_clock().now()
			# self.get_logger().info(f"{node_name} - {topic}: msg received")
		return callback

	def check_all_nodes(self):
		for node_name, config in self.monitored_nodes.items():
			node_ok = True
			for topic in config['topics']:
				elapsed = (self.get_clock().now() - config['last_received'][topic]).nanoseconds / 1e9
				self.get_logger().info(f'Time elapsed of {node_name} - {topic} since last message: {elapsed:.1f}s')
				if elapsed > config['timeout']:
					self.get_logger().warn(f"[{node_name}] Topic {config['topics']} has not published for {elapsed:.1f}s. Restarting node...")
					self.restart_node(node_name, config['launch_cmd'])
					# Reset last_received để tránh lặp lại
					config['last_received'][topic] = self.get_clock().now()
					config['count'] += 1
					node_ok = False
					
			if node_name == 'sti_rfid':
				self.component_status['rfid'] = node_ok
				self.component_count['rfid'] = config['count']

			elif node_name == 'ros_serial_bridge':
				self.component_status['rtc'] = node_ok
				self.component_count['rtc'] = config['count']

			elif node_name == 'sti_magnetic':
				self.component_status['magline'] = node_ok
				self.component_count['magline'] = config['count']

			# elif node_name == 'driver_all':
			# 	self.component_status['driver_all'] = node_ok
			# 	self.component_count['driver_all'] = config['count']

			self.publish_status()			

		self.get_logger().info("------------------------------------------------------------")

	def restart_node(self, process_name, launch_cmd):
		self.kill_process_by_name(process_name)
		time.sleep(2)
		subprocess.Popen(launch_cmd)
		self.get_logger().info(f"Launched node {process_name}.")

	def kill_process_by_name(self, name):
		found = False
		for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
			try:
				if name in ' '.join(proc.info['cmdline']):
					self.get_logger().info(f"Killing {name} (PID {proc.pid})...")
					os.kill(proc.pid, signal.SIGTERM)
					found = True
			except (psutil.NoSuchProcess, psutil.AccessDenied):
				continue
		if not found:
			self.get_logger().warn(f"Could not find process named {name}.")

def main(args=None):
    rclpy.init(args=args)
    node = MultiNodeWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
