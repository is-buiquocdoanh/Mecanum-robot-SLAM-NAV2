#!/usr/bin/env python3

import sys
import os
import time
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from message_pkg.msg import StatusPort

class CheckPhysical(LifecycleNode):
    def __init__(self):
        super().__init__('check_physical')
        self.get_logger().info("ROS 2 Node Initialized!")
        self.killnode = 0
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('port_rtc', '/dev/ttyUSB0'),
                ('port_magline', '/dev/ttyUSB0'),
                ('port_rfid', '/dev/ttyUSB0'),
            ]
        )
        
        self.port_rtc = self.get_parameter('port_rtc').value
        self.port_magline = self.get_parameter('port_magline').value
        self.port_rfid = self.get_parameter('port_rfid').value
        
        self.pub_status_port = self.create_publisher(StatusPort, '/status_port', 10)

        self.rate = 10
        self.timer_period = 1/self.rate
        self.timer = self.create_timer(self.timer_period, self.run)

        self.status_port = StatusPort()

    def on_shutdown(self, state):
        self.killnode = 1
        self.get_logger().warn("Shutting down! Exiting program...")
        return TransitionCallbackReturn.SUCCESS
    
    def ethernet_check(self, address):
        try:
            output = subprocess.check_output(f"ping -c 1 -w 1 {address}", shell=True)
            return 'time=' in str(output)
        except subprocess.CalledProcessError as e:
            return False
    
    def usb_serial_check(self, nameport):
        try:
            output = subprocess.check_output(f"ls /dev/ | grep {nameport}", shell=True)
            return nameport in str(output)
        except subprocess.CalledProcessError:
            return False
    
    def run(self):
        self.status_port.rtc = self.usb_serial_check(self.port_rtc)
        self.status_port.magline = self.usb_serial_check(self.port_magline)
        self.status_port.rfid = self.usb_serial_check(self.port_rfid)
        
        self.pub_status_port.publish(self.status_port)

        # -- KILL NODE -- 
        if self.killnode:
            sys.exit(0)
        
def main():
    rclpy.init()
    node = CheckPhysical()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('Program stopped')

if __name__ == '__main__':
    main()
