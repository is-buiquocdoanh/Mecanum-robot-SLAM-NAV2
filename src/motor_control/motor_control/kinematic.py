#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Import các thư viện cần thiết
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from geometry_msgs.msg import Twist           # Thư viện ROS để nhận lệnh vận tốc
from std_msgs.msg import Int16  # Để gửi dữ liệu dạng mảng số nguyên
from math import pi as PI                           # Sử dụng số pi cho tính toán góc và vận tốc
from message_pkg.msg import *
from message_pkg.msg import Velquery
import numpy as np
import time
import sys

class ControlMotorByKinematic(LifecycleNode):
    def __init__(self):
        # Khởi tạo node ROS tên 'control_motor' 
        super().__init__('kinematic')
        self.get_logger().info("ROS 2 Node Initialized!")
        self.killnode = 0

        self.declare_parameters(

            namespace='',
            parameters=[
                ('rate', 100),
                ('enb_debug', 0),
                ('r_banh', 0.035),
                ('lx', 0.085),
                ('ly', 0.11),
                ('pwm_max', 255),
                ('pwm_min', 0),
                ('rpm_max', 60)
            ]
        )
        
        self.rate = self.get_parameter('rate').value
        self.enb_debug = self.get_parameter('enb_debug').value
        
        # Get parameters
        self.r_banh = self.get_parameter('r_banh').get_parameter_value().double_value
        self.lx = self.get_parameter('lx').get_parameter_value().double_value
        self.ly = self.get_parameter('ly').get_parameter_value().double_value
        self.pwm_max = self.get_parameter('pwm_max').get_parameter_value().integer_value
        self.pwm_min = self.get_parameter('pwm_min').get_parameter_value().integer_value
        self.rpm_max= self.get_parameter('rpm_max').get_parameter_value().integer_value

        # Đăng ký callback để nhận lệnh vận tốc từ topic /cmd_vel
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmdVel_callback,10)
        self.data_cmdVel = Twist() 
        self.is_cmdVel = 0
        self.time_cmdVel_recv = time.time()

        self.pub_vel_query = self.create_publisher(Velquery, '/vel_query', 10)
        self.data_velquery = Velquery()
        
        self.timer = self.create_timer(1/self.rate, self.run)

    def on_shutdown(self, state):
        self.killnode = 1
        self.get_logger().warn("Shutting down! Exiting program...")
        return TransitionCallbackReturn.SUCCESS

    # Hàm callback khi có dữ liệu mới từ topic /cmd_vel
    def cmdVel_callback(self, data):
        self.data_cmdVel = data
        self.is_cmdVel = 1
        self.time_cmdVel_recv = time.time()

    def mecanum_kinematics_rpm(self, vel):
        """
        Vx, Vy: m/s (robot velocity)
        omega: rad/s (robot angular velocity)
        L, W: chiều dài / rộng từ tâm robot đến bánh
        r: bán kính bánh xe (m)
        Trả về: [rpm_FL, rpm_FR, rpm_RL, rpm_RR]
        """
        k = self.lx + self.ly
        M = np.array([
            [1, -1, -k],
            [1,  1,  k],
            [1,  1, -k],
            [1, -1,  k]
        ])
        V = np.array([vel.linear.x, vel.linear.y, vel.angular.z])
        
        # Tính tốc độ góc (rad/s)
        w_rad = (1 / self.r_banh) * M @ V
        
        # Chuyển sang vòng/phút
        w_rpm = w_rad * 60 / (2 * np.pi)

        # Giới hạn giá trị trong khoảng [-max_rpm, max_rpm]
        w_rpm_clipped = np.round(np.clip(w_rpm, -self.rpm_max, self.rpm_max))
        
        return w_rpm_clipped

    # Hàm điều khiển chính
    def run(self):
        if self.is_cmdVel == 1:
            # Tính toán giá trị PWM cho bánh trái và bánh phải
            wheel_rpm = self.mecanum_kinematics_rpm(self.data_cmdVel)

            if time.time() - self.time_cmdVel_recv < 1:
                print("Tốc độ từng bánh (RPM):")
                print(f"FL: {wheel_rpm[0]}")
                print(f"FR: {wheel_rpm[1]}")
                print(f"RL: {wheel_rpm[2]}")
                print(f"RR: {wheel_rpm[3]}")
                print("--------------------------")

                if wheel_rpm[0] > 0:
                    self.data_velquery.byte0 = 1
                    self.data_velquery.byte1 = int(wheel_rpm[0])
                elif wheel_rpm[0] < 0:
                    self.data_velquery.byte0 = 2
                    self.data_velquery.byte1 = int(-wheel_rpm[0])
                else:
                    self.data_velquery.byte0 = 0
                    self.data_velquery.byte1 = 0                    

                if wheel_rpm[1] > 0:
                    self.data_velquery.byte2 = 1
                    self.data_velquery.byte3 = int(wheel_rpm[1])
                elif wheel_rpm[1] < 0:
                    self.data_velquery.byte2 = 2
                    self.data_velquery.byte3 = int(-wheel_rpm[1])
                else:
                    self.data_velquery.byte2 = 0
                    self.data_velquery.byte3 = 0 

                if wheel_rpm[2] > 0:
                    self.data_velquery.byte4 = 1
                    self.data_velquery.byte5 = int(wheel_rpm[2])
                elif wheel_rpm[2] < 0:
                    self.data_velquery.byte4 = 2
                    self.data_velquery.byte5 = int(-wheel_rpm[2])
                else:
                    self.data_velquery.byte4 = 0
                    self.data_velquery.byte5 = 0 

                if wheel_rpm[3] > 0:
                    self.data_velquery.byte6 = 1
                    self.data_velquery.byte7 = int(wheel_rpm[3])
                elif wheel_rpm[3] < 0:
                    self.data_velquery.byte6 = 2
                    self.data_velquery.byte7 = int(-wheel_rpm[3])
                else:
                    self.data_velquery.byte6 = 0
                    self.data_velquery.byte7 = 0

            else:
                print("Tốc độ từng bánh (RPM):")
                print(f"FL: 0.0")
                print(f"FR: 0.0")
                print(f"RL: 0.0")
                print(f"RR: 0.0")
                print("--------------------------")

                self.data_velquery = Velquery()

            self.pub_vel_query.publish(self.data_velquery)
        # -- KILL NODE -- 
        if self.killnode:
            sys.exit(0)            
        
            
def main():
    rclpy.init()
    node = ControlMotorByKinematic()
    print("launch kinematic")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()
