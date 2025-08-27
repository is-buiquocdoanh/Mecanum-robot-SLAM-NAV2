#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Import các thư viện cần thiết
import sys
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float64     # Để gửi dữ liệu dạng mảng số nguyên
import time

from math import pi as PI                           # Sử dụng số pi cho tính toán góc và vận tốc
from math import fabs
import RPi.GPIO as GPIO

class IO_PIN():
    def __init__(self):
        # encoder use
        self.ENC_IN_LEFT_A = 25
        self.ENC_IN_LEFT_B = 26

        self.ENC_IN_RIGHT_A = 23
        self.ENC_IN_RIGHT_B = 24

        # left_wheel use
        self.RPWM_LEFT = 20
        self.LPWM_LEFT = 21

        self.RPWM_RIGHT = 5
        self.LPWM_RIGHT = 6
    
class MAIN_CORE(LifecycleNode):
    def __init__(self):
        # Khởi tạo node ROS tên 'control_motor' 

        super().__init__('motor_Control')
        self.get_logger().info("ROS 2 Node Initialized!")
        self.killnode = 0

        self.declare_parameters(

            namespace='',
            parameters=[
                ('rate', 30),
                ('enb_debug', 0),
                ('r_banh', 0.034),
                ('kc_hai_banh', 0.4),
                ('v_max_dong_co', 10000),
                ('pwm_max', 255),
                ('hop_giam_toc', 78)
            ]
        )
        self.rate = self.get_parameter('rate').value
        self.enb_debug = self.get_parameter('enb_debug').value

        # Các tham số vật lý của hệ thống
        self.r_banh = self.get_parameter('r_banh').value
        self.kc_hai_banh = self.get_parameter('kc_hai_banh').value
        self.v_max_dong_co = self.get_parameter('v_max_dong_co').value
        self.pwm_max = self.get_parameter('pwm_max').value
        self.hop_giam_toc = self.get_parameter('hop_giam_toc').value
        self.pwm_min = 40       # PWM tối thiểu

        # Đăng ký callback để nhận lệnh từ left_wheel_query && right_wheel_query
        self.create_subscription(Int16, "/left_back_query", self.callback_leftwheelquery, 1)
        self.data_leftwheel_query = Int16()
        self.ctime_recv_leftwheel_query = 0
        self.is_recv_leftwheel_query = 0 

        self.create_subscription(Int16, "/right_back_query", self.callback_rightwheelquery, 1)
        self.data_rightwheel_query = Int16() 
        self.is_recv_rightwheel_query = 0
        self.ctime_recv_rightwheel_query = 0

        # Publish data
        self.pub_leftTicks = self.create_publisher(Int16, "/left_back_ticks", 10)
        self.left_wheel_tick_count = Int16()

        self.pub_rightTicks = self.create_publisher(Int16, "/right_back_ticks", 10)
        self.right_wheel_tick_count = Int16()

        self.pub_leftwheel_vel = self.create_publisher(Float64, "/left_back_vel", 10)
        self.left_wheel_vel = Float64()

        self.pub_rightwheel_vel = self.create_publisher(Float64, "/right_back_vel", 10)
        self.right_wheel_vel = Float64()
        # -- 

        self.pub_coreData = self.create_publisher(Twist, "/core_back_info", 10)
        self.core_data = Twist()

        # Variable
        # True = Forward; False = Reverse
        self.Direction_left = True
        self.Direction_right = True
        
        # Minumum and maximum values for 16-bit integers
        #Range of 65,535
        self.encoder_minimum = -32768
        self.encoder_maximum = 32767

        # Time interval for measurements in milliseconds
        self.interval = 0.03
        self.previousMillis = 0
        self.currentMillis = 0

        # How much the PWM value can change each cycle
        self.PWM_INCREMENT = 1
        
        # Number of ticks per wheel revolution
        self.TICKS_PER_REVOLUTION = 440
        
        # Wheel radius in meters
        self.WHEEL_RADIUS = 0.0325
        
        # Distance from center of the left tire to the center of the right tire in m
        self.WHEEL_BASE = 0.40
        
        # Number of ticks a wheel makes moving a linear distance of 1 meter
        self.TICKS_PER_METER = 2155
        
        # Proportional constant, which was measured by measuring the 
        # PWM-Linear Velocity relationship for the robot.
        self.K_P = 278
        
        # Y-intercept for the PWM-Linear Velocity relationship for the robot
        self.b = 52
        
        # Correction multiplier for drift. Chosen through experimentation.
        self.DRIFT_MULTIPLIER = 120
        
        # Turning PWM output (0 = min, 255 = max for PWM values)
        self.PWM_TURN = 80
        
        # Set maximum and minimum limits for the PWM values
        self.PWM_MIN = 0
        self.PWM_MAX = 100
        
        # Set linear velocity and PWM variable values for each wheel
        self.velLeftWheel = 0
        self.velRightWheel = 0
        self.pwmLeftReq = 0
        self.pwmRightReq = 0

        self.vel_left = 0
        self.vel_right = 0
        # Record the time that the last velocity command was received
        self.lastCmdVelReceived = 0

        # direction of robot 
        self.leftwheel_dir = 0
        self.rightwheel_dir = 0

        self.is_recv_left_wheel = 0
        self.is_recv_right_wheel = 0
        self.robot_dir = 0
        self.prevT = 0
        self.eintegral_r = 0
        self.eintegral_l = 0
        self.pwmout1 = 0
        self.pwmout2 = 0
        self.pre_robot_dir = 0

        # Previous timestamp
        self.prevLeftTime = time.time()
        self.prevRightTime = time.time()
        # Variable gets created and initialized the first time a function is called.
        self.prevLeftCount = 0
        self.prevRightCount = 0
        self.numOfLeftTicks = 0
        self.numOfRightTicks = 0

        # These variables will hold our desired PWM values
        self.pwmLeftOut = 0
        self.pwmRightOut = 0
        # setup pin for PI
        # Cài đặt mode của GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self.main_pin = IO_PIN()
        # Cấu hình các chân GPIO
        GPIO.setup(self.main_pin.RPWM_LEFT, GPIO.OUT)
        GPIO.setup(self.main_pin.LPWM_LEFT, GPIO.OUT)
        # GPIO.setup(self.main_pin.REN_LEFT, GPIO.OUT)
        # GPIO.setup(self.main_pin.LEN_LEFT, GPIO.OUT)

        GPIO.setup(self.main_pin.RPWM_RIGHT, GPIO.OUT)
        GPIO.setup(self.main_pin.LPWM_RIGHT, GPIO.OUT)
        # GPIO.setup(self.main_pin.REN_RIGHT, GPIO.OUT)
        # GPIO.setup(self.main_pin.LEN_RIGHT, GPIO.OUT)

        GPIO.setup(self.main_pin.ENC_IN_LEFT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.main_pin.ENC_IN_LEFT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.main_pin.ENC_IN_RIGHT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.main_pin.ENC_IN_RIGHT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Kích hoạt điều khiển động cơ
        # GPIO.output(self.main_pin.REN_LEFT, GPIO.HIGH)
        # GPIO.output(self.main_pin.LEN_LEFT, GPIO.HIGH)
        # GPIO.output(self.main_pin.REN_RIGHT, GPIO.HIGH)
        # GPIO.output(self.main_pin.LEN_RIGHT, GPIO.HIGH)

        # Tạo PWM cho các chân RPWM và LPWM
        freq = 1000  # Tần số PWM (Hz)
        self.pwm_r1 = GPIO.PWM(self.main_pin.RPWM_LEFT, freq)
        self.pwm_l1 = GPIO.PWM(self.main_pin.LPWM_LEFT, freq)
        self.pwm_r2 = GPIO.PWM(self.main_pin.RPWM_RIGHT, freq)
        self.pwm_l2 = GPIO.PWM(self.main_pin.LPWM_RIGHT, freq)

        # Khởi động PWM với chu kỳ ban đầu là 0%
        self.pwm_r1.start(0)
        self.pwm_l1.start(0)
        self.pwm_r2.start(0)
        self.pwm_l2.start(0)

        # Xóa event cũ nếu tồn tại
        GPIO.remove_event_detect(self.main_pin.ENC_IN_LEFT_A)
        GPIO.remove_event_detect(self.main_pin.ENC_IN_RIGHT_A)

        # Thêm phát hiện sự kiện
        GPIO.add_event_detect(self.main_pin.ENC_IN_LEFT_A, GPIO.BOTH, callback=self.left_wheel_tick)
        # GPIO.add_event_detect(self.main_pin.ENC_IN_RIGHT_A, GPIO.BOTH, callback=self.right_wheel_tick)

        self.position = 0
        self.last_a_state_left = GPIO.input(self.main_pin.ENC_IN_LEFT_A)
        self.last_a_state_right = GPIO.input(self.main_pin.ENC_IN_RIGHT_A)
        self.currT = time.time()
        self.prevT = time.time()
        self.eintegral_l = 0
        self.eintegral_r = 0
        self.deltaT = 0

        self.timer = self.create_timer(1.0/self.rate, self.run)

    def cleanup(self):
        """
        Giải phóng tài nguyên GPIO.
        """
        GPIO.cleanup()

    # Increment the number of ticks
    def left_wheel_tick(self, data):
    
        # Read the value for the encoder for the left wheel
        a_state = GPIO.input(self.main_pin.ENC_IN_LEFT_A)
        b_state = GPIO.input(self.main_pin.ENC_IN_LEFT_B)

        # Xác định hướng quay
        if a_state != self.last_a_state_left:  # Có sự thay đổi tín hiệu
            if b_state == a_state:
                # self.position += 1  # Quay theo chiều kim đồng hồ
                if (self.left_wheel_tick_count.data == self.encoder_maximum):
                    self.left_wheel_tick_count.data = self.encoder_minimum
                else:
                    self.left_wheel_tick_count.data = self.left_wheel_tick_count.data + 1
            else:
                if (self.left_wheel_tick_count.data == self.encoder_minimum):
                    self.left_wheel_tick_count.data = self.encoder_maximum
                else:
                    self.left_wheel_tick_count.data = self.left_wheel_tick_count.data - 1

        self.last_a_state_left = a_state

        # print("Gia tri encoder ben trai la", self.left_wheel_tick_count.data)

    def right_wheel_tick(self, data):
    
        # Read the value for the encoder for the right wheel
        a_state = GPIO.input(self.main_pin.ENC_IN_RIGHT_A)
        b_state = GPIO.input(self.main_pin.ENC_IN_RIGHT_B)

        # Xác định hướng quay
        if a_state != self.last_a_state_right:  # Có sự thay đổi tín hiệu
            if b_state != a_state:
                # Quay theo chiều kim đồng hồ
                if (self.right_wheel_tick_count.data == self.encoder_maximum):
                    self.right_wheel_tick_count.data = self.encoder_minimum
                else:
                    self.right_wheel_tick_count.data = self.right_wheel_tick_count.data + 1
            else:
                if (self.right_wheel_tick_count.data == self.encoder_minimum):
                    self.right_wheel_tick_count.data = self.encoder_maximum
                else:
                    self.right_wheel_tick_count.data = self.right_wheel_tick_count.data - 1

        self.last_a_state_right = a_state

        # print("Gia tri encoder ben phai la", self.right_wheel_tick_count.data)

    # Calculate the left wheel linear velocity in RPM 
    def calc_vel_left_wheel(self):
        
        # Manage rollover and rollunder when we get outside the 16-bit integer range 
        self.numOfLeftTicks = (65535 + self.left_wheel_tick_count.data - self.prevLeftCount) % 65535
        
        # If we have had a big jump, it means the tick count has rolled over.
        if (self.numOfLeftTicks > 10000):
            self.numOfLeftTicks = 0 - (65535 - self.numOfLeftTicks)
        
        # Calculate wheel velocity in meters per millisecond
        self.velLeftWheel = self.numOfLeftTicks/self.TICKS_PER_METER/(time.time()-self.prevLeftTime)

        # Calculate left wheel velocity in RPM
        self.left_wheel_vel.data = self.velLeftWheel*30.0/PI/self.WHEEL_RADIUS

        # Keep track of the previous tick count
        self.prevLeftCount = self.left_wheel_tick_count.data
        
        # Update the timestamp
        self.prevLeftTime = time.time()

    # Calculate the right wheel linear velocity in RPM 
    def calc_vel_right_wheel(self):
        
        # Manage rollover and rollunder when we get outside the 16-bit integer range 
        self.numOfRightTicks = (65535 + self.right_wheel_tick_count.data - self.prevRightCount) % 65535
        
        # If we have had a big jump, it means the tick count has rolled over.
        if (self.numOfRightTicks > 10000):
            self.numOfRightTicks = 0 - (65535 - self.numOfRightTicks)
        
        # Calculate wheel velocity in meters per second
        self.velRightWheel = self.numOfRightTicks/self.TICKS_PER_METER/(time.time()-self.prevRightTime)

        # Calculate right wheel velocity in RPM
        self.right_wheel_vel.data = self.velRightWheel*30.0/PI/self.WHEEL_RADIUS

        # Keep track of the previous tick count
        self.prevRightCount = self.right_wheel_tick_count.data
        
        # Update the timestamp
        self.prevRightTime = time.time()

    # Hàm callback khi có dữ liệu mới từ topic /left_wheel_query
    def callback_leftwheelquery(self, data):
        self.data_leftwheel_query = data
        self.is_recv_left_wheel = 1
        self.lastCmdVelReceived = time.time()

    # Hàm callback khi có dữ liệu mới từ topic /right_wheel_query
    def callback_rightwheelquery(self, data):
        self.data_rightwheel_query = data
        self.is_recv_right_wheel = 1
        self.lastCmdVelReceived = time.time()

    def gain_dir(self, x, y):
        if( x > 0 and y > 0):
            return 1                                   # tiến trước 

        elif(( x > 0 and y < 0) or (x > 0 and y == 0) or (x == 0 and y < 0)):
            return 2                                   # quay phải 

        elif(( x < 0 and y > 0) or (x == 0 and y > 0) or (x < 0 and y == 0)):
            return 3                                   # quay trái 

        elif(x < 0 and y < 0 ):
            return 4                                   # lùi sau

        else:                                          # đứng yên
            return 0

    def set_pwm_values(self, dir):
    
        # Calculate the output PWM value by making slow changes to the current value
        if (abs(self.pwmLeftReq) > self.pwmLeftOut):
            self.pwmLeftOut += self.PWM_INCREMENT
        elif (abs(self.pwmLeftReq) < self.pwmLeftOut):
            self.pwmLeftOut -= self.PWM_INCREMENT
        
        if (abs(self.pwmRightReq) > self.pwmRightOut):
            self.pwmRightOut += self.PWM_INCREMENT
        elif(abs(self.pwmRightReq) < self.pwmRightOut):
            self.pwmRightOut -= self.PWM_INCREMENT
        
        # Conditional operator to limit PWM output at the maximum 
        self.pwmLeftOut = self.PWM_MAX  if (self.pwmLeftOut > self.PWM_MAX) else self.pwmLeftOut
        self.pwmRightOut = self.PWM_MAX if (self.pwmRightOut > self.PWM_MAX) else self.pwmRightOut
        
        # PWM output cannot be less than 0
        self.pwmLeftOut = 0 if (self.pwmLeftOut < 0) else self.pwmLeftOut
        self.pwmRightOut = 0 if (self.pwmRightOut < 0) else self.pwmRightOut

        self.pwmout1 = self.pwmLeftOut
        self.pwmout2 = self.pwmRightOut
        
        # Set the PWM value on the pins
        if(dir == 1):
            self.pwm_r1.ChangeDutyCycle(self.pwmLeftOut)
            self.pwm_l1.ChangeDutyCycle(0)
            self.pwm_r2.ChangeDutyCycle(self.pwmRightOut)
            self.pwm_l2.ChangeDutyCycle(0)

        elif(dir == 2):      #phai
            self.pwm_r1.ChangeDutyCycle(self.pwmLeftOut)
            self.pwm_l1.ChangeDutyCycle(0)
            self.pwm_r2.ChangeDutyCycle(0)
            self.pwm_l2.ChangeDutyCycle(self.pwmRightOut)

        elif(dir == 3):      # trai
            self.pwm_r1.ChangeDutyCycle(0)
            self.pwm_l1.ChangeDutyCycle(self.pwmLeftOut)
            self.pwm_r2.ChangeDutyCycle(self.pwmRightOut)
            self.pwm_l2.ChangeDutyCycle(0)

        elif(dir == 4):      # lui
            self.pwm_r1.ChangeDutyCycle(0)
            self.pwm_l1.ChangeDutyCycle(self.pwmLeftOut)
            self.pwm_r2.ChangeDutyCycle(0)
            self.pwm_l2.ChangeDutyCycle(self.pwmRightOut)    

        else:
            self.pwm_r1.ChangeDutyCycle(0)
            self.pwm_l1.ChangeDutyCycle(0)
            self.pwm_r2.ChangeDutyCycle(0)
            self.pwm_l2.ChangeDutyCycle(0)

    def on_shutdown(self, state):
        self.killnode = 1
        self.get_logger().warn("Shutting down! Exiting program...")
        return TransitionCallbackReturn.SUCCESS

    # Hàm điều khiển chính
    def run(self):
        # Record the time
        self.currentMillis = time.time()
        
        # If the time interval has passed, publish the number of ticks and calculate the velocities.
        if (self.currentMillis - self.previousMillis > self.interval):
            
            self.previousMillis = self.currentMillis
        
            # Publish tick counts to topics
            self.pub_leftTicks.publish(self.left_wheel_tick_count)
            self.pub_rightTicks.publish(self.right_wheel_tick_count)

        # Calculate the velocity of the right and left wheels
        self.calc_vel_right_wheel()
        self.calc_vel_left_wheel()

        self.pub_leftwheel_vel.publish(self.left_wheel_vel)
        self.pub_rightwheel_vel.publish(self.right_wheel_vel)

        if (self.is_recv_left_wheel == 1 and self.is_recv_right_wheel == 1):

            # Compute velocity with method 1
            self.currT = time.time()
            self.deltaT = (self.currT-self.prevT)

            # -- ROBOT RUN MANUALLY --
            # -- set vel target for left motor -- 
            vtl = abs(self.data_leftwheel_query.data)
            kpl = 10
            kil = 0
            el = vtl - abs(self.left_wheel_vel.data)
            self.eintegral_l = self.eintegral_l + el*self.deltaT

            ul = kpl*el + kil*self.eintegral_l

            self.pwmLeftReq = abs(ul)
            if(self.pwmLeftReq > self.PWM_MAX):
                self.pwmLeftReq = self.PWM_MAX

            # -- set vel target for right motor -- 
            vtr = abs(self.data_rightwheel_query.data)
            kpr = 10
            kir = 0
            er = vtr - abs(self.right_wheel_vel.data)
            self.eintegral_r = self.eintegral_r + er*self.deltaT

            ur = kpr*er + kir*self.eintegral_r

            self.pwmRightReq = abs(ur)
            if(self.pwmRightReq > self.PWM_MAX):
                self.pwmRightReq = self.PWM_MAX

            # -- 
            self.pwmLeftReq = int(self.pwmLeftReq)
            self.pwmRightReq = int(self.pwmRightReq)

            # -- robot run -- 
            # determine direction of robot
            robot_dir = self.gain_dir(self.data_leftwheel_query.data, self.data_rightwheel_query.data)

            # Case 1: robot stop => reset variables
            if (robot_dir == 0):
                self.pwmLeftReq = 0
                self.pwmRightReq = 0
                self.pwmLeftOut = 0
                self.pwmRightOut = 0
            
            # Case 2: robot change dir => stop robot before running
            if (self.pre_robot_dir != robot_dir):
                self.pwmLeftReq = 0
                self.pwmRightReq = 0
                self.pwmLeftOut = 0
                self.pwmRightOut = 0 

            # -- Case 3 : if real vel robot > target vel robot
            delta_left_vel = abs(self.left_wheel_vel.data) - abs(self.data_leftwheel_query.data)
            delta_right_vel = abs(self.right_wheel_vel.data) - abs(self.data_rightwheel_query.data)

            if delta_left_vel > 2:
                self.pwmLeftReq = 0
                self.pwmLeftOut = 0
            
            if delta_right_vel > 2:
                self.pwmRightReq = 0
                self.pwmRightOut = 0

            # Stop the car if there are no cmd_vel messages
            if(time.time() - self.lastCmdVelReceived > 1.0):
                self.pwmLeftReq = 0
                self.pwmRightReq = 0
                self.is_recv_left_wheel = 0
                self.is_recv_right_wheel = 0

            # self.pwmLeftReq = 10
            # self.pwmRightReq = 10
            self.set_pwm_values(robot_dir)

            self.core_data.linear.x = self.pwmLeftReq
            self.core_data.linear.y = self.pwmRightReq
            self.core_data.angular.x = self.pwmout1
            self.core_data.angular.y = self.pwmout2
            self.core_data.linear.z = robot_dir

            self.pub_coreData.publish(self.core_data)

            self.prevT = self.currT

        else:
            self.pwm_r1.ChangeDutyCycle(0)
            self.pwm_l1.ChangeDutyCycle(0)
            self.pwm_r2.ChangeDutyCycle(0)
            self.pwm_l2.ChangeDutyCycle(0)

		# -- KILL NODE -- 
        if self.killnode:
            sys.exit(0)

def main():
    rclpy.init()
    node = MAIN_CORE()
    print("launch motor_control")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()
