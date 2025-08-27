#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyMecanumTeleop(Node):
    def __init__(self):
        super().__init__('joy_mecanum_teleop')

        # Sub & Pub
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Tốc độ
        self.linear_speed = 0.5
        self.strafe_speed = 0.5
        self.angular_speed = 1.0

        self.get_logger().info("Joystick teleop for mecanum started.")

    def joy_callback(self, msg: Joy):
        twist = Twist()

        # Giả sử axes[1] = tiến/lùi, axes[0] = xoay trái/phải
        twist.linear.x = msg.axes[1] * self.linear_speed
        twist.angular.z = msg.axes[0] * self.angular_speed

        # Giả sử buttons[4] = strafe trái, buttons[5] = strafe phải
        if msg.buttons[4] == 1:
            twist.linear.y = self.strafe_speed
        elif msg.buttons[5] == 1:
            twist.linear.y = -self.strafe_speed
        else:
            twist.linear.y = 0.0

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyMecanumTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
