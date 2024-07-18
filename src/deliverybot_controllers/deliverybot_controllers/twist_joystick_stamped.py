#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
import math

class JoyToTwistStamped(Node):
    def __init__(self):
        super().__init__('joy_to_twist_stamped')
        self.joy_sub = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        self.twist_pub = self.create_publisher(TwistStamped, 'cmd_vel_stamped', 10)
        
        self.max_speed = 1.0  # m/s
        self.max_steering_angle = math.pi/4  # 45 degrees
        self.wheelbase = 1.353834  # meters
        self.get_logger().info(f"Joystick teleop Twist to TwistStamped node: clipping angular velocities > pi/4 ")

    def limit_angular_velocity(self, linear_vel, angular_vel):
        if abs(linear_vel) < 1e-6:
            return angular_vel
        
        #max_angular_vel = angular_vel * 0.4
        max_angular_vel = abs(linear_vel) * math.tan(self.max_steering_angle) / self.wheelbase

        #return max_angular_vel
        return max(-max_angular_vel, min(angular_vel, max_angular_vel))

        

    def twist_callback(self, msg):
        #limited_angular_vel = self.limit_angular_velocity(msg.linear.x, msg.angular.z)
        limited_angular_vel = msg.angular.z
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"
        twist_stamped.twist.linear.x = msg.linear.x
        twist_stamped.twist.angular.z = limited_angular_vel

        #self.get_logger().info(f"linear.x: {twist_stamped.twist.linear.x}, angular.z: {msg.angular.z} -> {twist_stamped.twist.angular.z}")
        self.twist_pub.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()