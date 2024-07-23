#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
import math

class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('twist_to_twist_stamped_node')

        self.declare_parameter("input_unstamped_topic", "cmd_vel")
        self.declare_parameter("output_stamped_topic", "cmd_vel_stamped")
        self.declare_parameter("frame_id", "base_link")

        self.input_unstamped_topic_ = self.get_parameter("input_unstamped_topic").value
        self.output_stamped_topic_ = self.get_parameter("output_stamped_topic").value
        self.frame_id_ = self.get_parameter("frame_id").value

        self.twist_sub = self.create_subscription(Twist, self.input_unstamped_topic_, self.twist_callback, 10)
        self.twist_pub = self.create_publisher(TwistStamped, self.output_stamped_topic_, 10)
        
        self.get_logger().info(f"Twist to TwistStamped node has been started")
        

    def twist_callback(self, msg):
        limited_angular_vel = msg.angular.z
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = self.frame_id_
        twist_stamped.twist.linear.x = msg.linear.x
        twist_stamped.twist.angular.z = limited_angular_vel
        self.twist_pub.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()