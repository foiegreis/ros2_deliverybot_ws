#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool

class TwistToTwistStampedAndSafety(Node):
    def __init__(self):
        super().__init__('twist_to_twist_stamped_and_safety_node')

        self.declare_parameter("input_unstamped_topic", "cmd_vel_ack")
        self.declare_parameter("output_stamped_topic", "cmd_vel_ack_stamped")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("safety_stop_topic", "safety_stop")
        self.declare_parameter("safety_warning_topic", "safety_warning")


        self.input_unstamped_topic_ = self.get_parameter("input_unstamped_topic").value
        self.output_stamped_topic_ = self.get_parameter("output_stamped_topic").value
        self.frame_id_ = self.get_parameter("frame_id").value
        self.safety_stop_topic_ = self.get_parameter("safety_stop_topic").value
        self.safety_warning_topic_ = self.get_parameter("safety_warning_topic").value


        self.twist_sub = self.create_subscription(Twist, self.input_unstamped_topic_, self.twist_callback, 10)
        self.twist_pub = self.create_publisher(TwistStamped, self.output_stamped_topic_, 10)
        self.safety_stop_sub = self.create_subscription(Bool, self.safety_stop_topic_, self.safety_stop_callback, 10)
        self.safety_warning_sub = self.create_subscription(Bool, self.safety_warning_topic_, self.safety_warning_callback, 10)

        self.safety_stop = False
        self.safety_warning = False
        self.current_twist = Twist()
        self.speed_reduction_factor = 1.0
        self.warning_reduction = 0.5 #In warning zone, i reduce velocity by half

        self.get_logger().info(f"Joystick teleop Twist to TwistStamped node with safety stop")

    def twist_callback(self, msg):
        self.current_twist = msg
        self.publish_modified_twist()

    def safety_warning_callback(self, msg):
        if msg.data and not self.safety_warning:
            self.get_logger().info("Safety warning activated")
            self.safety_warning = True
            self.speed_reduction_factor = self.warning_reduction 
        elif not msg.data and self.safety_warning:
            self.get_logger().info("Safety warning deactivated")
            self.safety_warning = False
            self.speed_reduction_factor = 1.0 if not self.safety_stop else 0.0
        self.publish_modified_twist()

    def safety_stop_callback(self, msg):
        if msg.data and not self.safety_stop:
            self.get_logger().info("Safety stop activated")
            self.safety_stop = True
            self.speed_reduction_factor = 0.0
        elif not msg.data and self.safety_stop:
            self.get_logger().info("Safety stop deactivated")
            self.safety_stop = False
            self.speed_reduction_factor = self.warning_reduction if self.safety_warning else 1.0
        self.publish_modified_twist()


    def publish_modified_twist(self):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = self.frame_id_
        twist_stamped.twist.linear.x = self.current_twist.linear.x * self.speed_reduction_factor
        twist_stamped.twist.angular.z = self.current_twist.angular.z * self.speed_reduction_factor

        self.twist_pub.publish(twist_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStampedAndSafety()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()