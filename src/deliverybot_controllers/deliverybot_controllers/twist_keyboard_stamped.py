#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import math

class KeyboardToTwistStamped(Node):
    def __init__(self):
        super().__init__('key_twist_to_twist_stamped')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10)
        self.publisher = self.create_publisher(TwistStamped, 'cmd_vel_stamped', 10)
        self.wheelbase = 0.668123  # Replace with your robot's wheelbase
        self.max_steering_angle = math.pi/4  # 45 degrees, adjust as needed

    def limit_angular_velocity(self, linear_vel, angular_vel):
       
        max_angular_vel = angular_vel * 0.4
        #max_angular_vel = abs(linear_vel) * math.tan(self.max_steering_angle) / self.wheelbase

        return max_angular_vel
        #return max(-max_angular_vel, min(angular_vel, max_angular_vel))
        

    def twist_callback(self, msg):
        limited_angular_vel = self.limit_angular_velocity(msg.linear.x, msg.angular.z)
        
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"
        twist_stamped.twist.linear.x = msg.linear.x
        twist_stamped.twist.angular.z = limited_angular_vel

        #self.get_logger().info(f"Original angular.z: {msg.angular.z}, Limited angular.z: {limited_angular_vel}")
        #self.get_logger().info(f"Publishing TwistStamped: linear.x={twist_stamped.twist.linear.x}, angular.z={twist_stamped.twist.angular.z}")
        self.publisher.publish(twist_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardToTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()