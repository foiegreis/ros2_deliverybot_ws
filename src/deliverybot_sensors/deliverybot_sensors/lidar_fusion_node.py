#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import tf2_ros
from tf2_sensor_msgs import do_transform_cloud
from sensor_msgs_py import point_cloud2

class LidarFusionNode(Node):

    def __init__(self):
        super().__init__('lidar_fusion_node')
        
        # Create subscribers for each LiDAR
        self.create_subscription(PointCloud2, '/lidar_front_left/points', self.lidar_callback, 10)
        self.create_subscription(PointCloud2, '/lidar_front_right/points', self.lidar_callback, 10)
        self.create_subscription(PointCloud2, '/lidar_rear_left/points', self.lidar_callback, 10)
        self.create_subscription(PointCloud2, '/lidar_rear_right/points', self.lidar_callback, 10)
        
        self.get_logger().info("Lidar fusion node has been started ")
        # Create publisher for fused point cloud
        self.publisher = self.create_publisher(PointCloud2, '/fused_pointcloud', 10)
        
        # Buffer to store latest point clouds
        self.latest_clouds = {}
        
        # Set up TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def lidar_callback(self, msg):
        # Store the latest point cloud for each LiDAR
        self.latest_clouds[msg.header.frame_id] = msg
        
        # Attempt to fuse point clouds
        self.fuse_point_clouds()

    def fuse_point_clouds(self):
        if len(self.latest_clouds) < 4:
            return  # Wait until we have data from all LiDARs
        
        combined_points = []
        
        for frame_id, cloud_msg in self.latest_clouds.items():
            try:
                # Transform point cloud to base_link frame
                transform = self.tf_buffer.lookup_transform('base_link', frame_id, rclpy.time.Time())
                cloud_transformed = do_transform_cloud(cloud_msg, transform)
                
                # Convert to list of points
                points = list(point_cloud2.read_points(cloud_transformed, field_names=("x", "y", "z")))
                combined_points.extend(points)
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.get_logger().warn(f'Could not transform point cloud from {frame_id} to base_link')
        
        if combined_points:
            # Convert combined points to numpy array
            all_points = np.array(combined_points)
            
            # Extract x, y, z coordinates
            xyz_points = all_points[['x', 'y', 'z']].view((np.float32, 3)).reshape(-1, 3)
            
            # Create and publish fused point cloud message
            header = self.latest_clouds[list(self.latest_clouds.keys())[0]].header
            header.frame_id = 'base_link'
            
            fused_msg = point_cloud2.create_cloud_xyz32(header, xyz_points)
            self.publisher.publish(fused_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()