#!/usr/bin/env python3
"""
LiDAR Obstacle Detection Node for PX4 UAVs

This node processes PointCloud2 data from a LiDAR sensor and detects obstacles
within a configurable forward-facing detection zone. Designed for simulation-to-real
transfer with minimal code changes.

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32
import numpy as np
import struct


class ObstacleDetectorNode(Node):
    """
    ROS 2 node for real-time LiDAR-based obstacle detection.
    
    Subscribes to PointCloud2 data, filters points within a forward-facing detection
    box, and publishes obstacle detection status and minimum distance.
    """

    def __init__(self):
        super().__init__('obstacle_detector')
        
        # Declare parameters with defaults (configured for 15ft wingspan UAV)
        self.declare_parameter('danger_distance', 80.0)  # meters
        self.declare_parameter('detection_width', 9.0)  # meters (|Y|)
        self.declare_parameter('detection_height', 6.0)  # meters (|Z|)
        self.declare_parameter('min_distance', 2.0)  # meters (ignore too close)
        self.declare_parameter('obstacle_inflation', 3.0)  # meters (safety margin)
        self.declare_parameter('lidar_topic', '/lidar/points')
        self.declare_parameter('update_rate', 10.0)  # Hz
        
        # Get parameter values
        self.danger_distance = self.get_parameter('danger_distance').value
        self.detection_width = self.get_parameter('detection_width').value
        self.detection_height = self.get_parameter('detection_height').value
        self.min_distance = self.get_parameter('min_distance').value
        self.obstacle_inflation = self.get_parameter('obstacle_inflation').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        
        # Create subscription to LiDAR point cloud
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.lidar_callback,
            10
        )
        
        # Create publishers
        self.obstacle_detected_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.obstacle_distance_pub = self.create_publisher(Float32, '/obstacle_distance', 10)
        
        # Log initialization
        self.get_logger().info('Obstacle Detector Node initialized (15ft wingspan UAV)')
        self.get_logger().info(f'  Danger distance: {self.danger_distance} m')
        self.get_logger().info(f'  Detection width: ±{self.detection_width} m')
        self.get_logger().info(f'  Detection height: ±{self.detection_height} m')
        self.get_logger().info(f'  Minimum distance: {self.min_distance} m')
        self.get_logger().info(f'  Obstacle inflation: {self.obstacle_inflation} m')
        self.get_logger().info(f'  Subscribing to: {self.lidar_topic}')

    def lidar_callback(self, msg: PointCloud2):
        """
        Process incoming LiDAR point cloud data.
        
        Args:
            msg: PointCloud2 message containing LiDAR scan data
        """
        try:
            # Convert PointCloud2 to NumPy array
            points = self.pointcloud2_to_array(msg)
            
            if points is None or len(points) == 0:
                self.publish_no_obstacle()
                return
            
            # Filter points within detection box (Phase 5)
            filtered_points = self.filter_detection_box(points)
            
            if len(filtered_points) == 0:
                self.publish_no_obstacle()
                return
            
            # Compute distances with obstacle inflation and find minimum (Phase 5)
            distances = np.linalg.norm(filtered_points, axis=1)
            # Apply obstacle inflation for structural clearance (wing tips, props)
            inflated_distances = distances - self.obstacle_inflation
            min_dist = np.min(inflated_distances)
            
            # Obstacle decision logic (Phase 6)
            obstacle_detected = min_dist < self.danger_distance
            
            # Publish results
            self.publish_obstacle_status(obstacle_detected, min_dist)
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')

    def pointcloud2_to_array(self, msg: PointCloud2) -> np.ndarray:
        """
        Convert PointCloud2 message to NumPy array (Phase 4).
        
        Assumes FLU (Forward-Left-Up) coordinate frame:
        - X: forward
        - Y: left
        - Z: up
        
        Args:
            msg: PointCloud2 message
            
        Returns:
            Nx3 NumPy array of [x, y, z] points, or None if conversion fails
        """
        # Get point step and number of points
        point_step = msg.point_step
        num_points = len(msg.data) // point_step
        
        # Parse XYZ from point cloud
        points = []
        for i in range(num_points):
            offset = i * point_step
            
            # Extract x, y, z (assuming float32, adjust if needed)
            x = struct.unpack_from('f', msg.data, offset + 0)[0]
            y = struct.unpack_from('f', msg.data, offset + 4)[0]
            z = struct.unpack_from('f', msg.data, offset + 8)[0]
            
            # Discard NaN points (Phase 4)
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                points.append([x, y, z])
        
        return np.array(points) if points else None

    def filter_detection_box(self, points: np.ndarray) -> np.ndarray:
        """
        Filter points within the forward-facing detection box (Phase 5).
        
        Bounding box criteria:
        - X > min_distance (forward, ignoring too close)
        - |Y| < detection_width (horizontal width)
        - |Z| < detection_height (vertical height)
        
        Args:
            points: Nx3 array of [x, y, z] points
            
        Returns:
            Filtered Nx3 array of points within detection box
        """
        # Forward direction filter
        forward_mask = points[:, 0] > self.min_distance
        
        # Horizontal width filter
        width_mask = np.abs(points[:, 1]) < self.detection_width
        
        # Vertical height filter
        height_mask = np.abs(points[:, 2]) < self.detection_height
        
        # Combine all filters
        combined_mask = forward_mask & width_mask & height_mask
        
        return points[combined_mask]

    def publish_obstacle_status(self, detected: bool, distance: float):
        """
        Publish obstacle detection results (Phase 6).
        
        Args:
            detected: True if obstacle is within danger distance
            distance: Minimum distance to obstacle in meters
        """
        # Publish detection flag
        bool_msg = Bool()
        bool_msg.data = detected
        self.obstacle_detected_pub.publish(bool_msg)
        
        # Publish minimum distance
        dist_msg = Float32()
        dist_msg.data = distance
        self.obstacle_distance_pub.publish(dist_msg)
        
        # Log detection events
        if detected:
            self.get_logger().warn(f'⚠️  OBSTACLE DETECTED at {distance:.2f} m')
        else:
            self.get_logger().debug(f'Clear - nearest point: {distance:.2f} m')

    def publish_no_obstacle(self):
        """
        Publish no-obstacle status when no points are in detection box.
        """
        bool_msg = Bool()
        bool_msg.data = False
        self.obstacle_detected_pub.publish(bool_msg)
        
        # Publish max distance to indicate "clear"
        dist_msg = Float32()
        dist_msg.data = float('inf')
        self.obstacle_distance_pub.publish(dist_msg)


def main(args=None):
    """Main entry point for the obstacle detector node."""
    rclpy.init(args=args)
    
    try:
        node = ObstacleDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in obstacle detector: {str(e)}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
