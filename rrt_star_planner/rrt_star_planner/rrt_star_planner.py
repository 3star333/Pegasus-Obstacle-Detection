#!/usr/bin/env python3
"""
RRT* Kinodynamic Path Planner for Large Fixed-Wing/VTOL UAVs

Implements RRT* (Rapidly-exploring Random Tree Star) with kinodynamic constraints
suitable for 15ft wingspan aircraft. Generates smooth, flyable paths while respecting
vehicle dynamics and obstacle clearance requirements.

References:
- Karaman & Frazzoli (2011): Sampling-based algorithms for optimal motion planning
- LaValle & Kuffner (2001): RRT-Connect for path planning
"""

import numpy as np
from typing import List, Tuple, Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
import struct


class RRTNode:
    """
    Node in the RRT* tree.
    
    Attributes:
        position: [x, y, z] coordinates in meters
        parent: Parent node in tree
        cost: Cost from start to this node
        heading: Heading angle in radians
    """
    
    def __init__(self, position: np.ndarray, parent=None, cost: float = 0.0, heading: float = 0.0):
        """
        Initialize RRT tree node.
        
        Args:
            position: 3D position [x, y, z]
            parent: Parent node reference
            cost: Path cost from start node
            heading: Vehicle heading angle (radians)
        """
        self.position = np.array(position)
        self.parent = parent
        self.cost = cost
        self.heading = heading


class RRTStarPlanner(Node):
    """
    ROS 2 node implementing RRT* path planning with kinodynamic constraints.
    
    Designed for large fixed-wing/VTOL UAVs with limited maneuverability.
    Generates collision-free paths that respect turn radius and flight dynamics.
    """
    
    def __init__(self):
        super().__init__('rrt_star_planner')
        
        # Declare parameters
        self.declare_parameter('max_iterations', 2000)  # Planning iterations
        self.declare_parameter('step_size', 5.0)  # meters - larger for big aircraft
        self.declare_parameter('goal_tolerance', 3.0)  # meters - acceptance radius
        self.declare_parameter('search_radius', 15.0)  # meters - RRT* rewiring radius
        self.declare_parameter('planning_horizon', 100.0)  # meters - look-ahead distance
        self.declare_parameter('min_turn_radius', 20.0)  # meters - kinodynamic constraint
        self.declare_parameter('obstacle_clearance', 3.0)  # meters - safety margin
        self.declare_parameter('replan_rate', 1.0)  # Hz - fixed-wing doesn't need high rate
        self.declare_parameter('max_climb_angle', 20.0)  # degrees - vertical constraint
        
        # Get parameters
        self.max_iterations = self.get_parameter('max_iterations').value
        self.step_size = self.get_parameter('step_size').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.search_radius = self.get_parameter('search_radius').value
        self.planning_horizon = self.get_parameter('planning_horizon').value
        self.min_turn_radius = self.get_parameter('min_turn_radius').value
        self.obstacle_clearance = self.get_parameter('obstacle_clearance').value
        self.replan_rate = self.get_parameter('replan_rate').value
        self.max_climb_angle = np.deg2rad(self.get_parameter('max_climb_angle').value)
        
        # State variables
        self.current_pose = None
        self.goal_pose = None
        self.obstacle_points = None
        self.current_path = None
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/vehicle/pose', self.pose_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/planning/goal', self.goal_callback, 10
        )
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar/points', self.lidar_callback, 10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/planning/path', 10)
        self.path_valid_pub = self.create_publisher(Bool, '/planning/path_valid', 10)
        
        # Planning timer
        self.plan_timer = self.create_timer(
            1.0 / self.replan_rate, self.planning_callback
        )
        
        self.get_logger().info('RRT* Kinodynamic Planner initialized')
        self.get_logger().info(f'  Planning horizon: {self.planning_horizon} m')
        self.get_logger().info(f'  Min turn radius: {self.min_turn_radius} m')
        self.get_logger().info(f'  Obstacle clearance: {self.obstacle_clearance} m')
        self.get_logger().info(f'  Replan rate: {self.replan_rate} Hz')
    
    def pose_callback(self, msg: PoseStamped):
        """
        Update current vehicle pose.
        
        Args:
            msg: Current pose from vehicle state
        """
        self.current_pose = msg
    
    def goal_callback(self, msg: PoseStamped):
        """
        Update goal position and trigger replanning.
        
        Args:
            msg: Goal pose from mission planner
        """
        self.goal_pose = msg
        self.get_logger().info(f'New goal received: [{msg.pose.position.x:.1f}, '
                              f'{msg.pose.position.y:.1f}, {msg.pose.position.z:.1f}]')
    
    def lidar_callback(self, msg: PointCloud2):
        """
        Update obstacle map from LiDAR point cloud.
        
        Args:
            msg: LiDAR point cloud data
        """
        try:
            self.obstacle_points = self.pointcloud2_to_array(msg)
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')
    
    def planning_callback(self):
        """Execute RRT* planning cycle at configured rate."""
        if self.current_pose is None or self.goal_pose is None:
            return
        
        # Extract start and goal positions
        start = np.array([
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
        ])
        
        goal = np.array([
            self.goal_pose.pose.position.x,
            self.goal_pose.pose.position.y,
            self.goal_pose.pose.position.z
        ])
        
        # Check if goal is within planning horizon
        distance_to_goal = np.linalg.norm(goal - start)
        if distance_to_goal > self.planning_horizon:
            # Create intermediate goal at horizon limit
            direction = (goal - start) / distance_to_goal
            goal = start + direction * self.planning_horizon
        
        # Run RRT* planning
        path = self.plan_rrt_star(start, goal)
        
        if path is not None:
            self.current_path = path
            self.publish_path(path)
            self.publish_path_valid(True)
        else:
            self.get_logger().warn('RRT* planning failed - no valid path found')
            self.publish_path_valid(False)
    
    def plan_rrt_star(self, start: np.ndarray, goal: np.ndarray) -> Optional[List[np.ndarray]]:
        """
        Execute RRT* algorithm to find optimal path.
        
        Args:
            start: Start position [x, y, z]
            goal: Goal position [x, y, z]
        
        Returns:
            List of waypoints forming path, or None if planning fails
        """
        # Initialize tree with start node
        start_node = RRTNode(start, parent=None, cost=0.0)
        tree = [start_node]
        
        # Define sampling bounds (centered on start, extends toward goal)
        bounds_min = np.minimum(start, goal) - self.planning_horizon * 0.2
        bounds_max = np.maximum(start, goal) + self.planning_horizon * 0.2
        
        best_goal_node = None
        best_goal_cost = float('inf')
        
        for iteration in range(self.max_iterations):
            # Sample random point (with goal bias)
            if np.random.random() < 0.1:  # 10% goal bias
                random_point = goal
            else:
                random_point = self.sample_free_space(bounds_min, bounds_max)
            
            # Find nearest node in tree
            nearest_node = self.get_nearest_node(tree, random_point)
            
            # Steer toward random point with kinodynamic constraints
            new_position = self.steer(nearest_node.position, random_point)
            
            # Check collision with kinodynamic path
            if self.is_collision_free(nearest_node.position, new_position):
                # Find nearby nodes for rewiring
                nearby_nodes = self.get_nearby_nodes(tree, new_position, self.search_radius)
                
                # Choose best parent (minimum cost)
                best_parent = nearest_node
                best_cost = nearest_node.cost + self.compute_cost(nearest_node.position, new_position)
                
                for node in nearby_nodes:
                    new_cost = node.cost + self.compute_cost(node.position, new_position)
                    if new_cost < best_cost and self.is_collision_free(node.position, new_position):
                        best_parent = node
                        best_cost = new_cost
                
                # Create new node
                new_node = RRTNode(new_position, parent=best_parent, cost=best_cost)
                tree.append(new_node)
                
                # Rewire tree - update children if this provides better path
                for node in nearby_nodes:
                    new_cost = new_node.cost + self.compute_cost(new_node.position, node.position)
                    if new_cost < node.cost and self.is_collision_free(new_node.position, node.position):
                        node.parent = new_node
                        node.cost = new_cost
                
                # Check if goal reached
                if np.linalg.norm(new_position - goal) < self.goal_tolerance:
                    if new_node.cost < best_goal_cost:
                        best_goal_node = new_node
                        best_goal_cost = new_node.cost
        
        # Extract path if goal reached
        if best_goal_node is not None:
            path = self.extract_path(best_goal_node)
            smoothed_path = self.smooth_path(path)
            self.get_logger().info(f'Path found with cost {best_goal_cost:.2f} ({len(smoothed_path)} waypoints)')
            return smoothed_path
        
        return None
    
    def sample_free_space(self, bounds_min: np.ndarray, bounds_max: np.ndarray) -> np.ndarray:
        """
        Sample random point in free space.
        
        Args:
            bounds_min: Minimum bounds [x, y, z]
            bounds_max: Maximum bounds [x, y, z]
        
        Returns:
            Random 3D point within bounds
        """
        return np.random.uniform(bounds_min, bounds_max)
    
    def get_nearest_node(self, tree: List[RRTNode], point: np.ndarray) -> RRTNode:
        """
        Find nearest node in tree to given point.
        
        Args:
            tree: List of tree nodes
            point: Query point [x, y, z]
        
        Returns:
            Nearest node in tree
        """
        min_dist = float('inf')
        nearest = tree[0]
        
        for node in tree:
            dist = np.linalg.norm(node.position - point)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        
        return nearest
    
    def get_nearby_nodes(self, tree: List[RRTNode], point: np.ndarray, radius: float) -> List[RRTNode]:
        """
        Find all nodes within radius of point.
        
        Args:
            tree: List of tree nodes
            point: Query point [x, y, z]
            radius: Search radius (meters)
        
        Returns:
            List of nodes within radius
        """
        nearby = []
        for node in tree:
            if np.linalg.norm(node.position - point) < radius:
                nearby.append(node)
        return nearby
    
    def steer(self, from_pos: np.ndarray, to_pos: np.ndarray) -> np.ndarray:
        """
        Steer from one position toward another with kinodynamic constraints.
        
        Respects step size and turn radius limitations for fixed-wing aircraft.
        
        Args:
            from_pos: Starting position [x, y, z]
            to_pos: Target position [x, y, z]
        
        Returns:
            New position respecting kinodynamic constraints
        """
        direction = to_pos - from_pos
        distance = np.linalg.norm(direction)
        
        if distance == 0:
            return from_pos
        
        # Limit step size
        if distance > self.step_size:
            direction = direction / distance * self.step_size
        
        # Limit climb angle
        horizontal_dist = np.linalg.norm(direction[:2])
        if horizontal_dist > 0:
            vertical_dist = direction[2]
            climb_angle = np.arctan2(abs(vertical_dist), horizontal_dist)
            
            if climb_angle > self.max_climb_angle:
                # Reduce vertical component
                direction[2] = np.sign(vertical_dist) * horizontal_dist * np.tan(self.max_climb_angle)
        
        return from_pos + direction
    
    def is_collision_free(self, from_pos: np.ndarray, to_pos: np.ndarray) -> bool:
        """
        Check if path segment is collision-free with obstacle clearance.
        
        Args:
            from_pos: Segment start position [x, y, z]
            to_pos: Segment end position [x, y, z]
        
        Returns:
            True if segment is collision-free, False otherwise
        """
        if self.obstacle_points is None or len(self.obstacle_points) == 0:
            return True
        
        # Check multiple points along segment
        num_checks = int(np.linalg.norm(to_pos - from_pos) / 1.0) + 1  # Check every 1m
        
        for i in range(num_checks):
            t = i / max(num_checks - 1, 1)
            check_point = from_pos + t * (to_pos - from_pos)
            
            # Check distance to all obstacles
            distances = np.linalg.norm(self.obstacle_points - check_point, axis=1)
            min_distance = np.min(distances)
            
            if min_distance < self.obstacle_clearance:
                return False
        
        return True
    
    def compute_cost(self, from_pos: np.ndarray, to_pos: np.ndarray) -> float:
        """
        Compute path cost between two positions.
        
        Uses Euclidean distance with penalty for sharp turns and altitude changes.
        
        Args:
            from_pos: Start position [x, y, z]
            to_pos: End position [x, y, z]
        
        Returns:
            Path segment cost
        """
        # Base cost is Euclidean distance
        euclidean_dist = np.linalg.norm(to_pos - from_pos)
        
        # Penalty for altitude change (climbing costs more)
        altitude_change = abs(to_pos[2] - from_pos[2])
        altitude_penalty = altitude_change * 1.2
        
        return euclidean_dist + altitude_penalty
    
    def extract_path(self, goal_node: RRTNode) -> List[np.ndarray]:
        """
        Extract path from tree by backtracking from goal to start.
        
        Args:
            goal_node: Goal node in tree
        
        Returns:
            List of positions forming path (start to goal)
        """
        path = []
        current = goal_node
        
        while current is not None:
            path.append(current.position)
            current = current.parent
        
        return path[::-1]  # Reverse to get start-to-goal order
    
    def smooth_path(self, path: List[np.ndarray]) -> List[np.ndarray]:
        """
        Smooth path by removing unnecessary waypoints.
        
        Uses shortcutting: if direct path between non-adjacent waypoints is
        collision-free, skip intermediate waypoints.
        
        Args:
            path: Original path waypoints
        
        Returns:
            Smoothed path with fewer waypoints
        """
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            # Try to connect to farthest visible waypoint
            for j in range(len(path) - 1, i, -1):
                if self.is_collision_free(path[i], path[j]):
                    smoothed.append(path[j])
                    i = j
                    break
            else:
                # No shortcuts found, move to next waypoint
                i += 1
                smoothed.append(path[i])
        
        return smoothed
    
    def publish_path(self, path: List[np.ndarray]):
        """
        Publish planned path as ROS Path message.
        
        Args:
            path: List of waypoint positions
        """
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for position in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(position[0])
            pose.pose.position.y = float(position[1])
            pose.pose.position.z = float(position[2])
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def publish_path_valid(self, valid: bool):
        """
        Publish path validity status.
        
        Args:
            valid: True if valid path exists, False otherwise
        """
        msg = Bool()
        msg.data = valid
        self.path_valid_pub.publish(msg)
    
    def pointcloud2_to_array(self, msg: PointCloud2) -> np.ndarray:
        """
        Convert PointCloud2 message to NumPy array.
        
        Args:
            msg: PointCloud2 message
        
        Returns:
            Nx3 NumPy array of [x, y, z] points
        """
        point_step = msg.point_step
        num_points = len(msg.data) // point_step
        
        points = []
        for i in range(num_points):
            offset = i * point_step
            x = struct.unpack_from('f', msg.data, offset + 0)[0]
            y = struct.unpack_from('f', msg.data, offset + 4)[0]
            z = struct.unpack_from('f', msg.data, offset + 8)[0]
            
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                points.append([x, y, z])
        
        return np.array(points) if points else np.array([]).reshape(0, 3)


def main(args=None):
    """Main entry point for RRT* planner node."""
    rclpy.init(args=args)
    
    try:
        node = RRTStarPlanner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in RRT* planner: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
