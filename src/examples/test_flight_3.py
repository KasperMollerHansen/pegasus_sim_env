#!/usr/bin/env python3

import rclpy
import math
import time
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TestFlight(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__("offboard_control_takeoff_and_land")

        # Declare the 'start' parameter with a default value
        self.declare_parameter("start", True)

        # Get the value of the 'start' parameter
        self.start = self.get_parameter("start").get_parameter_value().bool_value

        # Configure QoS profile to match AStarPlanner
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Matches AStarPlanner
            durability=DurabilityPolicy.VOLATILE,  # Matches AStarPlanner
            history=HistoryPolicy.KEEP_LAST,  # Matches AStarPlanner
            depth=1,  # Matches AStarPlanner
        )

        # Path publisher
        self.path_publisher = self.create_publisher(
            Path, "/osep/viewpoints", qos_profile
        )

        odometry_qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,  # Match the publisher's QoS
        durability=DurabilityPolicy.VOLATILE,      # No need to store messages
        history=HistoryPolicy.KEEP_LAST,           # Keep only the last message
        depth=1,                                   # Queue size of 1
        )

        # tf topic subscriber
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.vehicle_odometry_subscription = self.create_subscription(
            Odometry,  # Update the message type to nav_msgs.msg.Odometry
            "/isaac/odom",  # Update the topic name
            self.vehicle_odometry_callback,
            odometry_qos_profile,
        )

        self.waypoints_adjustment = self.create_subscription(
            Path,
            "/planner/viewpoints_adjusted",
            self.adjust_waypoints_callback,
            qos_profile,
        )

        self.vehicle_odometry = Odometry()

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.current_checkpoint = 0
        self.coordinates, self.yaw  = generate_coordinates()
        self.number_of_waypoints = 23
        self.coordinates_to_vist = self.coordinates.copy()
        

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.last_update_time = time.time()  # Initialize the last update time
        self.update_cooldown = 1.0  # Cooldown period in seconds (adjust as needed)

    def adjust_waypoints_callback(self, waypoints_adjusted):
        new_coordinates = []
        i = 0
        for pose in waypoints_adjusted.poses:
            i += 1
            header = pose.header.frame_id # Check if the header frame_id is "odom"
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            new_coordinates.append([x, y, z])
            if i == 1 and header == "":
                self.current_checkpoint += 1

        self.coordinates_to_vist = new_coordinates

    def vehicle_odometry_callback(self, vehicle_odometry):
        """Callback function for vehicle_odometry topic subscriber."""
        self.vehicle_odometry = vehicle_odometry

    def publish_path(self) -> None:
        """Publish the next three setpoints as a Path message."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"

        for i in range(self.number_of_waypoints):
            checkpoint_index = (self.current_checkpoint + i) % len(self.coordinates)
            position = self.coordinates[checkpoint_index]
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "odom"
            pose.pose.position.x = position[0]
            pose.pose.position.y = position[1]
            pose.pose.position.z = position[2]

            # Calculate yaw angle toward the center
            yaw = self.yaw[checkpoint_index][0]  # Use the yaw from the yaw list

            # Convert yaw to quaternion
            quaternion = R.from_euler('xyz', [0, 0, yaw]).as_quat()
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info(f"Publishing path with next 3 setpoints.")

    def update_coordinates(self) -> None:
        """Check if the vehicle is close to any point in the coordinates_to_vist vector."""
        current_time = time.time()
        if current_time - self.last_update_time < self.update_cooldown:
            # Skip update if cooldown period hasn't passed
            return

        current = np.array([
            self.vehicle_odometry.pose.pose.position.x,
            self.vehicle_odometry.pose.pose.position.y,
            self.vehicle_odometry.pose.pose.position.z,
        ])

        for i, target in enumerate(self.coordinates_to_vist):
            target = np.array(target)
            distance = np.linalg.norm(current - target)
            self.get_logger().info(f"Distance to point {i}: {distance}")

            if distance < 2.0:  # Threshold for being "close"
                self.get_logger().info(f"Reached point {i}: {target}")
                self.current_checkpoint += i+1  # Update checkpoint to the next point
                self.last_update_time = current_time  # Update the last update time
                break

    def timer_callback(self) -> None:
        self.publish_path()
        self.update_coordinates()


def generate_points_in_radius(center_x, center_y, center_z, radius, num_points, height):
    """Generates points within a specified radius around a center point."""
    coordinates = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        coordinates.append([float(x), float(y), float(height)])
    return coordinates

def calculate_distance(point1, point2):
    """Calculates the Euclidean distance between two points."""
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2)

def calculate_angle(point, center):
    """Calculates the angle of a point relative to the center."""
    return math.atan2(point[1] - center[1], point[0] - center[0])

def generate_coordinates():
    """Generates coordinates in a circle, starting from the closest point to [0, 0, 50]."""
    coordinates = [[0.0, 0.0, 100.0], 
                      [185.0, -8.0, 120.0], 
                      [188.0, -11.0, 130.0], [174.0, -11.0, 142.0], [160.0, -11.0, 155.0],
                      [160.0, -6.0, 155.0],[174.0, -6.0, 142.0], [188.0, -6.0, 130.0],
                      [210.0, -6.0, 125.0], [228.0, -6.0, 130.0], [250.0, -6.0, 135.0],
                      [250.0, -11.0, 135.0],[228.0, -11.0, 130.0], [210.0, -11.0, 125.0],
                      [198.0, -11.0, 107.0], [193.0, -11.0, 90.0], [188.0, -11.0, 67.0],
                      [188.0, -6.0, 67.0], [193.0, -6.0, 90.0], [198.0, -6.0, 107.0], 
                      [185.0, 0.0, 120.0], [20.0, 0.0, 105.0]] # Initial points to start from
    yaw = [[0.0], 
           [0.0], 
           [np.pi/2], [np.pi/2], [np.pi/2 - np.pi/18],
           [-np.pi/2 + np.pi/18], [-np.pi/2], [-np.pi/2],
           [-np.pi/2], [-np.pi/2], [-np.pi/2 -np.pi/18],
           [np.pi/2 + np.pi/18], [np.pi/2], [np.pi/2],
           [np.pi/2], [np.pi/2], [np.pi/2 - np.pi/18],
           [-np.pi/2 + np.pi/18], [-np.pi/2], [-np.pi/2],
           [0.0],[np.pi]]  # Yaw angles for each initial point

    return coordinates, yaw

def main(args=None) -> None:
    print("Starting offboard control node...")
    rclpy.init(args=args)
    test_flight = TestFlight()
    rclpy.spin(test_flight)
    test_flight.destroy_node()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)