#!/usr/bin/env python3

import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleOdometry,
    VehicleStatus,
)

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TestFlight(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__("offboard_control_takeoff_and_land")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, "in/target_setpoint", qos_profile
        )

        # tf topic subscriber
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.vehicle_odometry_subscription = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.vehicle_odometry_callback,
            qos_profile,
        )

        self.vehicle_odometry = VehicleOdometry()

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.current_checkpoint = 0
        self.coordinates = generate_coordinates(center_x=200, center_y=0, radius=30, num_points=8, height=125)
        self.yaw = 0.0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_odometry_callback(self, vehicle_odometry):
        """Callback function for vehicle_odometry topic subscriber."""
        self.vehicle_odometry = vehicle_odometry

    def publish_position_setpoint(self, position: list, yaw: float) -> None:
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = position
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing global position setpoints {position}")

    def update_coordinates(self) -> None:
        target = self.coordinates[self.current_checkpoint]
        current = self.vehicle_odometry.position
        current = self.transform_position(current)
        target = np.array(target)
        current = np.array(current)
        if np.linalg.norm(current - target) < 2.0:
            self.current_checkpoint += 1

    @staticmethod
    def transform_position(position: list):
        x, y, z = position
        return [y, x, -z]

    def timer_callback(self) -> None:
        position = self.coordinates[self.current_checkpoint]
        position = [pos for pos in position]
        yaw = np.deg2rad(self.yaw)
        self.publish_position_setpoint(position, yaw)
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

def generate_coordinates(center_x=150, center_y=0, center_z=0, radius=75, num_points=90, height=125):
    """Generates coordinates in a circle, starting from the closest point to [0, 0, 50]."""
    initial_point = [1.0, 0.0, 50.0]
    end_point = [0.0, 0.0, float(height)]
    generated_points = generate_points_in_radius(center_x, center_y, center_z, radius, num_points, height)

    # Find the closest point
    closest_point = min(generated_points, key=lambda point: calculate_distance(initial_point, point))
    closest_point_index = generated_points.index(closest_point)

    # Arrange the points in circular order starting from the closest
    ordered_points = generated_points[closest_point_index:] + generated_points[:closest_point_index]

    coordinates = [initial_point]+[ordered_points[0]] + ordered_points[1:] + [end_point]

    return coordinates

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
