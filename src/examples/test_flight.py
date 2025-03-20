#!/usr/bin/env python3

import rclpy
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
        self.coordinates = [
            [0.0, 0.0, 1.0],
            [0.0,0.0, 2.0],
            [0.0, 0.0, 3.0],
            [0.0, 0.0, 4.0],
            [0.0, 0.0, 5.0],
            [0.0, 0.0, 6.0],
            [0.0, 0.0, 7.0],
            [0.0, 0.0, 8.0],
            [0.0, 0.0, 9.0],
            [0.0, 0.0, 10.0],
            [0.0, 0.0, 11.0],
            [0.0, 0.0, 12.0],
            [0.0, 0.0, 13.0],
            [0.0, 0.0, 14.0],
            [0.0, 0.0, 15.0],
            [0.0, 0.0, 16.0],
            [0.0, 0.0, 17.0],
            [0.0, 0.0, 18.0],
            [0.0, 0.0, 19.0],
            [0.0, 0.0, 20.0],
            [0.0, 0.0, 21.0],
            [0.0, 0.0, 22.0],
            [0.0, 0.0, 23.0],
            [0.0, 0.0, 24.0],
            [0.0, 0.0, 25.0],
            [0.0, 0.0, 26.0],
            [0.0, 0.0, 27.0],
            [0.0, 0.0, 28.0],
            [0.0, 0.0, 29.0],
            [0.0, 0.0, 30.0],
            [0.0, 0.0, 31.0],
            [0.0, 0.0, 32.0],
            [0.0, 0.0, 33.0],
            [0.0, 0.0, 34.0],
            [0.0, 0.0, 35.0],
            [0.0, 0.0, 36.0],
            [0.0, 0.0, 37.0],
            [0.0, 0.0, 38.0],
            [0.0, 0.0, 39.0],
            [0.0, 0.0, 40.0],
            [0.0, 0.0, 41.0],
            [0.0, 0.0, 42.0],
            [0.0, 0.0, 43.0],
            [0.0, 0.0, 44.0],
            [0.0, 0.0, 45.0],
            [0.0, 0.0, 46.0],
            [0.0, 0.0, 47.0],
            [0.0, 0.0, 48.0],
            [0.0, 0.0, 49.0],
            [0.0, 0.0, 50.0],
            [0.0, 0.0, 49.0],
            [0.0, 0.0, 48.0],
            [0.0, 0.0, 47.0],
            [0.0, 0.0, 46.0],
            [0.0, 0.0, 45.0],
            [0.0, 0.0, 44.0],
            [0.0, 0.0, 43.0],
            [0.0, 0.0, 42.0],
            [0.0, 0.0, 41.0],
            [0.0, 0.0, 40.0],
            [0.0, 0.0, 39.0],
            [0.0, 0.0, 38.0],
            [0.0, 0.0, 37.0],
            [0.0, 0.0, 36.0],
            [0.0, 0.0, 35.0],
            [0.0, 0.0, 34.0],
            [0.0, 0.0, 33.0],
            [0.0, 0.0, 32.0],
            [0.0, 0.0, 31.0],
            [0.0, 0.0, 30.0],
            [0.0, 0.0, 29.0],
            [0.0, 0.0, 28.0],
            [0.0, 0.0, 27.0],
            [0.0, 0.0, 26.0],
            [0.0, 0.0, 25.0],
            [0.0, 0.0, 24.0],
            [0.0, 0.0, 23.0],
            [0.0, 0.0, 22.0],
            [0.0, 0.0, 21.0],
            [0.0, 0.0, 20.0],
            [0.0, 0.0, 19.0],
            [0.0, 0.0, 18.0],
            [0.0, 0.0, 17.0],
            [0.0, 0.0, 16.0],
            [0.0, 0.0, 15.0],
            [0.0, 0.0, 14.0],
            [0.0, 0.0, 13.0],
            [0.0, 0.0, 12.0],
            [0.0, 0.0, 11.0],
            [0.0, 0.0, 10.0],
            [0.0, 0.0, 9.0],
            [0.0, 0.0, 8.0],
            [0.0, 0.0, 7.0],
            [0.0, 0.0, 6.0],
            [0.0, 0.0, 5.0],
            [0.0, 0.0, 4.0],
            [0.0, 0.0, 3.0],
            [0.0, 0.0, 2.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
        ]

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
        target = np.array(target) * 3
        current = np.array(current)
        if np.linalg.norm(current - target) < 0.5:
            self.current_checkpoint += 1
            self.yaw += 15.0

    @staticmethod
    def transform_position(position: list):
        x, y, z = position
        return [y, x, -z]

    def timer_callback(self) -> None:
        position = self.coordinates[self.current_checkpoint]
        position = [pos*3 for pos in position]
        yaw = np.deg2rad(self.yaw)
        self.publish_position_setpoint(position, yaw)
        self.update_coordinates()


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
