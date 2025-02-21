#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TestFlight(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/target_setpoint', qos_profile)
        
        # tf topic subscriber
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.current_checkpoint = 0
        self.coordinates = [[0.,0.,5.],[-3.,0.,5.],[-3.,5.,5.],[0.,0.,1.]]

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def publish_position_setpoint(self, position: list, yaw: float) -> None:
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = position
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing global position setpoints {position}")

    def timer_callback(self) -> None:
        position = self.coordinates[self.current_checkpoint]
        yaw = 180.
        self.publish_position_setpoint(position,yaw)

    def update_coordinates(self) -> None:
        self.current_checkpoint += 0

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    test_flight = TestFlight()
    rclpy.spin(test_flight)
    test_flight.destroy_node()
    

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
