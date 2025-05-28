#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import csv
import os

class PathCollector(Node):
    def __init__(self):
        super().__init__('logger')
        # For loggin only position

        # self.data_type = "position"
        # self.path_topic_name = "/planner/ground_truth_trajectory"
        # self.path_fname = "gt_traj_03.csv"

        # For loggin both position and orientation

        self.data_type = "pose" 
        # self.path_topic_name = "/planner/ground_truth_trajectory"
        self.path_topic_name = "osep/viewpoints"
        # self.path_fname = "ground_truth_3d_2.csv"
        self.path_fname = "osep_3d_2.csv"


        # For logging extracted skeleton


        self.path_sub = self.create_subscription(Path, self.path_topic_name, self.on_path_msg, 10)
        self.path_done = False
        self.get_logger().info(f'Waiting for first Path message on {self.path_topic_name}...')

    def on_path_msg(self, msg: Path):
        # prepare file in package's current dir (or absolute path if you prefer)
        if self.path_done: return

        with open(self.path_fname, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # writer.writerow(['x','y','z'])
            if self.data_type == "position":
                writer.writerow(['x', 'y', 'z'])
                for stamped in msg.poses:
                    p = stamped.pose.position
                    writer.writerow([p.x, p.y, p.z])
            elif self.data_type == "pose":
                writer.writerow(['x', 'y', 'z', 'ox', 'oy', 'oz', 'ow'])
                for stamped in msg.poses:
                    p = stamped.pose.position
                    o = stamped.pose.orientation
                    writer.writerow([p.x, p.y, p.z, o.x, o.y, o.z, o.w])

        if msg.poses:
            t0 = msg.poses[0].header.stamp
            t1 = msg.poses[-1].header.stamp
            # convert to seconds
            sec0 = t0.sec + t0.nanosec * 1e-9
            sec1 = t1.sec + t1.nanosec * 1e-9
            duration = sec1 - sec0
            self.get_logger().info(
                f'Path spans {duration:.6f} seconds '
                f'(from {t0.sec}.{t0.nanosec:09d} to {t1.sec}.{t1.nanosec:09d})'
            )

        self.get_logger().info(f'Wrote {len(msg.poses)} points to {self.path_fname}')
        self.path_done = True #only log once...

def main(args=None):
    rclpy.init(args=args)
    node = PathCollector()
    try:
        # spin in small increments until our callback sets `done = True`
        while rclpy.ok() and (not node.path_done):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()