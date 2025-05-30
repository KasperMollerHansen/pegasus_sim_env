from launch import LaunchDescription
from launch_ros.actions import Node
import math

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pegasus_sim_env",              # Replace with your package name
            executable="cost_map_node",             # Replace with your node executable name
            name="esdf_cost_map_node",
            output="screen",
            parameters=[
                {"resolution": 1.0},                # Cell size in meters
                {"free_center_radius": 5.0},        # Free space radius
                {"local_map_size": 400.0},          # Local map size (400 m x 400 m)
                {"global_map_size": 1600.0},        # Global map size (1600 m x 1600 m)
                {"frame_id": "base_link"},          # Map centered at base_link
                {"safety_distance_min": 8.0},       # Minimum safety distance
                {"safety_distance_max": 12.0},       # Maximum safety distance    
            ]
        ),
        Node(
            package="pegasus_sim_env",              # Replace with your package name
            executable="planner_node",              # Replace with your node executable name
            name="planner",
            output="screen",
            parameters=[
                {'obstacle_threshold': 50},         # Obstacle threshold, depends on safety distance 50 = mean(max+min)
                {"frame_id": "base_link"},          # Map centered at base_link
                {"interpolation_distance": 3.0},    # Interpolation distance
                {"costmap_topic": "/local_costmap/costmap"},
                {"waypoints_topic": "/osep/viewpoints"},
                {"path_planner_prefix": "/planner"},
                {"ground_truth_update_interval": 8000}, # Update interval for ground truth (miliseconds)
                {"extra_safty_distance": 1.0},          # Extra safety distance (Total = mean(max+min) + extra) )
            ]
        ),
        Node(
            package="pegasus_sim_env",              # Replace with your package name
            executable="vel_controller",            # Replace with your node executable name
            name="velocity_controller",
            output="screen",
            parameters=[
                {"interpolation_distance": 3.0},    # Interpolation distance
                {"frame_id": "base_link"},          # Map centered at base_link
                {"max_velocity": 10.0},             # Maximum velocity
                {"min_velocity": 1.5},              # Minimum velocity (1 m/s)
                {"max_acceleration": 0.2},          # Acceleration
                {"max_angle_change": math.pi / 18},  # Maximum angle change (10 degrees)
                {"max_yaw_to_velocity_angle": 2* math.pi / 3}, # Maximum angle between yaw and velocity (120 degrees)
                {"path_topic": "/planner/smoothed_path"}, # Path topic
            ]
        ),
    ])