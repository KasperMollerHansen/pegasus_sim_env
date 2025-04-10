from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pegasus_sim_env',       # Replace with your package name
            executable='cost_map_node',      # Replace with your node executable name
            name='esdf_cost_map_node',
            output='screen',
            parameters=[
                {'resolution': 1.0},            # Cell size in meters
                {'local_map_size': 300.0},      # Local map size (300 m x 300 m)
                {'global_map_size': 1500.0},    # Global map size (1500 m x 1500 m)
                {'frame_id': 'base_link'},       # Map centered at base_link
                {'safety_distance_min': 5.0},  # Minimum safety distance
                {'safety_distance_max': 10.0}, # Maximum safety distance    
            ]
        )
    ])