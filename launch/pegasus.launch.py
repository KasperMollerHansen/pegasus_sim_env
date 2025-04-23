from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pegasus_sim_env',              # Replace with your package name
            executable='cost_map_node',             # Replace with your node executable name
            name='esdf_cost_map_node',
            output='screen',
            parameters=[
                {'resolution': 1.0},                # Cell size in meters
                {'free_center_radius': 5.0},        # Free space radius
                {'local_map_size': 400.0},          # Local map size (400 m x 400 m)
                {'global_map_size': 1600.0},        # Global map size (1600 m x 1600 m)
                {'frame_id': 'base_link'},          # Map centered at base_link
                {'safety_distance_min': 7.0},       # Minimum safety distance
                {'safety_distance_max': 12.0},      # Maximum safety distance    
            ]
        ),
        Node(
            package='pegasus_sim_env',              # Replace with your package name
            executable='a_star_node',               # Replace with your node executable name
            name='a_star_planner',
            output='screen',
            parameters=[
                {'obstacle_threshold': 50},         # Obstacle threshold, depends on safety distance
                {'frame_id': 'base_link'},          # Map centered at base_link
                {'interpolation_distance': 2.0},    # Interpolation distance
            ]
        ),
    ])