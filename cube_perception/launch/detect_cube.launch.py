from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cube_perception',
            executable='detect_cube_node',
            name='detect_cube_node',
            output='screen',
            parameters=[
                # Example of how to set parameters, if needed.
                # Default values are set in the C++ node.
                # {"passthrough_x_max": 1.5},
                # {"voxel_leaf_size": 0.01},
                # {"color_r_min": 130}, 
                # {"color_r_max": 255},
                # {"color_g_min": 0},
                # {"color_g_max": 100},
                # {"color_b_min": 0},
                # {"color_b_max": 100},
                # {"cluster_tolerance": 0.03}, # 3cm
                # {"cluster_min_size": 20},
                # {"cluster_max_size": 1000}
            ]
        )
    ])
