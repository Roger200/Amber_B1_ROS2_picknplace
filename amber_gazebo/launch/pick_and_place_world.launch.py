import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the directory where the world file is located
    pkg_amber_gazebo = get_package_share_directory('amber_gazebo')
    
    # Path to the custom world file
    world_file_name = 'pick_and_place.world'
    world_path = os.path.join(pkg_amber_gazebo, 'worlds', world_file_name)

    # Launch Gazebo with the custom world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'),
            '/gazebo.launch.py'
        ]),
        launch_arguments={'world': world_path}.items()
    )

    return LaunchDescription([
        gazebo_launch
    ])
