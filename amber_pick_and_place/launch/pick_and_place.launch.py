from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments for parameters
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "place_pose_xyz",
            default_value='[0.0, 0.5, 0.05]', # Default as a string representing a list
            description="Target XYZ coordinates for placing the cube."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "home_pose_joints",
            default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]', # Default for 7 joints
            description="Joint values for the robot's home pose."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "offset_above_cube",
            default_value='0.1',
            description="Offset above the cube for pre-grasp and pre-place approaches."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "grasp_depth_offset",
            default_value='0.01', # slight downward offset for grasp
            description="Slight Z offset to ensure gripper reaches cube."
        )
    )

    # Get parameter values from launch arguments
    # Note: These need to be converted to the correct type if passed directly as string.
    # The C++ node currently expects double array for xyz and joints.
    # ROS 2 launch system handles string-to-basic-type conversion (like double for offset_above_cube).
    # For lists (vectors), the C++ node's parameter parsing from stringified list might need care,
    # or use a YAML file for complex parameters.
    # The node currently uses get_parameter("...").as_double_array() which expects a list of doubles.
    # The string '[0.0, 0.5, 0.1]' needs to be parsed by the node or passed as a YAML file.
    # For simplicity, the node attempts to parse these. Let's ensure the format is compatible or use YAML.
    # Using YAML is generally more robust for lists.

    # Create a params file path (optional, but good practice for complex params)
    # config_file_path = os.path.join(
    #     get_package_share_directory('amber_pick_and_place'),
    #     'config',
    #     'pick_and_place_params.yaml'
    # )

    # Create the node action
    pick_and_place_node = Node(
        package='amber_pick_and_place',
        executable='pick_and_place_node',
        name='pick_and_place_node',
        output='screen',
        parameters=[
            # Pass launch configurations directly. The node will handle parsing.
            # This works for simple types. For lists, direct string passing might be tricky.
            # The node's get_parameter().as_double_array() might not parse "[0.0, ...]" correctly.
            # It's better to use a YAML file or set them individually if not using YAML.
            # However, ROS 2 Humble's parameter system has improved list-from-string parsing.
            {
                "place_pose_xyz": LaunchConfiguration("place_pose_xyz"),
                "home_pose_joints": LaunchConfiguration("home_pose_joints"),
                "offset_above_cube": LaunchConfiguration("offset_above_cube"),
                "grasp_depth_offset": LaunchConfiguration("grasp_depth_offset"),
            }
            # Or load from YAML:
            # config_file_path 
        ]
    )

    return LaunchDescription(declared_arguments + [pick_and_place_node])
