import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Package Paths
    pkg_amber_gazebo = get_package_share_directory('amber_gazebo')
    pkg_amber_b1_description = get_package_share_directory('amber_b1_description')
    pkg_amber_arm_moveit_config = get_package_share_directory('amber_arm_moveit_config')
    pkg_cube_perception = get_package_share_directory('cube_perception')
    pkg_amber_pick_and_place = get_package_share_directory('amber_pick_and_place')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Pick and Place parameters - allow overriding via launch arguments
    place_pose_xyz = LaunchConfiguration('place_pose_xyz', default='[0.0, 0.5, 0.05]')
    home_pose_joints = LaunchConfiguration('home_pose_joints', default='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]')


    # 1. Gazebo Launch
    world_file_name = 'pick_and_place.world'
    world_path = os.path.join(pkg_amber_gazebo, 'worlds', world_file_name)
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'),
            '/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': world_path,
            'verbose': 'false', # Set to true for more Gazebo output
            'pause': 'false',    # Set to true to start Gazebo paused
            # 'extra_gazebo_args': '--ros-args --params-file /path/to/your/gazebo_params.yaml' # If needed
        }.items()
    )
    
    # 2. Robot State Publisher
    # Load robot description (URDF)
    # The xacro processing must happen inside a launch function if it depends on other launch configs
    # or if we want to pass it as a string to robot_state_publisher.
    
    # This OpaqueFunction allows us to run Python code at launch time to process xacro
    def load_robot_description(context, *args, **kwargs):
        robot_description_path = os.path.join(pkg_amber_b1_description, 'urdf', 'amber_b1.urdf.xacro')
        # Process xacro file
        robot_description_config = xacro.process_file(robot_description_path)
        robot_description = robot_description_config.toxml()
        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_description
                }],
            )
        ]

    # robot_state_publisher_node_loader = OpaqueFunction(function=load_robot_description)
    # Assuming MoveIt's demo.launch.py will handle robot_state_publisher.
    # If not, or if there are issues, the above OpaqueFunction can be re-enabled
    # and demo.launch.py might need to be configured not to launch its own RSP.


    # 3. MoveIt! Launch
    # demo.launch.py from MoveIt typically starts:
    # - move_group node
    # - robot_state_publisher (usually configured with the correct URDF)
    # - rviz2 with MoveIt plugins
    # - optionally, a joint_state_publisher_gui
    # - optionally, Gazebo (this is the part we want to ensure doesn't conflict)

    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amber_arm_moveit_config, 'launch', 'demo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # Attempt to prevent demo.launch.py from starting another Gazebo instance.
            # The exact name of this argument can vary based on how demo.launch.py is written.
            # Common names are 'launch_gazebo', 'start_gazebo', 'gazebo_simulation'.
            # If this doesn't work, we might need to use a more specific MoveIt launch file
            # like move_group.launch.py and moveit_rviz.launch.py.
            'launch_gazebo': 'false', # Assuming this argument exists in demo.launch.py
            'start_gazebo': 'false', # Another common variant
            'gazebo_simulation': 'false', # Yet another variant
        }.items()
    )
    # If demo.launch.py does not support disabling Gazebo via arguments,
    # and it starts Gazebo, this will lead to a conflict.
    # In that case, one would typically:
    # 1. Modify demo.launch.py to add such an argument.
    # 2. Or, avoid demo.launch.py and launch move_group.launch.py and moveit_rviz.launch.py separately.
    #    Example (if these files exist and are suitable):
    #    move_group_launch = IncludeLaunchDescription(
    #        PythonLaunchDescriptionSource(PathJoinSubstitution(
    #            [pkg_amber_arm_moveit_config, 'launch', 'move_group.launch.py'])),
    #        launch_arguments={'use_sim_time': use_sim_time}.items()
    #    )
    #    rviz_launch = IncludeLaunchDescription(
    #        PythonLaunchDescriptionSource(PathJoinSubstitution(
    #            [pkg_amber_arm_moveit_config, 'launch', 'moveit_rviz.launch.py'])),
    #        # Pass the rviz config file used by demo.launch.py if known
    #        # launch_arguments={'rviz_config': PathJoinSubstitution([pkg_amber_arm_moveit_config, 'launch', 'moveit.rviz'])}.items()
    #    )

    # A more robust way for MoveIt if demo.launch.py is problematic:
    # move_group_launch = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource(PathJoinSubstitution(
    # [pkg_amber_arm_moveit_config, 'launch', 'move_group.launch.py'])),
    # launch_arguments={'use_sim_time': use_sim_time}.items()
    # )
    # moveit_rviz_launch = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource(PathJoinSubstitution(
    # [pkg_amber_arm_moveit_config, 'launch', 'moveit_rviz.launch.py'])),
    # launch_arguments={'use_sim_time': use_sim_time}.items()
    # )


    # 4. Cube Perception Node
    detect_cube_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_cube_perception, 'launch'),
            '/detect_cube.launch.py'
        ]),
        # Parameters for detect_cube_node can be passed here if its launch file supports them
        # launch_arguments={'some_param_for_cube_detection': 'value'}.items()
    )

    # 5. Pick and Place Node
    pick_and_place_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_amber_pick_and_place, 'launch'),
            '/pick_and_place.launch.py'
        ]),
        launch_arguments={
            'place_pose_xyz': place_pose_xyz,
            'home_pose_joints': home_pose_joints,
            # Add other parameters for pick_and_place_node if needed
            # 'offset_above_cube': '0.1',
            # 'grasp_depth_offset': '0.01'
        }.items()
    )
    
    # 6. RViz (if not launched by MoveIt's demo.launch.py)
    # rviz_config_file = PathJoinSubstitution(
    #     [pkg_amber_arm_moveit_config, "config", "moveit.rviz"]
    # )
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     parameters=[{"use_sim_time": use_sim_time}],
    # )


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock?'),
        DeclareLaunchArgument('place_pose_xyz', default_value='[0.0, 0.5, 0.05]', description='Target XYZ for place'),
        DeclareLaunchArgument('home_pose_joints', default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]', description='Home joint configuration'),
        
        gazebo_launch,
        # robot_state_publisher_node_loader, # Disabled, assuming demo.launch.py handles RSP.
        moveit_demo_launch, 
        detect_cube_launch,
        pick_and_place_launch,
    ])
