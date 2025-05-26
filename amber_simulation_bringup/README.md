# Amber Simulation Bringup Package

This package provides the main launch file to start the complete Amber B1 robot arm pick and place simulation.

## Launch File: `complete_pick_and_place.launch.py`

-   **Path:** `launch/complete_pick_and_place.launch.py`
-   **Purpose:** This is the top-level launch file designed to bring up all necessary components for the pick and place simulation in a coordinated manner.

### Components Launched

The `complete_pick_and_place.launch.py` script includes and configures the following:

1.  **Gazebo Simulation Environment:**
    -   Launches Gazebo using `gazebo_ros/launch/gazebo.launch.py`.
    -   Loads the custom world `pick_and_place.world` from the `amber_gazebo` package. This world contains the Amber robot, a RealSense camera, a small red cube, and the `GazeboRosLinkAttacherPlugin`.

2.  **Robot State Publisher & MoveIt! Setup:**
    -   Includes `demo.launch.py` from the `amber_arm_moveit_config` package. This is typically the standard way to launch MoveIt! for a configured robot and is expected to:
        -   Start the `move_group` node for motion planning.
        -   Load the robot's URDF (from `amber_b1_description`) and start the `robot_state_publisher` to publish robot joint states and TF transforms.
        -   Start RViz with the MoveIt! plugins loaded for visualization and interaction.
    -   The launch file attempts to pass arguments (`launch_gazebo:=false`, `start_gazebo:=false`, `gazebo_simulation:=false`) to `demo.launch.py` to prevent it from starting a separate, conflicting Gazebo instance. The success of this depends on `demo.launch.py` supporting these arguments. If conflicts arise, `demo.launch.py` might need modification, or `move_group` and `rviz` might need to be launched individually.

3.  **Cube Perception Node:**
    -   Includes `detect_cube.launch.py` from the `cube_perception` package.
    -   This starts the `detect_cube_node`, which processes point cloud data from the simulated RealSense camera to find the red cube and publish its pose.

4.  **Pick and Place Orchestration Node:**
    -   Includes `pick_and_place.launch.py` from the `amber_pick_and_place` package.
    -   This starts the `pick_and_place_node`, which subscribes to the detected cube's pose, uses MoveIt! for planning, and interacts with the Gazebo link attacher plugin to simulate grasping.

### Parameters

-   **`use_sim_time`**: A global launch argument (default: `true`) passed to relevant nodes to ensure they use Gazebo's simulation clock.
-   **Pick and Place Parameters:**
    -   `place_pose_xyz` (string representing list, default: `'[0.0, 0.5, 0.05]'`): Target XYZ coordinates for placing the cube. Can be overridden when launching.
    -   `home_pose_joints` (string representing list, default: `'[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'`): Home joint configuration for the robot. Can be overridden.
    These are passed to the `amber_pick_and_place` launch file.

## Usage

To run the entire simulation:
```bash
ros2 launch amber_simulation_bringup complete_pick_and_place.launch.py
```
You can also override parameters:
```bash
ros2 launch amber_simulation_bringup complete_pick_and_place.launch.py place_pose_xyz:="'[0.1, 0.55, 0.1]'"
```
(Note the quoting for list-like parameters passed as strings via command line).

This launch file integrates all previously developed components into a single, runnable simulation. Ensure all dependent packages are correctly built and the workspace is sourced before attempting to launch.
