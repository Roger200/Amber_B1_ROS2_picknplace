# Amber Pick and Place Package

This package contains the ROS 2 node responsible for orchestrating the pick and place task using the Amber B1 robot arm and MoveIt!.

## Node: `pick_and_place_node`

The `pick_and_place_node` controls the robot arm to pick up a detected cube and place it at a predefined location.

### Subscribed Topics

-   **`/detected_cube_pose`** (`geometry_msgs/PoseStamped`):
    Receives the pose of the cube to be picked up, as detected by the `cube_perception` node. This pose is expected to be in a frame that can be transformed to the robot's base frame (`base_link`).

### Published Topics

-   (Indirectly through MoveIt!) Various topics related to robot motion planning and execution, such as `/display_planned_path`, `/execute_trajectory`, etc.
-   (Indirectly through `moveit_visual_tools`) RViz markers for visualizing target poses and trajectories on the topic `rviz_visual_tools`.

### Services Used

-   **`/gazebo_link_attacher/attach`** (`gazebo_ros_link_attacher/srv/Attach`):
    Calls this service to simulate grasping by creating a fixed joint in Gazebo between the robot's end-effector link (`link_gripper`) and the cube's link (`link`).
-   **`/gazebo_link_attacher/detach`** (`gazebo_ros_link_attacher/srv/Attach`):
    Calls this service to simulate releasing by removing the fixed joint in Gazebo.

### Functionality

1.  **Initialization:**
    -   Initializes `moveit::planning_interface::MoveGroupInterface` for the `amber_arm` planning group.
    -   Sets the end-effector link to `link_gripper` and the planning frame to `base_link`.
    -   Initializes `moveit::planning_interface::PlanningSceneInterface` to manage collision objects.
    -   Sets up TF2 listener to transform cube poses.
    -   Creates service clients for the Gazebo link attacher services.
    -   Initializes `moveit_visual_tools` for RViz visualization.

2.  **Cube Pose Callback (`cube_pose_callback`):**
    -   Triggered when a new cube pose is received.
    -   **Transforms Cube Pose:** Transforms the received pose from its original frame to `ROBOT_BASE_FRAME` (`base_link`).
    -   **Defines Target Poses:** Calculates key poses for the pick and place sequence:
        -   `pre_grasp_pose`: Above the cube.
        -   `grasp_pose`: At the cube, slightly lowered for grasp.
        -   `pre_place_pose`: Above the target placement location.
        -   `place_pose_target`: At the target placement location.
        All these poses use a fixed downward-facing orientation for the end-effector (roll=0, pitch=PI, yaw=0 relative to `base_link`).
    -   **Executes Motion Sequence:**
        1.  Moves to `pre_grasp_pose` (Point-to-Point motion).
        2.  Moves to `grasp_pose` (Cartesian linear motion).
        3.  **Grasps Cube:**
            -   Calls the `/gazebo_link_attacher/attach` service.
            -   Adds the cube as a collision object to the MoveIt! planning scene and attaches it to the end-effector (`add_collision_object_to_scene(cube_pose, true)`).
        4.  Moves to `pre_place_pose` (PTP motion, with cube attached).
        5.  Moves to `place_pose_target` (Cartesian linear motion, with cube attached).
        6.  **Releases Cube:**
            -   Calls the `/gazebo_link_attacher/detach` service.
            -   Detaches the cube from the end-effector in the MoveIt! planning scene and removes the collision object (`add_collision_object_to_scene(cube_pose, false)`).
        7.  Moves to a predefined `home_pose_joints` (PTP motion).
    -   Uses helper functions `move_to_pose` and `move_cartesian` for planning and execution.

### Parameters

The node exposes the following ROS 2 parameters:

-   `place_pose_xyz` (vector of double, meters, default: `{0.0, 0.5, 0.1}`): The target [X, Y, Z] coordinates for placing the cube in the `ROBOT_BASE_FRAME`.
-   `home_pose_joints` (vector of double, radians, default: `{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}`): The joint angles for the robot's home/neutral pose (assuming 7 joints for `amber_arm`).
-   `offset_above_cube` (double, meters, default: `0.1`): Vertical offset used for pre-grasp and pre-place approach heights.
-   `grasp_depth_offset` (double, meters, default: `0.01`): Small downward offset added at the grasp pose to ensure the gripper reaches the cube.

### Launch File

-   **`launch/pick_and_place.launch.py`**:
    -   Launches the `pick_and_place_node`.
    -   Allows overriding the declared ROS parameters via launch arguments.

## Usage

This node is designed to be launched as part of the main simulation sequence. It can be launched standalone for testing if the required services and topics (MoveIt!, TF, detected cube pose) are available:
```bash
# Example: Override place_pose_xyz
ros2 launch amber_pick_and_place pick_and_place.launch.py place_pose_xyz:="[0.1, 0.6, 0.15]"
```
