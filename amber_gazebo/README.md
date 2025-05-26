# Amber Gazebo Package

This package contains Gazebo-specific files for the Amber B1 robot arm pick and place simulation.

## Contents

### World File

-   **`worlds/pick_and_place.world`**:
    -   Defines the Gazebo simulation environment.
    -   Includes a ground plane.
    -   Includes a small red cube (`small_cube` model) with dimensions 0.05m x 0.05m x 0.05m.
    -   Includes the Amber B1 robot arm (`amber_robot` model), loaded from the `amber_b1_description` package.
    -   Includes the RealSense D435 camera (`realsense_camera` model), loaded from the `realsense_d435_description` package. The camera is attached to the robot's end-effector (`link_eef`) via a fixed joint defined in this world file.
    -   Configures the `libgazebo_ros_camera.so` plugin for the RealSense camera to publish color images, depth images, and point cloud data (topic: `/camera/depth/color/points`).
    -   **Loads the `GazeboRosLinkAttacherPlugin`**: This world plugin (from the `gazebo_ros_link_attacher` package) provides ROS 2 services to dynamically create and destroy fixed joints between links in Gazebo, enabling the simulation of grasping. The plugin is loaded via the line:
        ```xml
        <plugin name="gazebo_ros_link_attacher" filename="libgazebo_ros_link_attacher.so">
        </plugin>
        ```

### Launch File

-   **`launch/pick_and_place_world.launch.py`**:
    -   A simple launch file to start Gazebo with the `pick_and_place.world`.
    -   It uses `gazebo_ros/launch/gazebo.launch.py` and passes the path to `pick_and_place.world` as an argument.

### Models (if any)

This package may also contain custom Gazebo models if they are not part of other dedicated description packages. (Currently, the robot and camera are in their own description packages, and the cube is defined directly in the world file or could be a simple SDF here).

## Usage

The `pick_and_place.world` is typically launched via the main simulation bringup launch file in the `amber_simulation_bringup` package.
To launch it standalone for testing:
```bash
ros2 launch amber_gazebo pick_and_place_world.launch.py
```
Ensure that the `GAZEBO_MODEL_PATH` includes the paths to `amber_b1_description` and `realsense_d435_description` packages, which is usually handled by sourcing your ROS 2 workspace. The plugin `libgazebo_ros_link_attacher.so` must also be findable by Gazebo (its package built and sourced).
