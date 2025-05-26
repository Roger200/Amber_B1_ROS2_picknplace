# RealSense D435 Camera Description Package

This package provides the Gazebo model for a simulated Intel RealSense D435 RGB-D camera.

## Contents

-   **`sdf/realsense_d435.sdf`**:
    -   Defines the camera model using the SDF (Simulation Description Format).
    -   Includes the camera's visual geometry (a simple box representing the D435).
    -   Defines the depth camera sensor properties:
        -   Horizontal Field of View (FOV)
        -   Image dimensions (width, height)
        -   Image format
        -   Clipping planes (near and far)
        -   Update rate
    -   The SDF is structured to be included in a Gazebo world file. The Gazebo ROS camera plugin (`libgazebo_ros_camera.so`) is typically configured in the world file where this model is included, rather than directly in this SDF. This allows for more flexible configuration of ROS topics and namespaces.

## Usage

This model is intended to be included in Gazebo world files. For example, in a `.world` file:

```xml
<model name='realsense_camera'>
  <include>
    <uri>model://realsense_d435_description</uri> <!-- Or relative path like ../../realsense_d435_description -->
  </include>
  <pose>0 0 1 0 0 0</pose> <!-- Example pose -->
  <!-- The Gazebo ROS camera plugin is typically configured here -->
  <plugin name='camera_plugin_world' filename='libgazebo_ros_camera.so'>
    <ros>
      <namespace>/camera</namespace>
      <argument>--ros-args -r image_raw:=color/image_raw -r depth/image_raw:=depth/image_rect_raw -r camera_info:=color/camera_info -r depth/camera_info:=depth/camera_info -r points:=depth/color/points</argument>
    </ros>
    <camera_name>realsense_d435_camera_sensor_sdf</camera_name> <!-- Matches camera name in SDF -->
    <frame_name>camera_depth_optical_frame</frame_name>
    <!-- Other plugin parameters -->
  </plugin>
</model>
```

Ensure that the `GAZEBO_MODEL_PATH` environment variable includes the path to this package so Gazebo can find it using the `model://` URI, or use a relative path from the world file to this package's directory. Sourcing your ROS 2 workspace usually handles this.

In this project, `realsense_d435_description` is used in `amber_gazebo/worlds/pick_and_place.world`.
