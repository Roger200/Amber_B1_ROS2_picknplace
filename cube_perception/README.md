# Cube Perception Package

This package provides a ROS 2 node for detecting a red cube in a 3D point cloud.

## Node: `detect_cube_node`

The `detect_cube_node` processes point cloud data to find a red cube and publish its pose.

### Subscribed Topics

-   **`/camera/depth/color/points`** (`sensor_msgs/PointCloud2`):
    Input point cloud data, expected to be an XYZRGB point cloud.

### Published Topics

-   **`/detected_cube_pose`** (`geometry_msgs/PoseStamped`):
    The estimated pose (position and orientation) of the detected red cube. The orientation is currently fixed (no rotation relative to the input cloud's frame). The pose is published in the same frame as the input point cloud.

### Functionality

The node implements the following Point Cloud Library (PCL) based processing pipeline:

1.  **Passthrough Filter:** Filters points outside a defined X, Y, Z range to focus on a region of interest.
2.  **Voxel Grid Downsampling:** Reduces the number of points using a voxel grid, which helps in speeding up processing and noise reduction.
3.  **Color Segmentation:** Filters points based on their RGB values to isolate points that fall within a predefined "red" color range.
4.  **Statistical Outlier Removal:** Removes sparse outliers from the color-segmented point cloud.
5.  **Euclidean Clustering:** Groups the remaining points into distinct spatial clusters.
6.  **Cube Identification:** Assumes the largest cluster found is the target red cube.
7.  **Pose Calculation:** Calculates the centroid of the largest cluster to determine the cube's position. The orientation is set to a default (identity quaternion).
8.  **Publication:** Publishes the calculated pose as a `geometry_msgs/PoseStamped` message.

### Parameters

The node exposes several ROS 2 parameters to tune the detection pipeline (default values are set in the C++ node):

-   **Passthrough Filter:**
    -   `passthrough_x_min`, `passthrough_x_max` (double, meters): X-axis limits.
    -   `passthrough_y_min`, `passthrough_y_max` (double, meters): Y-axis limits.
    -   `passthrough_z_min`, `passthrough_z_max` (double, meters): Z-axis limits.
-   **Voxel Grid Filter:**
    -   `voxel_leaf_size` (double, meters): Leaf size for the voxel grid (e.g., 0.01 for 1cm).
-   **Color Segmentation (for Red):**
    -   `color_r_min`, `color_r_max` (int, 0-255): Red channel range.
    -   `color_g_min`, `color_g_max` (int, 0-255): Green channel range.
    -   `color_b_min`, `color_b_max` (int, 0-255): Blue channel range.
-   **Statistical Outlier Removal:**
    -   `sor_mean_k` (int): Number of nearest neighbors to analyze.
    -   `sor_stddev_mul_thresh` (double): Standard deviation multiplier threshold.
-   **Euclidean Clustering:**
    -   `cluster_tolerance` (double, meters): Maximum distance between points to be considered in the same cluster.
    -   `cluster_min_size` (int): Minimum number of points for a valid cluster.
    -   `cluster_max_size` (int): Maximum number of points for a valid cluster.

These parameters can be adjusted in the `detect_cube.launch.py` file or via the command line when launching the node.

### Launch File

-   **`launch/detect_cube.launch.py`**:
    -   A simple launch file to start the `detect_cube_node`.
    -   Allows overriding of the declared ROS parameters.

## Usage

This node is typically launched as part of a larger simulation or robot application. It can be launched standalone for testing:
```bash
ros2 launch cube_perception detect_cube.launch.py
```
Ensure that a point cloud is being published to `/camera/depth/color/points` for the node to process.
