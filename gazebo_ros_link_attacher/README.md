# Gazebo ROS Link Attacher Package

This package provides a Gazebo WorldPlugin that allows dynamic attachment and detachment of links belonging to different models within the Gazebo simulation environment. This is achieved by creating or removing a fixed joint between the specified links via ROS 2 services.

This functionality is particularly useful for simulating grasping or docking operations where one model (e.g., a robot gripper) needs to rigidly connect to another model (e.g., an object to be picked up).

## Plugin: `GazeboRosLinkAttacherPlugin`

-   **Type:** Gazebo WorldPlugin (`gazebo::WorldPlugin`).
-   **Functionality:**
    -   Initializes a ROS 2 node within Gazebo.
    -   Advertises two ROS 2 services:
        -   `/gazebo_link_attacher/attach`
        -   `/gazebo_link_attacher/detach`
    -   These services use the custom service type `gazebo_ros_link_attacher/srv/Attach`.

### Service Definition (`srv/Attach.srv`)

The `Attach.srv` file defines the request and response structure for the services:

**Request:**
-   `string model_name_1`: Name of the first model (e.g., the robot).
-   `string link_name_1`: Name of the link in the first model (e.g., the gripper link).
-   `string model_name_2`: Name of the second model (e.g., the object to be grasped).
-   `string link_name_2`: Name of the link in the second model (e.g., the object's base link).

**Response:**
-   `bool success`: `true` if the attach/detach operation was successful, `false` otherwise.
-   `string message`: An optional message providing details about the operation's result.

### How it Works

-   **Attach:** When the `/gazebo_link_attacher/attach` service is called:
    1.  The plugin finds the specified models and links in the Gazebo world.
    2.  It constructs a unique name for the joint to be created (e.g., `model1_link1_to_model2_link2_fixed_joint`).
    3.  If a joint with this name already exists, it attempts to remove it first to ensure a clean state.
    4.  It then creates a new "fixed" type joint using `world->Physics()->CreateJoint("fixed", model1_ptr)`.
    5.  The joint is loaded with `link1` (from `model1`) as the parent and `link2` (from `model2`) as the child. An identity pose is used, meaning `link2` is fixed relative to `link1` at their current spatial relationship.
    6.  The joint is initialized using `joint->Init()`, which activates it in the simulation.

-   **Detach:** When the `/gazebo_link_attacher/detach` service is called:
    1.  The plugin reconstructs the unique joint name based on the request parameters.
    2.  It attempts to find this joint in the Gazebo world.
    3.  If found, it removes the joint from the model it's associated with (typically `model1` as per the attach logic) using `model_ptr->RemoveJoint(joint_name)`.

## Usage

1.  **Build:** Ensure this package (`gazebo_ros_link_attacher`) is built as part of your ROS 2 workspace.
2.  **Load in Gazebo:** Include the plugin in your Gazebo world (`.world`) file:
    ```xml
    <sdf version='1.7'> <!-- Or your SDF version -->
      <world name='default'>
        <!-- ... other world contents ... -->

        <plugin name="gazebo_ros_link_attacher" filename="libgazebo_ros_link_attacher.so">
            <!-- Optional: Add any specific SDF parameters for the plugin here if needed -->
        </plugin>
      </world>
    </sdf>
    ```
    The `filename` should be `lib<package_name>.so`, so `libgazebo_ros_link_attacher.so`. Gazebo needs to be able to find this plugin library (usually handled by sourcing your ROS 2 workspace).

3.  **Call Services from ROS 2 Nodes:**
    Create ROS 2 service clients in your C++ or Python nodes to call `/gazebo_link_attacher/attach` and `/gazebo_link_attacher/detach` as needed. Refer to the `amber_pick_and_place_node.cpp` in the `amber_pick_and_place` package for an example of how to use these service clients.

## Important Considerations

-   **Model and Link Names:** The model and link names provided in the service request must exactly match those in the Gazebo simulation.
-   **Joint Uniqueness:** The plugin generates a deterministic joint name. If multiple attachments are needed between the same pair of links without intermediate detaches, the plugin currently handles this by removing the old joint before creating a new one.
-   **Plugin Loading:** Ensure the Gazebo environment can find the compiled plugin (e.g., `LD_LIBRARY_PATH` or `GAZEBO_PLUGIN_PATH` should include the `lib` directory of this package's installation space, typically handled by `source install/setup.bash`). The `<export><gazebo_ros gazebo_model_path="${prefix}/../../.." /></export>` in `package.xml` might also help Gazebo locate plugins from ROS packages, though explicitly setting paths or ensuring `colcon` handles it is key.

This package is used in the Amber Arm Pick and Place Simulation to simulate the robot grasping the cube.
