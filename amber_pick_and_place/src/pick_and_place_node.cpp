#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For tf2::doTransform
#include "gazebo_ros_link_attacher/srv/attach.hpp" // Custom service for attaching links

// For M_PI (Math constant PI)
#define _USE_MATH_DEFINES
#include <cmath>


// Static logger for the node
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place_node");

// Constants for MoveIt! and Gazebo model/link names
// These should match the SRDF configuration and Gazebo world file definitions.
const std::string PLANNING_GROUP = "amber_arm";          // MoveIt! planning group for the arm
const std::string END_EFFECTOR_LINK = "link_gripper";    // End-effector link name in SRDF and URDF
const std::string ROBOT_BASE_FRAME = "base_link";        // Base frame of the robot for planning
const std::string ROBOT_MODEL_NAME_IN_GAZEBO = "amber_robot"; // Name of the robot model in Gazebo
const std::string CUBE_MODEL_NAME_IN_GAZEBO = "small_cube";   // Name of the cube model in Gazebo
const std::string CUBE_LINK_NAME_IN_GAZEBO = "link";          // Name of the cube's link in Gazebo

class PickAndPlaceNode : public rclcpp::Node
{
public:
    PickAndPlaceNode() : Node("pick_and_place_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        // Declare and get ROS parameters for configurable values.
        // These define target locations, robot home pose, and operational offsets.
        this->declare_parameter<std::vector<double>>("place_pose_xyz", {0.0, 0.5, 0.1}); // Default place position [x,y,z]
        this->declare_parameter<std::vector<double>>("home_pose_joints", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}); // Default home joint configuration (7 joints)
        this->declare_parameter<double>("offset_above_cube", 0.1); // Vertical offset for pre-grasp/pre-place approach (meters)
        this->declare_parameter<double>("grasp_depth_offset", 0.01); // Small downward offset for grasp to ensure contact (meters)

        place_pose_xyz_ = this->get_parameter("place_pose_xyz").as_double_array();
        home_pose_joints_ = this->get_parameter("home_pose_joints").as_double_array();
        offset_above_cube_ = this->get_parameter("offset_above_cube").as_double();
        grasp_depth_offset_ = this->get_parameter("grasp_depth_offset").as_double();

        // Validate parameter sizes to prevent runtime errors.
        if (place_pose_xyz_.size() != 3) {
            RCLCPP_ERROR(LOGGER, "place_pose_xyz parameter must have 3 values. Using default [0.0, 0.5, 0.1].");
            place_pose_xyz_ = {0.0, 0.5, 0.1};
        }
        if (home_pose_joints_.size() != 7) { // Assuming 7 joints for the 'amber_arm' group
            RCLCPP_ERROR(LOGGER, "home_pose_joints parameter must have 7 values for amber_arm. Using default all zeros.");
            home_pose_joints_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        }

        // Initialize MoveGroupInterface for motion planning.
        // It requires the Node to be passed as a shared_ptr.
        // Using MultiThreadedExecutor in main() is recommended for MoveIt operations.
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<rclcpp::Node>(this), // Node instance
            PLANNING_GROUP                       // Planning group name
        );
        move_group_->setEndEffectorLink(END_EFFECTOR_LINK);     // Specify the end-effector link
        move_group_->setPlanningFrame(ROBOT_BASE_FRAME);        // Set the reference frame for planning

        // Initialize PlanningSceneInterface to interact with the MoveIt planning scene (e.g., add collision objects).
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // Initialize TF2 buffer and listener for transforming poses between different coordinate frames.
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize service clients for the Gazebo link attacher plugin.
        // These services will be used to simulate grasping by creating/destroying a fixed joint in Gazebo.
        attach_client_ = this->create_client<gazebo_ros_link_attacher::srv::Attach>("/gazebo_link_attacher/attach");
        detach_client_ = this->create_client<gazebo_ros_link_attacher::srv::Attach>("/gazebo_link_attacher/detach");

        // Wait for the services to be available to prevent calls to non-existent services.
        while (!attach_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
            RCLCPP_INFO(LOGGER, "Waiting for /gazebo_link_attacher/attach service...");
        }
        while (!detach_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
            RCLCPP_INFO(LOGGER, "Waiting for /gazebo_link_attacher/detach service...");
        }
        RCLCPP_INFO(LOGGER, "Gazebo link attacher services found.");

        // Subscribe to the detected cube's pose.
        cube_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/detected_cube_pose", 10, // Topic name and QoS depth
            std::bind(&PickAndPlaceNode::cube_pose_callback, this, std::placeholders::_1)); // Callback function
        
        // Initialize MoveItVisualTools for Rviz visualizations (optional but helpful for debugging).
        visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            std::shared_ptr<rclcpp::Node>(this), // Node instance
            ROBOT_BASE_FRAME,                    // Base frame for markers
            "rviz_visual_tools"                  // Marker topic namespace
        );
        visual_tools_->deleteAllMarkers();       // Clear previous markers
        visual_tools_->loadRemoteControl();      // Allows step-by-step execution via RViz panel

        RCLCPP_INFO(LOGGER, "PickAndPlaceNode initialized. Waiting for cube pose to start pick and place sequence...");
    }

private:
    // Callback function for handling incoming cube pose messages.
    // This function orchestrates the entire pick and place sequence.
    void cube_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(LOGGER, "Received cube pose in frame '%s'. Starting pick and place sequence.", msg->header.frame_id.c_str());

        geometry_msgs::msg::PoseStamped cube_pose_transformed; // Will hold the cube's pose in the robot's base frame.
        try
        {
            // Transform the received cube pose into the robot's base frame (planning frame).
            // This is crucial as MoveIt plans in a fixed frame (ROBOT_BASE_FRAME).
            if (!tf_buffer_->canTransform(ROBOT_BASE_FRAME, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(1.0))) {
                RCLCPP_ERROR(LOGGER, "TF transform from '%s' to '%s' not available. Aborting sequence.", msg->header.frame_id.c_str(), ROBOT_BASE_FRAME.c_str());
                return;
            }
            cube_pose_transformed = tf_buffer_->transform(*msg, ROBOT_BASE_FRAME);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(LOGGER, "Could not transform cube pose from '%s' to '%s': %s. Aborting sequence.",
                         msg->header.frame_id.c_str(), ROBOT_BASE_FRAME.c_str(), ex.what());
            return;
        }

        RCLCPP_INFO(LOGGER, "Cube pose transformed to '%s': [x:%.2f, y:%.2f, z:%.2f]",
                    cube_pose_transformed.header.frame_id.c_str(),
                    cube_pose_transformed.pose.position.x,
                    cube_pose_transformed.pose.position.y,
                    cube_pose_transformed.pose.position.z);

        // Define key poses for the pick and place sequence.
        geometry_msgs::msg::Pose pre_grasp_pose, grasp_pose, pre_place_pose, place_pose_target;

        // Define a downward-facing orientation for the end-effector for picking and placing.
        // Roll=0, Pitch=PI (180 degrees), Yaw=0. This orients the gripper downwards along the Z-axis of the base_link.
        tf2::Quaternion q_down;
        q_down.setRPY(0, M_PI, 0); 
        geometry_msgs::msg::Quaternion orientation_down = tf2::toMsg(q_down);

        // 1. Pre-grasp pose: Positioned above the detected cube.
        pre_grasp_pose.position.x = cube_pose_transformed.pose.position.x;
        pre_grasp_pose.position.y = cube_pose_transformed.pose.position.y;
        pre_grasp_pose.position.z = cube_pose_transformed.pose.position.z + offset_above_cube_; // Configurable offset
        pre_grasp_pose.orientation = orientation_down;

        // 2. Grasp pose: Positioned at the cube, with a slight Z offset to ensure contact.
        grasp_pose.position.x = cube_pose_transformed.pose.position.x;
        grasp_pose.position.y = cube_pose_transformed.pose.position.y;
        grasp_pose.position.z = cube_pose_transformed.pose.position.z - grasp_depth_offset_; // Small offset for secure grasp
        grasp_pose.orientation = orientation_down;

        // 3. Pre-place pose: Positioned above the target placement location.
        pre_place_pose.position.x = place_pose_xyz_[0]; // Target X from parameters
        pre_place_pose.position.y = place_pose_xyz_[1]; // Target Y from parameters
        pre_place_pose.position.z = place_pose_xyz_[2] + offset_above_cube_; // Target Z + offset
        pre_place_pose.orientation = orientation_down;

        // 4. Place pose: At the target placement location.
        place_pose_target.position.x = place_pose_xyz_[0];
        place_pose_target.position.y = place_pose_xyz_[1];
        place_pose_target.position.z = place_pose_xyz_[2];
        place_pose_target.orientation = orientation_down;
        
        // Visualize these poses in RViz for debugging.
        visual_tools_->publishAxisLabeled(pre_grasp_pose, "1_pre_grasp_pose");
        visual_tools_->publishAxisLabeled(grasp_pose, "2_grasp_pose");
        visual_tools_->publishAxisLabeled(pre_place_pose, "3_pre_place_pose");
        visual_tools_->publishAxisLabeled(place_pose_target, "4_place_pose");
        visual_tools_->trigger(); // Send markers to RViz

        // --- Pick and Place Motion Sequence ---

        // Move to Pre-Grasp: Approach the cube from above.
        if (!move_to_pose(pre_grasp_pose, "Pre-Grasp")) { RCLCPP_ERROR(LOGGER, "Pre-grasp move failed."); return; }
        
        // Move to Grasp: A linear Cartesian move downwards to the grasp pose.
        if (!move_cartesian(grasp_pose, "Grasp (Cartesian)")) { RCLCPP_ERROR(LOGGER, "Grasp (Cartesian) move failed."); return; }

        // Perform Grasp: Call Gazebo service to attach cube to gripper and update MoveIt planning scene.
        call_attach_service(ROBOT_MODEL_NAME_IN_GAZEBO, END_EFFECTOR_LINK, CUBE_MODEL_NAME_IN_GAZEBO, CUBE_LINK_NAME_IN_GAZEBO);
        add_collision_object_to_scene(cube_pose_transformed, true); // Add cube as attached collision object

        // Move to Pre-Place: Lift the cube and move towards the place location.
        if (!move_to_pose(pre_place_pose, "Pre-Place")) { RCLCPP_ERROR(LOGGER, "Pre-place move failed."); return; }
        
        // Move to Place: A linear Cartesian move downwards to the place pose.
        if (!move_cartesian(place_pose_target, "Place (Cartesian)")) { RCLCPP_ERROR(LOGGER, "Place (Cartesian) move failed."); return; }

        // Perform Release: Call Gazebo service to detach cube and update MoveIt planning scene.
        call_detach_service(ROBOT_MODEL_NAME_IN_GAZEBO, END_EFFECTOR_LINK, CUBE_MODEL_NAME_IN_GAZEBO, CUBE_LINK_NAME_IN_GAZEBO);
        add_collision_object_to_scene(cube_pose_transformed, false); // Remove cube as attached collision object

        // Move to Home: Return the robot to its home configuration.
        if (!move_to_home()) { RCLCPP_ERROR(LOGGER, "Move to home failed."); return; }

        RCLCPP_INFO(LOGGER, "Pick and place sequence completed successfully.");
    }

    // Helper function to plan and execute a Point-to-Point (PTP) motion to a target pose.
    bool move_to_pose(const geometry_msgs::msg::Pose &target_pose, const std::string& pose_name)
    {
        RCLCPP_INFO(LOGGER, "Moving to %s pose.", pose_name.c_str());
        move_group_->setPoseTarget(target_pose); // Set the desired pose for the end-effector.
        
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // Attempt to plan the motion.
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(LOGGER, "Planning to %s successful. Executing...", pose_name.c_str());
            // Visualize the planned trajectory in RViz.
            visual_tools_->publishTrajectoryLine(my_plan.trajectory_, move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP));
            visual_tools_->trigger();
            // visual_tools_->prompt("Press 'next' in RViz to execute planned path"); // Uncomment for manual step-through
            
            // Execute the planned motion.
            return (move_group_->execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        else
        {
            RCLCPP_ERROR(LOGGER, "Planning to %s failed.", pose_name.c_str());
            return false;
        }
    }
    
    // Helper function to plan and execute a Cartesian (linear) motion to a target pose.
    // Useful for direct approach/retract movements like grasping or placing.
    bool move_cartesian(const geometry_msgs::msg::Pose &target_pose, const std::string& pose_name)
    {
        RCLCPP_INFO(LOGGER, "Moving to %s pose (Cartesian).", pose_name.c_str());
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose); // Only one waypoint: the target pose itself.

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0; // Disable jump threshold (allows large joint space jumps if necessary for Cartesian path)
        const double eef_step = 0.01;      // Interpolation step size for the Cartesian path (e.g., 1cm)

        // Compute the Cartesian path. This generates a trajectory that moves the end-effector linearly.
        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction >= 0.9) // Consider successful if at least 90% of the path is planned.
        {
            RCLCPP_INFO(LOGGER, "Cartesian path to %s planned (%.2f%% complete). Executing...", pose_name.c_str(), fraction * 100.0);
            // Visualize the Cartesian trajectory.
            visual_tools_->publishTrajectoryLine(trajectory, move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP));
            visual_tools_->trigger();
            // visual_tools_->prompt("Press 'next' in RViz to execute Cartesian path"); // Uncomment for manual step-through
            
            // Execute the planned Cartesian path.
            // Note: computeCartesianPath returns a RobotTrajectory, which needs to be put into a Plan object for execution.
            moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
            cartesian_plan.trajectory_ = trajectory;
            return (move_group_->execute(cartesian_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        else
        {
            RCLCPP_ERROR(LOGGER, "Planning Cartesian path to %s failed (fraction: %.2f).", pose_name.c_str(), fraction);
            return false;
        }
    }

    // Helper function to move the robot to its predefined home joint configuration.
    bool move_to_home()
    {
        RCLCPP_INFO(LOGGER, "Moving to home pose.");
        move_group_->setJointValueTarget(home_pose_joints_); // Set target by joint values.
        
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(LOGGER, "Planning to home pose successful. Executing...");
            visual_tools_->publishTrajectoryLine(my_plan.trajectory_, move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP));
            visual_tools_->trigger();
            // visual_tools_->prompt("Press 'next' in RViz to move to home"); // Uncomment for manual step-through
            return (move_group_->execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        else
        {
            RCLCPP_ERROR(LOGGER, "Planning to home pose failed.");
            return false;
        }
    }

    // Calls the Gazebo service to attach two links.
    void call_attach_service(const std::string& model1, const std::string& link1, const std::string& model2, const std::string& link2) {
        auto request = std::make_shared<gazebo_ros_link_attacher::srv::Attach::Request>();
        request->model_name_1 = model1; // e.g., "amber_robot"
        request->link_name_1 = link1;   // e.g., "link_gripper"
        request->model_name_2 = model2; // e.g., "small_cube"
        request->link_name_2 = link2;   // e.g., "link"

        // Asynchronously send the request and wait for the response.
        // Using spin_until_future_complete to make the call synchronous within this function.
        auto future = attach_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(5)) == // Timeout of 5s
            rclcpp::FutureReturnCode::SUCCESS)
        {
            if (future.get()->success) {
                RCLCPP_INFO(LOGGER, "Successfully attached %s:%s to %s:%s via Gazebo service.", model1.c_str(), link1.c_str(), model2.c_str(), link2.c_str());
            } else {
                RCLCPP_ERROR(LOGGER, "Failed to attach via Gazebo service: %s", future.get()->message.c_str());
            }
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to call /gazebo_link_attacher/attach service (timeout or other error).");
        }
    }

    // Calls the Gazebo service to detach two links.
    void call_detach_service(const std::string& model1, const std::string& link1, const std::string& model2, const std::string& link2) {
        auto request = std::make_shared<gazebo_ros_link_attacher::srv::Attach::Request>();
        request->model_name_1 = model1;
        request->link_name_1 = link1;
        request->model_name_2 = model2;
        request->link_name_2 = link2;

        auto future = detach_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(5)) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
             if (future.get()->success) {
                RCLCPP_INFO(LOGGER, "Successfully detached %s:%s from %s:%s via Gazebo service.", model1.c_str(), link1.c_str(), model2.c_str(), link2.c_str());
            } else {
                RCLCPP_ERROR(LOGGER, "Failed to detach via Gazebo service: %s", future.get()->message.c_str());
            }
        } else {
            RCLCPP_ERROR(LOGGER, "Failed to call /gazebo_link_attacher/detach service (timeout or other error).");
        }
    }
    
    // Adds or removes the detected cube as a collision object in the MoveIt planning scene.
    // Also attaches/detaches it from the robot's end-effector in the planning scene.
    void add_collision_object_to_scene(const geometry_msgs::msg::PoseStamped& cube_pose, bool attach) {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = cube_pose.header.frame_id; // Should be ROBOT_BASE_FRAME after transformation
        collision_object.id = CUBE_MODEL_NAME_IN_GAZEBO; // Unique ID for this collision object

        // Define the primitive shape (a box for the cube)
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.05; // Cube dimensions (match Gazebo model)
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.05;
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.05;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(cube_pose.pose); // Pose of the cube
        
        if (attach) {
            // Add the collision object to the world
            collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
            planning_scene_interface_->applyCollisionObject(collision_object);
            
            // Attach the collision object to the robot's end-effector link in the planning scene.
            // This tells MoveIt that the object will move with the end-effector and should be
            // considered in collision checking for the robot itself (e.g., arm colliding with attached object).
            move_group_->attachObject(collision_object.id, END_EFFECTOR_LINK);
            RCLCPP_INFO(LOGGER, "Attached '%s' to '%s' in MoveIt planning scene.", collision_object.id.c_str(), END_EFFECTOR_LINK.c_str());
        } else {
            // Detach the object from the robot's end-effector in the planning scene.
            move_group_->detachObject(collision_object.id);
            
            // Remove the collision object from the world.
            collision_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
            planning_scene_interface_->applyCollisionObject(collision_object);
            RCLCPP_INFO(LOGGER, "Detached and removed '%s' from MoveIt planning scene.", collision_object.id.c_str());
        }
    }

    // ROS 2 and MoveIt! member variables
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cube_pose_subscriber_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

    // TF2 components
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Service clients for Gazebo link attacher
    rclcpp::Client<gazebo_ros_link_attacher::srv::Attach>::SharedPtr attach_client_;
    rclcpp::Client<gazebo_ros_link_attacher::srv::Attach>::SharedPtr detach_client_;

    // Node parameters
    std::vector<double> place_pose_xyz_;
    std::vector<double> home_pose_joints_;
    double offset_above_cube_;
    double grasp_depth_offset_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // Using a MultiThreadedExecutor allows MoveGroupInterface to spin in its own thread if needed,
    // or handle callbacks concurrently. For this node, SingleThreadedExecutor might be okay
    // as the callback does most of the work sequentially. However, MGI often benefits from its own thread.
    // Let's start with SingleThreaded for simplicity and see if it causes issues.
    // If move_group_->plan() or execute() block indefinitely or cause issues with subscription,
    // then MultiThreadedExecutor or a dedicated thread for MGI is needed.
    // For MoveIt, it's generally recommended to use a MultiThreadedExecutor.
    rclcpp::executors::MultiThreadedExecutor executor;
    auto pick_and_place_node = std::make_shared<PickAndPlaceNode>();
    executor.add_node(pick_and_place_node);
    executor.spin();
    // rclcpp::spin(std::make_shared<PickAndPlaceNode>());
    rclcpp::shutdown();
    return 0;
}
