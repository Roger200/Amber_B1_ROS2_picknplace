#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo_ros/node.hpp>
#include "rclcpp/rclcpp.hpp"
#include "gazebo_ros_link_attacher/srv/attach.hpp" // Custom service

#include <ignition/math/Pose3.hh>

namespace gazebo_plugins
{

class GazeboRosLinkAttacherPlugin : public gazebo::WorldPlugin
{
public:
    GazeboRosLinkAttacherPlugin() : WorldPlugin() {}

    void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override
    {
        this->world_ = _world;
        this->sdf_ = _sdf;

        // Initialize ROS node
        if (!rclcpp::is_initialized())
        {
            rclcpp::init(0, nullptr);
        }
        this->ros_node_ = gazebo_ros::Node::Get(this->sdf_);
        
        RCLCPP_INFO(this->ros_node_->get_logger(), "Loading GazeboRosLinkAttacherPlugin.");

        // Create services for attaching and detaching links.
        // The services use a custom type `gazebo_ros_link_attacher::srv::Attach`.
        attach_service_ = this->ros_node_->create_service<gazebo_ros_link_attacher::srv::Attach>(
            "/gazebo_link_attacher/attach", // ROS 2 service name
            std::bind(&GazeboRosLinkAttacherPlugin::AttachCallback, this,
                      std::placeholders::_1, std::placeholders::_2)); // Callback function

        detach_service_ = this->ros_node_->create_service<gazebo_ros_link_attacher::srv::Attach>(
            "/gazebo_link_attacher/detach", // ROS 2 service name
            std::bind(&GazeboRosLinkAttacherPlugin::DetachCallback, this,
                      std::placeholders::_1, std::placeholders::_2)); // Callback function
        
        RCLCPP_INFO(this->ros_node_->get_logger(), "GazeboRosLinkAttacherPlugin services advertised: /gazebo_link_attacher/attach and /gazebo_link_attacher/detach");
    }

private:
    // Service callback for attaching two links.
    void AttachCallback(
        const std::shared_ptr<gazebo_ros_link_attacher::srv::Attach::Request> req,
        std::shared_ptr<gazebo_ros_link_attacher::srv::Attach::Response> res)
    {
        RCLCPP_INFO(this->ros_node_->get_logger(), "Attach request received for %s:%s and %s:%s",
                    req->model_name_1.c_str(), req->link_name_1.c_str(),
                    req->model_name_2.c_str(), req->link_name_2.c_str());

        gazebo::physics::ModelPtr model1 = this->world_->ModelByName(req->model_name_1);
        gazebo::physics::ModelPtr model2 = this->world_->ModelByName(req->model_name_2);

        if (!model1) {
            RCLCPP_ERROR(this->ros_node_->get_logger(), "Model1 [%s] not found.", req->model_name_1.c_str());
            res->success = false;
            res->message = "Model1 not found";
            return;
        }
        if (!model2) {
            RCLCPP_ERROR(this->ros_node_->get_logger(), "Model2 [%s] not found.", req->model_name_2.c_str());
            res->success = false;
            res->message = "Model2 not found";
            return;
        }

        gazebo::physics::LinkPtr link1 = model1->GetLink(req->link_name_1);
        gazebo::physics::LinkPtr link2 = model2->GetLink(req->link_name_2);

        // Validate links
        if (!link1) {
            RCLCPP_ERROR(this->ros_node_->get_logger(), "Link1 [%s] in Model1 [%s] not found.", req->link_name_1.c_str(), req->model_name_1.c_str());
            res->success = false;
            res->message = "Link1 not found in Model1";
            return;
        }
        if (!link2) {
            RCLCPP_ERROR(this->ros_node_->get_logger(), "Link2 [%s] in Model2 [%s] not found.", req->link_name_2.c_str(), req->model_name_2.c_str());
            res->success = false;
            res->message = "Link2 not found in Model2";
            return;
        }
        
        // Construct a unique joint name to prevent conflicts and allow for later detachment.
        joint_name_ = req->model_name_1 + "_" + req->link_name_1 + "_to_" + req->model_name_2 + "_" + req->link_name_2 + "_fixed_joint";
        
        // Check if a joint with this name already exists.
        // This can happen if a previous attach call was made without a corresponding detach,
        // or if the simulation state is reloaded.
        if (this->world_->JointByName(joint_name_)) {
             RCLCPP_WARN(this->ros_node_->get_logger(), "Joint [%s] already exists. Detaching old one before creating new.", joint_name_.c_str());
             // Attempt to remove the existing joint to ensure a clean state.
             // If DetachJoint fails to remove it, we might proceed with a warning or error out.
             // For robustness, it's better to ensure the old one is gone.
             if (!DetachJoint(joint_name_)) { // DetachJoint returns true if successfully removed or not found
                RCLCPP_ERROR(this->ros_node_->get_logger(), "Failed to remove existing joint [%s]. Attach operation aborted.", joint_name_.c_str());
                res->success = false;
                res->message = "Failed to remove pre-existing joint.";
                return;
             }
        }

        RCLCPP_INFO(this->ros_node_->get_logger(), "Creating fixed joint [%s] between %s:%s and %s:%s",
                    joint_name_.c_str(),
                    req->model_name_1.c_str(), req->link_name_1.c_str(),
                    req->model_name_2.c_str(), req->link_name_2.c_str());

        // Create a new fixed joint using the world's physics engine.
        // The joint is initially associated with model1.
        gazebo::physics::JointPtr new_joint = this->world_->Physics()->CreateJoint("fixed", model1); 
        
        if (!new_joint) {
            RCLCPP_ERROR(this->ros_node_->get_logger(), "Physics engine failed to create a new fixed joint.");
            res->success = false;
            res->message = "Physics engine failed to create joint.";
            return;
        }

        new_joint->SetName(joint_name_); // Set the unique name for the joint.
        
        // Load the joint with the parent (link1) and child (link2) links.
        // The pose of the joint is specified in the child link's frame.
        // For a fixed joint, an identity pose means link2 will be fixed relative to link1
        // at their current relative positions and orientations.
        ignition::math::Pose3d relative_pose; // Default (identity) pose.
        new_joint->Load(link1, link2, relative_pose);
        
        // Initialize the joint. This step actually connects the links and activates the joint in the simulation.
        new_joint->Init(); 

        RCLCPP_INFO(this->ros_node_->get_logger(), "Successfully created and initialized joint: %s", joint_name_.c_str());
        res->success = true;
        res->message = "Links attached successfully.";
    }
    
    // Helper function to detach a joint by name.
    // Returns true if the joint was found and successfully removed, or if it was not found (already detached).
    // Returns false if the joint was found but could not be removed.
    bool DetachJoint(const std::string& name_of_joint_to_detach) {
        gazebo::physics::JointPtr existing_joint = this->world_->JointByName(name_of_joint_to_detach);
        
        if (existing_joint) {
            RCLCPP_INFO(this->ros_node_->get_logger(), "Found joint [%s] to detach.", name_of_joint_to_detach.c_str());
            
            // A common way to remove a joint is to remove it from the model that owns it.
            // Joints created with world_->Physics()->CreateJoint("type", model_ptr) are owned by that model_ptr.
            // We need to determine which model was used as the first argument to CreateJoint.
            // In our AttachCallback, 'model1' was used.
            
            // Get the parent and child models from the joint's links
            gazebo::physics::ModelPtr parent_model = existing_joint->GetParent()->GetModel();
            gazebo::physics::ModelPtr child_model = existing_joint->GetChild()->GetModel();

            bool removed = false;
            // Try removing from parent model first if it exists
            if (parent_model) {
                // Check if the joint is listed in the parent model's joints
                if (parent_model->GetJoint(name_of_joint_to_detach)) {
                    parent_model->RemoveJoint(name_of_joint_to_detach);
                    RCLCPP_INFO(this->ros_node_->get_logger(), "Joint [%s] removed from parent model [%s].", name_of_joint_to_detach.c_str(), parent_model->GetName().c_str());
                    removed = true;
                }
            }
            
            // If not removed and child model is different, try removing from child model
            // This case is less common for world-created joints but could be a fallback.
            if (!removed && child_model && child_model != parent_model) {
                 if (child_model->GetJoint(name_of_joint_to_detach)) {
                    child_model->RemoveJoint(name_of_joint_to_detach);
                    RCLCPP_INFO(this->ros_node_->get_logger(), "Joint [%s] removed from child model [%s].", name_of_joint_to_detach.c_str(), child_model->GetName().c_str());
                    removed = true;
                 }
            }

            if (!removed) {
                // If the joint still exists (e.g., not owned by either model directly in a way RemoveJoint works,
                // or if the joint was created differently), this indicates a problem or a more complex ownership.
                // Gazebo's joint removal can sometimes be tricky depending on how it was created.
                // For joints created with world->Physics()->CreateJoint(type, model),
                // model->RemoveJoint(name) is the standard way.
                // If we assumed model1 was the owner:
                // gazebo::physics::ModelPtr model1_assumed_owner = world_->ModelByName(existing_joint->GetParent()->GetModel()->GetName()); // Re-fetch model1 if needed
                // model1_assumed_owner->RemoveJoint(name_of_joint_to_detach);
                RCLCPP_WARN(this->ros_node_->get_logger(), "Joint [%s] was found but could not be removed from its presumed parent model(s). Manual check might be needed.", name_of_joint_to_detach.c_str());
                // Consider if joint->Detach() or joint->Fini() are more appropriate, though Model::RemoveJoint is typical.
                return false; // Indicates failure to remove an existing joint.
            }
            return true; // Successfully removed
        } else {
            RCLCPP_INFO(this->ros_node_->get_logger(), "Joint [%s] not found. Assuming already detached.", name_of_joint_to_detach.c_str());
            return true; // Joint doesn't exist, so it's effectively "detached".
        }
    }

    // Service callback for detaching two links.
    // This essentially means removing the fixed joint that was created between them.
    void DetachCallback(
        const std::shared_ptr<gazebo_ros_link_attacher::srv::Attach::Request> req,
        std::shared_ptr<gazebo_ros_link_attacher::srv::Attach::Response> res)
    {
        RCLCPP_INFO(this->ros_node_->get_logger(), "Detach request for joint between %s:%s and %s:%s",
                    req->model_name_1.c_str(), req->link_name_1.c_str(),
                    req->model_name_2.c_str(), req->link_name_2.c_str());

        // Reconstruct the unique joint name based on the request parameters.
        // This must match the name used during the attach operation.
        std::string name_of_joint_to_detach = req->model_name_1 + "_" + req->link_name_1 + "_to_" + req->model_name_2 + "_" + req->link_name_2 + "_fixed_joint";
        
        if (DetachJoint(name_of_joint_to_detach)) {
             // DetachJoint returns true if joint was removed or was not found (already detached).
             res->success = true;
             res->message = "Detachment successful (joint removed or was not present).";
        } else {
            // This case means the joint was found but could not be removed, indicating an issue.
            RCLCPP_ERROR(this->ros_node_->get_logger(), "Joint [%s] was found but failed to be removed.", name_of_joint_to_detach.c_str());
            res->success = false; 
            res->message = "Joint found but could not be removed.";
        }
    }

private:
    gazebo::physics::WorldPtr world_;
    sdf::ElementPtr sdf_;
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Service<gazebo_ros_link_attacher::srv::Attach>::SharedPtr attach_service_;
    rclcpp::Service<gazebo_ros_link_attacher::srv::Attach>::SharedPtr detach_service_;
    std::string joint_name_; // To store the name of the created joint
};

GZ_REGISTER_WORLD_PLUGIN(GazeboRosLinkAttacherPlugin)
} // namespace gazebo_plugins
