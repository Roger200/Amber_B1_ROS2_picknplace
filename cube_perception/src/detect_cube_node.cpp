#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // For tf2::toMsg

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/statistical_outlier_removal.h>

// Define a PCL point type that includes RGB information.
using PointT = pcl::PointXYZRGB;

class DetectCubeNode : public rclcpp::Node
{
public:
    DetectCubeNode() : Node("detect_cube_node")
    {
        // Subscription to the input point cloud topic.
        // The topic name "/camera/depth/color/points" is common for RGB-D cameras in Gazebo.
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", 10, // QoS profile depth 10
            std::bind(&DetectCubeNode::pointcloud_callback, this, std::placeholders::_1));

        // Publisher for the detected cube's pose.
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/detected_cube_pose", 10);

        // Declare ROS parameters for various filtering and segmentation stages.
        // These parameters allow tuning the detection pipeline without recompiling the code.

        // Passthrough filter: Used to isolate a region of interest (ROI) by discarding points outside a defined range on X, Y, Z axes.
        // This helps to focus on the area where the cube is expected.
        this->declare_parameter<double>("passthrough_x_min", 0.0);       // Min distance along X-axis (camera forward)
        this->declare_parameter<double>("passthrough_x_max", 1.0);       // Max distance along X-axis
        this->declare_parameter<double>("passthrough_y_min", -0.5);      // Min distance along Y-axis (camera left/right)
        this->declare_parameter<double>("passthrough_y_max", 0.5);       // Max distance along Y-axis
        this->declare_parameter<double>("passthrough_z_min", 0.0);       // Min distance along Z-axis (camera up/down, relative to camera frame)
        this->declare_parameter<double>("passthrough_z_max", 0.5);       // Max distance along Z-axis

        // Voxel Grid filter: Downsamples the point cloud to reduce computational load and noise.
        // It creates a 3D grid and replaces all points within each voxel with their centroid.
        this->declare_parameter<double>("voxel_leaf_size", 0.01); // Leaf size (resolution) of the voxel grid in meters (e.g., 1cm).

        // Color Segmentation: Filters points based on their RGB values to isolate objects of a specific color (e.g., a red cube).
        // These ranges should be tuned based on the specific color of the target object and lighting conditions.
        this->declare_parameter<int>("color_r_min", 130); // Minimum Red channel value
        this->declare_parameter<int>("color_r_max", 255); // Maximum Red channel value
        this->declare_parameter<int>("color_g_min", 0);   // Minimum Green channel value
        this->declare_parameter<int>("color_g_max", 100); // Maximum Green channel value
        this->declare_parameter<int>("color_b_min", 0);   // Minimum Blue channel value
        this->declare_parameter<int>("color_b_max", 100); // Maximum Blue channel value
        
        // Statistical Outlier Removal (SOR): Removes sparse outliers from the point cloud.
        // It calculates the mean distance of each point to its K nearest neighbors and removes points
        // whose mean distance is outside a threshold (mean + std_dev_multiplier * std_dev).
        this->declare_parameter<int>("sor_mean_k", 50);                     // Number of nearest neighbors to analyze for each point.
        this->declare_parameter<double>("sor_stddev_mul_thresh", 1.0);     // Standard deviation multiplier threshold.

        // Euclidean Clustering: Groups points into clusters based on their spatial proximity.
        // This helps to segment individual objects from the scene.
        this->declare_parameter<double>("cluster_tolerance", 0.03); // Maximum distance between neighboring points for them to be considered part of the same cluster (e.g., 3cm).
        this->declare_parameter<int>("cluster_min_size", 20);      // Minimum number of points a cluster must have to be considered valid.
                                                                  // This helps filter out small noise clusters.
        this->declare_parameter<int>("cluster_max_size", 1000);   // Maximum number of points a cluster can have.
                                                                  // For a 5cm cube and 1cm voxel size, expect around 5*5*5 = 125 points.
                                                                  // This helps filter out very large erroneous clusters.

        RCLCPP_INFO(this->get_logger(), "DetectCubeNode initialized with parameters.");
    }

private:
    // Main callback function triggered upon receiving a point cloud message.
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 message to PCL PointCloud type.
        // Using PointXYZRGB to preserve color information, crucial for color segmentation.
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msg, *cloud);

        // Early exit if the input cloud is empty to prevent further processing errors.
        if (cloud->empty())
        {
            // RCLCPP_WARN(this->get_logger(), "Input cloud is empty, skipping processing.");
            return;
        }

        // --- Point Cloud Processing Pipeline ---

        // 1. Passthrough Filter:
        // Apply filters along X, Y, and Z axes to narrow down the search space.
        // This is crucial for performance and reducing false positives.
        pcl::PointCloud<PointT>::Ptr cloud_passthrough(new pcl::PointCloud<PointT>);
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(this->get_parameter("passthrough_x_min").as_double(), this->get_parameter("passthrough_x_max").as_double());
        pass.filter(*cloud_passthrough);
        
        // It's important to chain the passthrough filters, using the output of one as the input to the next.
        pass.setInputCloud(cloud_passthrough); 
        pass.setFilterFieldName("y");
        pass.setFilterLimits(this->get_parameter("passthrough_y_min").as_double(), this->get_parameter("passthrough_y_max").as_double());
        pass.filter(*cloud_passthrough);

        pass.setInputCloud(cloud_passthrough);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(this->get_parameter("passthrough_z_min").as_double(), this->get_parameter("passthrough_z_max").as_double());
        pass.filter(*cloud_passthrough);

        // Early exit if passthrough filter results in an empty cloud.
        if (cloud_passthrough->empty())
        {
            // RCLCPP_WARN(this->get_logger(), "Cloud empty after passthrough filter(s).");
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "Cloud size after passthrough: %ld points.", cloud_passthrough->points.size());

        // 2. Voxel Grid Downsampling:
        // Reduces the number of points, speeding up subsequent processing and smoothing out noise.
        pcl::PointCloud<PointT>::Ptr cloud_voxel(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cloud_passthrough);
        double leaf_size = this->get_parameter("voxel_leaf_size").as_double();
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_filter.filter(*cloud_voxel);

        if (cloud_voxel->empty())
        {
            // RCLCPP_WARN(this->get_logger(), "Cloud empty after voxel filter.");
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "Cloud size after voxel filter: %ld points.", cloud_voxel->points.size());

        // 3. Color Segmentation:
        // Iterate through the downsampled cloud and keep only points that fall within the defined RGB range for "red".
        // This is a simple but effective way to isolate objects of a known color.
        pcl::PointCloud<PointT>::Ptr cloud_color_segmented(new pcl::PointCloud<PointT>);
        int r_min = this->get_parameter("color_r_min").as_int();
        int r_max = this->get_parameter("color_r_max").as_int();
        int g_min = this->get_parameter("color_g_min").as_int();
        int g_max = this->get_parameter("color_g_max").as_int();
        int b_min = this->get_parameter("color_b_min").as_int();
        int b_max = this->get_parameter("color_b_max").as_int();

        for (const auto& point : cloud_voxel->points)
        {
            if (point.r >= r_min && point.r <= r_max &&
                point.g >= g_min && point.g <= g_max &&
                point.b >= b_min && point.b <= b_max)
            {
                cloud_color_segmented->points.push_back(point);
            }
        }
        // Important: Update cloud properties after manually populating points.
        cloud_color_segmented->width = cloud_color_segmented->points.size();
        cloud_color_segmented->height = 1; // Unordered point cloud
        cloud_color_segmented->is_dense = true;

        if (cloud_color_segmented->empty())
        {
            // RCLCPP_WARN(this->get_logger(), "Cloud empty after color segmentation (no red points found).");
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "Cloud size after color segmentation: %ld points.", cloud_color_segmented->points.size());

        // 4. Statistical Outlier Removal (SOR):
        // Cleans up the color-segmented cloud by removing isolated points (noise)
        // that are statistically different from their neighborhood.
        pcl::PointCloud<PointT>::Ptr cloud_sor(new pcl::PointCloud<PointT>);
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cloud_color_segmented);
        sor.setMeanK(this->get_parameter("sor_mean_k").as_int());
        sor.setStddevMulThresh(this->get_parameter("sor_stddev_mul_thresh").as_double());
        sor.filter(*cloud_sor);

        if (cloud_sor->empty()) {
            // RCLCPP_WARN(this->get_logger(), "Cloud empty after Statistical Outlier Removal.");
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "Cloud size after SOR filter: %ld points.", cloud_sor->points.size());

        // 5. Euclidean Clustering:
        // Groups the remaining points into distinct clusters. Each cluster represents a potential object.
        // A KD-Tree is used for efficient nearest-neighbor searches.
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud_sor); // Input the cleaned, color-segmented cloud.

        std::vector<pcl::PointIndices> cluster_indices; // This will store lists of point indices for each detected cluster.
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(this->get_parameter("cluster_tolerance").as_double()); 
        ec.setMinClusterSize(this->get_parameter("cluster_min_size").as_int());
        ec.setMaxClusterSize(this->get_parameter("cluster_max_size").as_int());
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_sor);
        ec.extract(cluster_indices); // Perform clustering.

        if (cluster_indices.empty())
        {
            // RCLCPP_WARN(this->get_logger(), "No clusters found after Euclidean Clustering.");
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "Found %ld clusters.", cluster_indices.size());

        // 6. Identify the Largest Cluster:
        // Assumption: The largest cluster of red points is the target cube.
        // This is a common heuristic but might fail if other larger red objects are present.
        pcl::PointCloud<PointT>::Ptr largest_cluster(new pcl::PointCloud<PointT>);
        size_t largest_cluster_size = 0;
        for (const auto& indices : cluster_indices)
        {
            if (indices.indices.size() > largest_cluster_size)
            {
                largest_cluster_size = indices.indices.size();
                // Extract the points belonging to the current largest cluster.
                pcl::ExtractIndices<PointT> extract;
                extract.setInputCloud(cloud_sor);
                pcl::PointIndices::Ptr point_indices(new pcl::PointIndices(indices));
                extract.setIndices(point_indices);
                extract.setNegative(false); // Keep the points specified by indices.
                extract.filter(*largest_cluster);
            }
        }
        
        if (largest_cluster->empty()) {
            // This should ideally not happen if cluster_indices was not empty.
            // RCLCPP_WARN(this->get_logger(), "Largest cluster is empty after extraction (unexpected).");
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "Largest cluster size: %ld points.", largest_cluster->points.size());

        // 7. Calculate Centroid and Publish Pose:
        // The centroid of the largest cluster is used as the position of the detected cube.
        Eigen::Vector4f centroid; // PCL's centroid type (x, y, z, 1).
        pcl::compute3DCentroid(*largest_cluster, centroid);

        geometry_msgs::msg::PoseStamped pose_msg;
        // Use the same frame_id as the input point cloud.
        // The timestamp can be from the input message or current time. Using current time is often preferred for fresh poses.
        pose_msg.header = msg->header; 
        pose_msg.header.stamp = this->get_clock()->now(); 

        pose_msg.pose.position.x = centroid[0];
        pose_msg.pose.position.y = centroid[1];
        pose_msg.pose.position.z = centroid[2];

        // Set a default orientation (no rotation relative to the point cloud's frame).
        // A more advanced approach might involve fitting a bounding box to estimate orientation.
        tf2::Quaternion q;
        q.setRPY(0, 0, 0); // Roll, Pitch, Yaw
        pose_msg.pose.orientation = tf2::toMsg(q);
        
        // Optional: Add checks based on expected cube size or centroid properties if needed.
        // For example, if the centroid's Z value is too high or low, it might indicate an error.
        // The passthrough filter already constrains Z, but additional checks could be useful.

        pose_publisher_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published detected cube pose: [x: %.3f, y: %.3f, z: %.3f] in frame '%s'",
                    pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z, pose_msg.header.frame_id.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_; // ROS 2 subscription object.
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectCubeNode>());
    rclcpp::shutdown();
    return 0;
}
