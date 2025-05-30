#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>

class ESDFCostMapNode : public rclcpp::Node {
public:
  ESDFCostMapNode()
  : Node("esdf_cost_map_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Declare and retrieve parameters
    this->declare_parameter("resolution", 1.0);     // Cell size in meters
    this->declare_parameter("free_center_radius", 10.0); // Radius of the free center area
    this->declare_parameter("local_map_size", 200.0);    // Local map size (200 m x 200 m)
    this->declare_parameter("global_map_size", 1500.0);  // Global map size (1500 m x 1500 m)
    this->declare_parameter("frame_id", "base_link");    // Map centered at base_link
    this->declare_parameter("safety_distance_min", 5.0);  // Minimum safety distance
    this->declare_parameter("safety_distance_max", 10.0);  // Maximum safety distance

    resolution_ = this->get_parameter("resolution").as_double();
    free_center_radius_ = this->get_parameter("free_center_radius").as_double();
    local_map_size_ = this->get_parameter("local_map_size").as_double();
    global_map_size_ = this->get_parameter("global_map_size").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();
    safety_distance_min_ = this->get_parameter("safety_distance_min").as_double();
    safety_distance_max_ = this->get_parameter("safety_distance_max").as_double();

    // Compute grid dimensions
    local_grid_size_ = static_cast<int>(local_map_size_ / resolution_);
    global_grid_size_ = static_cast<int>(global_map_size_ / resolution_);
    local_half_size_ = local_map_size_ / 2.0;
    global_half_size_ = global_map_size_ / 2.0;

    // Initialize the global map
    global_map_.info.resolution = resolution_;
    global_map_.info.width = global_grid_size_;
    global_map_.info.height = global_grid_size_;
    global_map_.info.origin.position.x = -global_half_size_;
    global_map_.info.origin.position.y = -global_half_size_;
    global_map_.info.origin.position.z = 0.0;
    global_map_.info.origin.orientation.w = 1.0;
    global_map_.data.resize(global_grid_size_ * global_grid_size_, -1); // Initialize as unknown

    // Subscribe to the ESDF point cloud topic
    esdf_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/nvblox_node/static_esdf_pointcloud", 10,
      std::bind(&ESDFCostMapNode::esdf_callback, this, std::placeholders::_1)
    );

    // Publisher for the global cost map
    global_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap", 10);

    // Publisher for the local cost map
    local_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap/costmap", 10);

    RCLCPP_INFO(this->get_logger(), "ESDF cost map node initialized.");
  }

private:
    void esdf_callback(const sensor_msgs::msg::PointCloud2::SharedPtr esdf_msg)
    {
        // Convert the PointCloud2 message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*esdf_msg, *cloud);

        // Transform the base_link position into the global frame
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform("odom", frame_id_, rclcpp::Time(0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to odom: %s", frame_id_.c_str(), ex.what());
            return;
        }

        // Create a local occupancy grid based on the global map
        nav_msgs::msg::OccupancyGrid local_map;
        local_map.info.resolution = resolution_;
        local_map.info.width = local_grid_size_;
        local_map.info.height = local_grid_size_;
        local_map.info.origin.position.x = std::floor((transform.transform.translation.x - local_half_size_ - global_map_.info.origin.position.x) / resolution_) * resolution_ + global_map_.info.origin.position.x;
        local_map.info.origin.position.y = std::floor((transform.transform.translation.y - local_half_size_ - global_map_.info.origin.position.y) / resolution_) * resolution_ + global_map_.info.origin.position.y;
        local_map.info.origin.position.z = 0.0;
        local_map.info.origin.orientation.w = 1.0;
        local_map.data.resize(local_grid_size_ * local_grid_size_, -1); // Initialize as unknown

        // Extract the corresponding region from the global map
        for (int y = 0; y < local_grid_size_; ++y) {
            for (int x = 0; x < local_grid_size_; ++x) {
                int global_x = static_cast<int>((local_map.info.origin.position.x + x * resolution_ - global_map_.info.origin.position.x) / resolution_);
                int global_y = static_cast<int>((local_map.info.origin.position.y + y * resolution_ - global_map_.info.origin.position.y) / resolution_);
                int global_index = global_y * global_grid_size_ + global_x;
                int local_index = y * local_grid_size_ + x;

                if (global_x >= 0 && global_x < global_grid_size_ && global_y >= 0 && global_y < global_grid_size_) {
                    local_map.data[local_index] = global_map_.data[global_index];
                }
            }
        }

        // Overwrite the local map with ESDF data
        for (const auto& point : cloud->points) {
            int grid_x = static_cast<int>((point.x - local_map.info.origin.position.x) / resolution_);
            int grid_y = static_cast<int>((point.y - local_map.info.origin.position.y) / resolution_);

            if (grid_x >= 0 && grid_x < local_grid_size_ && grid_y >= 0 && grid_y < local_grid_size_) {
                float distance = point.intensity; // Distance to nearest obstacle from ESDF
                int cost;

                if (distance < 0.0f) {
                    // Inside an obstacle or invalid value
                    cost = 100; // Maximum cost for obstacle
                } else if (distance <= safety_distance_min_) {
                    // Within minimum safety distance
                    cost = 100; // Maximum cost (unsafe region)
                } else if (distance <= safety_distance_max_) {
                    // Between minimum and maximum safety distance
                    // Linearly interpolate cost from 100 to 0
                    cost = static_cast<int>(100.0f * (1.0f - (distance - safety_distance_min_) /
                                                            (safety_distance_max_ - safety_distance_min_)));
                } else {
                    // Beyond maximum safety distance
                    cost = 0; // No cost (safe region)
                }

                // Ensure cost stays within the range [0, 100]
                cost = std::clamp(cost, 0, 100);

                local_map.data[grid_y * local_grid_size_ + grid_x] = cost;
            }
        }

        // Clear the local area corresponding to the center of the global map
        double clear_radius = free_center_radius_; // Radius in meters
        int clear_radius_cells = static_cast<int>(clear_radius / resolution_);

        // Calculate the center of the global map in the local map's coordinate system
        int global_center_x = static_cast<int>((global_map_.info.origin.position.x + global_half_size_ - local_map.info.origin.position.x) / resolution_);
        int global_center_y = static_cast<int>((global_map_.info.origin.position.y + global_half_size_ - local_map.info.origin.position.y) / resolution_);

        for (int y = 0; y < local_grid_size_; ++y) {
            for (int x = 0; x < local_grid_size_; ++x) {
                int dx = x - global_center_x;
                int dy = y - global_center_y;
                if (dx * dx + dy * dy <= clear_radius_cells * clear_radius_cells) {
                    local_map.data[y * local_grid_size_ + x] = 0; // Clear cell
                }
            }
        }

        // Merge the updated local map back into the global map
        for (int y = 0; y < local_grid_size_; ++y) {
            for (int x = 0; x < local_grid_size_; ++x) {
                int local_index = y * local_grid_size_ + x;
                int global_x = static_cast<int>((local_map.info.origin.position.x + x * resolution_ - global_map_.info.origin.position.x) / resolution_);
                int global_y = static_cast<int>((local_map.info.origin.position.y + y * resolution_ - global_map_.info.origin.position.y) / resolution_);
                int global_index = global_y * global_grid_size_ + global_x;

                if (global_x >= 0 && global_x < global_grid_size_ && global_y >= 0 && global_y < global_grid_size_) {
                    if (local_map.data[local_index] != -1) { // If the local cell is observed
                        global_map_.data[global_index] = local_map.data[local_index];
                    }
                }
            }
        }

        // Publish the local map
        local_map.header.stamp = this->now();
        local_map.header.frame_id = "odom";
        local_map_pub_->publish(local_map);

        // Publish the global map
        global_map_.header.stamp = this->now();
        global_map_.header.frame_id = "odom";
        global_map_pub_->publish(global_map_);

        // RCLCPP_INFO(this->get_logger(), "Published local and global cost maps.");
    }

    // Parameters and computed values
    double resolution_;
    double free_center_radius_;
    double local_map_size_;
    double global_map_size_;
    int local_grid_size_;
    int global_grid_size_;
    double local_half_size_;
    double global_half_size_;
    double safety_distance_min_;
    double safety_distance_max_;
    std::string frame_id_;

    // TF2 buffer and listener
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // ROS publisher and subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_pub_;

    // Global map
    nav_msgs::msg::OccupancyGrid global_map_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ESDFCostMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}