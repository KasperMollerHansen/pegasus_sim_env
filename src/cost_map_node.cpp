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

class ESDFCostMapNode : public rclcpp::Node {
public:
  ESDFCostMapNode()
  : Node("esdf_cost_map_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Declare and retrieve parameters
    this->declare_parameter("resolution", 1.0);     // Cell size in meters
    this->declare_parameter("map_size", 150.0);    // Map is 150 m x 150 m
    this->declare_parameter("frame_id", "base_link"); // Map centered at base_link

    resolution_ = this->get_parameter("resolution").as_double();
    map_size_   = this->get_parameter("map_size").as_double();
    frame_id_   = this->get_parameter("frame_id").as_string();

    // Compute grid dimensions: number of cells in each direction
    grid_size_ = static_cast<int>(map_size_ / resolution_);
    half_size_ = map_size_ / 2.0;

    // Subscribe to the ESDF point cloud topic
    esdf_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/nvblox_node/static_esdf_pointcloud", 10,
      std::bind(&ESDFCostMapNode::esdf_callback, this, std::placeholders::_1)
    );

    // Publisher for the cost map (occupancy grid)
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/cost_map", 10);

    RCLCPP_INFO(this->get_logger(), "ESDF cost map node initialized.");
  }

private:
  void esdf_callback(const sensor_msgs::msg::PointCloud2::SharedPtr esdf_msg)
  {
    // Convert the PointCloud2 message to a PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*esdf_msg, *cloud);

    // Create an occupancy grid
    nav_msgs::msg::OccupancyGrid grid;

    // Use the ESDF message timestamp and set the frame_id
    grid.header.stamp = esdf_msg->header.stamp;
    grid.header.frame_id = "odom"; // Use a fixed frame like "map" or "odom"

    // Transform the base_link position into the global frame
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform("odom", frame_id_, rclcpp::Time(0));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to odom: %s", frame_id_.c_str(), ex.what());
        return;
    }

    // Set up map metadata
    grid.info.resolution = resolution_;
    grid.info.width = grid_size_;
    grid.info.height = grid_size_;
    grid.info.origin.position.x = transform.transform.translation.x - half_size_;
    grid.info.origin.position.y = transform.transform.translation.y - half_size_;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    // Initialize the grid data with unknown values (-1)
    grid.data.resize(grid_size_ * grid_size_, -1);

    // Populate the grid based on the ESDF data
    for (const auto& point : cloud->points) {
      // Convert the point's position to grid coordinates
      int grid_x = static_cast<int>((point.x - grid.info.origin.position.x) / resolution_);
      int grid_y = static_cast<int>((point.y - grid.info.origin.position.y) / resolution_);

      // Check if the point is within the grid bounds
      if (grid_x >= 0 && grid_x < grid_size_ && grid_y >= 0 && grid_y < grid_size_) {
        // Compute the cost based on the distance (e.g., inverse of distance)
        float distance = std::sqrt(point.x * point.x + point.y * point.y);
        int cost = static_cast<int>(100.0f * std::exp(-distance));// Example cost function

        // Update the grid cell
        grid.data[grid_y * grid_size_ + grid_x] = cost;
      }
    }

    // Publish the occupancy grid
    costmap_pub_->publish(grid);
    RCLCPP_INFO(this->get_logger(), "Published ESDF-based cost map at time: %u.%u", 
                grid.header.stamp.sec, grid.header.stamp.nanosec);
  }

  // Parameters and computed values
  double resolution_;
  double map_size_;
  int grid_size_;
  double half_size_;
  std::string frame_id_;

  // TF2 buffer and listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ROS publisher and subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ESDFCostMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}