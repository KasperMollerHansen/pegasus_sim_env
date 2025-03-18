#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <cmath>

using namespace std;

class GroundPlaneFilterNode : public rclcpp::Node
{
public:
    GroundPlaneFilterNode() : Node("ground_plane_filter_node")
    {
        // Create a subscriber for the PointCloud2 topic
        cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/isaac/lidar/raw/pointcloud", rclcpp::SensorDataQoS(),
            std::bind(&GroundPlaneFilterNode::pointCloudCallback, this, std::placeholders::_1));

        // Create a publisher for the filtered PointCloud2
        cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/isaac/lidar/filtered/pointcloud", 10);

        // Create a subscriber for the vehicle odometry
        odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&GroundPlaneFilterNode::odometryCallback, this, std::placeholders::_1));   
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Create a new PointCloud for filtered points
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Apply filtering logic
        for (const auto &point : pcl_cloud->points)
        {
            // Discard points with Z between -0.1 and 0.1
            if (point.z > -0.1 && point.z < 0.1)
                continue;

            // Retain points within a 15-meter radius in the X-Y plane
            double distance = std::hypot(point.x - current_position_.x, point.y - current_position_.y);
            if (distance <= 15.0)
            {
                filtered_cloud->points.push_back(point);
            }
        }

        // Convert filtered PCL PointCloud back to ROS PointCloud2
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);
        filtered_msg.header = msg->header;

        // Publish the filtered PointCloud
        cloud_publisher_->publish(filtered_msg);
    }

    void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // Update the current position from the odometry message
        current_position_.x = msg->position[1]; // Y
        current_position_.y = msg->position[0]; // X
        current_position_.z = -msg->position[2]; // -Z
    }

    struct Position
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    } current_position_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundPlaneFilterNode>());
    rclcpp::shutdown();
    return 0;
}
