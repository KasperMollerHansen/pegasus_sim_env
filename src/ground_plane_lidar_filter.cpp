#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h> // Include pcl_conversions

using namespace std;

class GroundPlaneFilterNode : public rclcpp::Node
{
public:
    GroundPlaneFilterNode() : Node("ground_plane_filter_node")
    {
        // Create a subscriber for the PointCloud2 topic
        cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/isaac/lidar/pointcloud", rclcpp::SensorDataQoS(), 
            std::bind(&GroundPlaneFilterNode::pointCloudCallback, this, std::placeholders::_1));

        // Create a publisher for the filtered PointCloud2
        cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/isaac/lidar/filtered/pointcloud", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Apply a PassThrough filter to remove ground plane (Z < threshold)
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.05, std::numeric_limits<float>::max()); // Only keep points with Z > 0.05 meters
        pass.filter(*pcl_cloud);

        // Convert filtered PCL PointCloud back to ROS PointCloud2
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*pcl_cloud, filtered_msg);
        filtered_msg.header = msg->header;

        // Publish the filtered PointCloud
        cloud_publisher_->publish(filtered_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundPlaneFilterNode>());
    rclcpp::shutdown();
    return 0;
}