#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // Updated header
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>

// Define a struct for A* nodes
struct AStarNode {
    int x, y;
    float cost;

    bool operator>(const AStarNode &other) const {
        return cost > other.cost;
    }
};

class AStarPlanner : public rclcpp::Node {
public:
    AStarPlanner() : Node("astar_planner") {
        // Declare and retrieve parameters
        this->declare_parameter<int>("obstacle_threshold", 50);
        this->declare_parameter<std::string>("frame_id", "base_link");
        this->declare_parameter<int>("smoothing_window", 4);

        obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        smoothing_window_ = this->get_parameter("smoothing_window").as_int();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // QoS profile matching the TestFlight node
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        // Subscribe to the local costmap
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/local_costmap/costmap", 10,
            std::bind(&AStarPlanner::costmapCallback, this, std::placeholders::_1));

        // Subscribe to the Path topic
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "in/trajectory_path", qos_profile,
            std::bind(&AStarPlanner::pathCallback, this, std::placeholders::_1));

        // Publisher for the planned path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
    int obstacle_threshold_;
    int smoothing_window_;
    std::string frame_id_;

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        costmap_ = msg;
        RCLCPP_INFO(this->get_logger(), "Received costmap");
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!costmap_) {
            RCLCPP_ERROR(this->get_logger(), "Costmap not available for path planning");
            return;
        }

        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty Path message");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received Path with %zu poses", msg->poses.size());

        // Plan a path through all the poses in the Path message
        nav_msgs::msg::Path planned_path;
        planned_path.header.stamp = this->now();
        planned_path.header.frame_id = costmap_->header.frame_id;

        for (size_t i = 0; i < msg->poses.size() - 1; ++i) {
            const auto &start_pose = msg->poses[i];
            const auto &goal_pose = msg->poses[i + 1];

            // Transform start and goal poses to the costmap frame
            geometry_msgs::msg::PoseStamped transformed_start, transformed_goal;
            try {
                transformed_start = tf_buffer_->transform(start_pose, costmap_->header.frame_id, tf2::durationFromSec(0.1));
                transformed_goal = tf_buffer_->transform(goal_pose, costmap_->header.frame_id, tf2::durationFromSec(0.1));
            } catch (const tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
                return;
            }

            // Plan a path between the current pose and the next pose
            auto segment_path = planPath(transformed_start, transformed_goal);

            // Append the segment to the planned path
            planned_path.poses.insert(planned_path.poses.end(), segment_path.begin(), segment_path.end());
        }

        // Publish the planned path
        path_pub_->publish(planned_path);
        RCLCPP_INFO(this->get_logger(), "Published planned path with %zu poses", planned_path.poses.size());
    }

    std::vector<geometry_msgs::msg::PoseStamped> planPath(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal) {
        if (!costmap_) {
            RCLCPP_ERROR(this->get_logger(), "No costmap available");
            return {};
        }
    
        int width = costmap_->info.width;
        int height = costmap_->info.height;
        float resolution = costmap_->info.resolution;
    
        auto toIndex = [&](int x, int y) { return y * width + x; };
    
        auto heuristic = [&](int x1, int y1, int x2, int y2) {
            return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
        };
    
        int start_x = static_cast<int>((start.pose.position.x - costmap_->info.origin.position.x) / resolution);
        int start_y = static_cast<int>((start.pose.position.y - costmap_->info.origin.position.y) / resolution);
        int goal_x = static_cast<int>((goal.pose.position.x - costmap_->info.origin.position.x) / resolution);
        int goal_y = static_cast<int>((goal.pose.position.y - costmap_->info.origin.position.y) / resolution);
    
        // Handle out-of-bounds start or goal positions
        if (start_x < 0 || start_x >= width || start_y < 0 || start_y >= height) {
            RCLCPP_WARN(this->get_logger(), "Start position is outside the costmap bounds. Assuming free space.");
            start_x = std::clamp(start_x, 0, width - 1);
            start_y = std::clamp(start_y, 0, height - 1);
        }
    
        if (goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height) {
            RCLCPP_WARN(this->get_logger(), "Goal position is outside the costmap bounds. Assuming free space.");
            goal_x = std::clamp(goal_x, 0, width - 1);
            goal_y = std::clamp(goal_y, 0, height - 1);
        }
    
        // A* algorithm
        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
        std::unordered_map<int, int> came_from;
        std::unordered_map<int, float> cost_so_far;
    
        open_list.push({start_x, start_y, 0});
        cost_so_far[toIndex(start_x, start_y)] = 0;
    
        std::vector<int> dx = {1, -1, 0, 0};
        std::vector<int> dy = {0, 0, 1, -1};
    
        while (!open_list.empty()) {
            AStarNode current = open_list.top();
            open_list.pop();
    
            if (current.x == goal_x && current.y == goal_y) {
                break;
            }
    
            for (size_t i = 0; i < dx.size(); ++i) {
                int next_x = current.x + dx[i];
                int next_y = current.y + dy[i];
    
                if (next_x < 0 || next_y < 0 || next_x >= width || next_y >= height) {
                    continue;
                }
    
                int index = toIndex(next_x, next_y);
                if (costmap_->data[index] > obstacle_threshold_) { // Obstacle threshold
                    continue;
                }
    
                float new_cost = cost_so_far[toIndex(current.x, current.y)] + 1;
                if (!cost_so_far.count(index) || new_cost < cost_so_far[index]) {
                    cost_so_far[index] = new_cost;
                    float priority = new_cost + heuristic(next_x, next_y, goal_x, goal_y);
                    open_list.push({next_x, next_y, priority});
                    came_from[index] = toIndex(current.x, current.y);
                }
            }
        }
    
        // Reconstruct the path
        std::vector<geometry_msgs::msg::PoseStamped> path;
        int current_index = toIndex(goal_x, goal_y);
        while (came_from.count(current_index)) {
            int x = current_index % width;
            int y = current_index / width;
    
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = costmap_->header.frame_id;
            pose.pose.position.x = x * resolution + costmap_->info.origin.position.x;
            pose.pose.position.y = y * resolution + costmap_->info.origin.position.y;
            pose.pose.position.z = start.pose.position.z; // Keep the same z-coordinate
            path.push_back(pose);
    
            current_index = came_from[current_index];
        }
    
        std::reverse(path.begin(), path.end());
        return path;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AStarPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}