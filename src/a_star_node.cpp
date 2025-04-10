#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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

        // QoS profile for subscriptions
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

        // Get the robot's current position in the costmap frame
        geometry_msgs::msg::PoseStamped current_position;
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                costmap_->header.frame_id, frame_id_, tf2::TimePointZero);

            current_position.pose.position.x = transform.transform.translation.x;
            current_position.pose.position.y = transform.transform.translation.y;
            current_position.pose.position.z = transform.transform.translation.z;
            current_position.header.frame_id = costmap_->header.frame_id;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
            return;
        }

        // Initialize the planned path
        nav_msgs::msg::Path planned_path;
        planned_path.header.stamp = this->now();
        planned_path.header.frame_id = costmap_->header.frame_id;

        // Plan paths sequentially between points
        geometry_msgs::msg::PoseStamped start = current_position;
        for (const auto &goal : msg->poses) {
            // Plan the path from the current start to the goal
            auto segment_path = planPath(start, goal);

            if (segment_path.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to plan a path between points");
                return;
            }

            // Concatenate the segment to the planned path
            planned_path.poses.insert(planned_path.poses.end(), segment_path.begin(), segment_path.end());

            // Update the start for the next segment
            start = goal;
        }

        // Smooth the concatenated path
        auto smoothed_path = smoothPath(planned_path.poses);

        // Update the planned path with the smoothed poses
        planned_path.poses = smoothed_path;

        // Publish the smoothed path
        path_pub_->publish(planned_path);
        RCLCPP_INFO(this->get_logger(), "Published smoothed path with %zu poses", planned_path.poses.size());
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

        // Check if the start or goal is in an obstacle
        if (costmap_->data[toIndex(start_x, start_y)] > obstacle_threshold_) {
            RCLCPP_WARN(this->get_logger(), "Start position is in an obstacle. Assuming free space.");
        }
        if (costmap_->data[toIndex(goal_x, goal_y)] > obstacle_threshold_) {
            RCLCPP_WARN(this->get_logger(), "Goal position is in an obstacle. Assuming free space.");
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
        float z_start = start.pose.position.z;
        float z_goal = goal.pose.position.z;

        // Calculate the total number of steps in the path
        std::vector<int> reverse_indices;
        int temp_index = current_index;
        while (came_from.count(temp_index)) {
            reverse_indices.push_back(temp_index);
            temp_index = came_from[temp_index];
        }
        reverse_indices.push_back(toIndex(start_x, start_y)); // Include the start position

        // Ensure there are steps to interpolate
        int total_steps = reverse_indices.size();
        if (total_steps <= 1) {
            RCLCPP_WARN(this->get_logger(), "Path has insufficient steps for interpolation. Creating a direct path.");

            // Special case: Directly create a path with the start and goal positions
            std::vector<geometry_msgs::msg::PoseStamped> direct_path;

            // Add the start position
            geometry_msgs::msg::PoseStamped start_pose;
            start_pose.header.frame_id = costmap_->header.frame_id;
            start_pose.pose.position.x = start.pose.position.x;
            start_pose.pose.position.y = start.pose.position.y;
            start_pose.pose.position.z = start.pose.position.z;
            direct_path.push_back(start_pose);

            // Add the goal position
            geometry_msgs::msg::PoseStamped goal_pose;
            goal_pose.header.frame_id = costmap_->header.frame_id;
            goal_pose.pose.position.x = goal.pose.position.x;
            goal_pose.pose.position.y = goal.pose.position.y;
            goal_pose.pose.position.z = goal.pose.position.z;
            direct_path.push_back(goal_pose);

            return direct_path;
        }

        // Extract x-y coordinates from reverse_indices
        std::vector<std::pair<float, float>> xy_points;
        for (int i = 0; i < total_steps; ++i) {
            int index = reverse_indices[total_steps - 1 - i]; // Reverse the order
            int x = index % width;
            int y = index / width;
            float world_x = x * resolution + costmap_->info.origin.position.x;
            float world_y = y * resolution + costmap_->info.origin.position.y;
            xy_points.emplace_back(world_x, world_y);
        }

        // Calculate the z-step for interpolation
        float z_step = (z_goal - z_start) / (xy_points.size() - 1); // Linear interpolation step

        // Reconstruct the path with z-coordinate
        for (size_t i = 0; i < xy_points.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = costmap_->header.frame_id;
            pose.pose.position.x = xy_points[i].first;
            pose.pose.position.y = xy_points[i].second;
            pose.pose.position.z = z_start + (z_step * i); // Interpolate z-coordinate
            path.push_back(pose);
        }

        return path;
    }

    std::vector<geometry_msgs::msg::PoseStamped> smoothPath(const std::vector<geometry_msgs::msg::PoseStamped> &path) {
        if (path.size() <= 2) {
            // No smoothing needed for paths with 2 or fewer points
            return path;
        }

        std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;
        smoothed_path.push_back(path.front()); // Preserve the start position

        int smoothing_window = smoothing_window_;
        for (size_t i = 1; i < path.size() - 1; ++i) { // Exclude the first and last points
            float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
            int count = 0;
            for (int j = -smoothing_window; j <= smoothing_window; ++j) {
                int idx = i + j;
                if (idx >= 0 && idx < static_cast<int>(path.size())) {
                    sum_x += path[idx].pose.position.x;
                    sum_y += path[idx].pose.position.y;
                    sum_z += path[idx].pose.position.z;
                    count++;
                }
            }

            geometry_msgs::msg::PoseStamped smoothed_pose;
            smoothed_pose.header.frame_id = path[i].header.frame_id;
            smoothed_pose.pose.position.x = sum_x / count;
            smoothed_pose.pose.position.y = sum_y / count;
            smoothed_pose.pose.position.z = sum_z / count;
            smoothed_path.push_back(smoothed_pose);
        }

        smoothed_path.push_back(path.back()); // Preserve the end position
        return smoothed_path;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AStarPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}