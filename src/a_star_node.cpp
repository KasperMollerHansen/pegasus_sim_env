#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
        this->declare_parameter<std::string>("frame_id", "odom");

        obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // Subscribe to the local costmap
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "local_costmap/costmap", 10,
            std::bind(&AStarPlanner::costmapCallback, this, std::placeholders::_1));

        // Subscribe to goal setpoints
        setpoints_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_setpoints", 10,
            std::bind(&AStarPlanner::setpointCallback, this, std::placeholders::_1));

        // Publisher for the planned path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr setpoints_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
    int obstacle_threshold_;
    std::string frame_id_;
    geometry_msgs::msg::PoseStamped::SharedPtr last_goal_;

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        costmap_ = msg;
        RCLCPP_INFO(this->get_logger(), "Received costmap");

        // Re-plan if a goal is already set
        if (last_goal_) {
            planAndPublishPath(*last_goal_);
        }
    }

    void setpointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received new setpoint");
        last_goal_ = msg;
        planAndPublishPath(*msg);
    }

    void planAndPublishPath(const geometry_msgs::msg::PoseStamped &goal) {
        if (!costmap_) {
            RCLCPP_ERROR(this->get_logger(), "Costmap not available yet");
            return;
        }

        // Assume the robot's current position is at the origin of the costmap
        geometry_msgs::msg::PoseStamped start;
        start.pose.position.x = 0.0;
        start.pose.position.y = 0.0;
        start.pose.position.z = 0.0;
        start.header.frame_id = costmap_->header.frame_id;

        // Plan the path to the goal
        auto path_poses = planPath(start, goal);
        if (path_poses.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan a path to the setpoint");
            return;
        }

        // Create a nav_msgs::msg::Path message
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = costmap_->header.frame_id;
        path_msg.poses = path_poses;

        // Publish the path
        path_pub_->publish(path_msg);
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

        std::vector<geometry_msgs::msg::PoseStamped> path;
        int current_index = toIndex(goal_x, goal_y);
        while (came_from.count(current_index)) {
            int x = current_index % width;
            int y = current_index / width;
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = costmap_->header.frame_id;
            pose.pose.position.x = x * resolution + costmap_->info.origin.position.x;
            pose.pose.position.y = y * resolution + costmap_->info.origin.position.y;
            pose.pose.position.z = 0.0;
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