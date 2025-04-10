#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
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

        obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // QoS profile matching the TestFlight node
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        // Subscribe to the local costmap
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "local_costmap/costmap", 10,
            std::bind(&AStarPlanner::costmapCallback, this, std::placeholders::_1));

        // Subscribe to goal setpoints from TestFlight
        setpoints_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
            "in/target_setpoint", qos_profile,
            std::bind(&AStarPlanner::setpointCallback, this, std::placeholders::_1));

        // Publisher for the planned path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        // Timer for debugging the cost of the current position
        debug_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&AStarPlanner::debugCurrentPositionCost, this));
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoints_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr debug_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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

    void setpointCallback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received new setpoint");

        // Convert TrajectorySetpoint to PoseStamped
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = this->now();
        goal.header.frame_id = frame_id_;  // Ensure the frame matches the costmap
        goal.pose.position.x = msg->position[0];
        goal.pose.position.y = msg->position[1];
        goal.pose.position.z = msg->position[2];
        // Convert yaw to quaternion
        float yaw = msg->yaw;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = std::sin(yaw / 2.0);
        goal.pose.orientation.w = std::cos(yaw / 2.0);

        last_goal_ = std::make_shared<geometry_msgs::msg::PoseStamped>(goal);
        planAndPublishPath(goal);
    }

    void debugCurrentPositionCost() {
        if (!costmap_) {
            RCLCPP_WARN(this->get_logger(), "Costmap not available for debugging");
            return;
        }

        // Assume the robot's current position is at the origin of the costmap
        int current_x = static_cast<int>((0.0 - costmap_->info.origin.position.x) / costmap_->info.resolution);
        int current_y = static_cast<int>((0.0 - costmap_->info.origin.position.y) / costmap_->info.resolution);

        if (current_x < 0 || current_x >= static_cast<int>(costmap_->info.width) ||
            current_y < 0 || current_y >= static_cast<int>(costmap_->info.height)) {
            RCLCPP_WARN(this->get_logger(), "Current position is outside the costmap bounds");
            return;
        }

        int index = current_y * costmap_->info.width + current_x;
        int cost = costmap_->data[index];

        if (cost == -1) {
            RCLCPP_INFO(this->get_logger(), "Current position cost: Unobserved (free space)");
        } else {
            RCLCPP_INFO(this->get_logger(), "Current position cost: %d", cost);
        }
    }
    void planAndPublishPath(const geometry_msgs::msg::PoseStamped &goal) {
        if (!costmap_) {
            RCLCPP_ERROR(this->get_logger(), "Costmap not available yet");
            return;
        }
    
        // Get the current position of the robot in the costmap frame
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
    
        // Plan the path to the goal
        auto path_poses = planPath(current_position, goal);
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
    
        // Check if the start or goal is in an obstacle
        if (costmap_->data[toIndex(start_x, start_y)] > obstacle_threshold_) {
            RCLCPP_WARN(this->get_logger(), "Start position is in an obstacle. Assuming free space.");
        }
        if (costmap_->data[toIndex(goal_x, goal_y)] > obstacle_threshold_) {
            RCLCPP_WARN(this->get_logger(), "Goal position is in an obstacle. Assuming free space.");
        }
    
        // A* algorithm for horizontal movement
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
            RCLCPP_WARN(this->get_logger(), "Path has insufficient steps for interpolation.");
            return {};
        }

        // Calculate the z-step for interpolation
        float z_step = (z_goal - z_start) / (total_steps - 1); // Linear interpolation step

        // Reconstruct the path in reverse order
        for (int i = 0; i < total_steps; ++i) {
            int index = reverse_indices[total_steps - 1 - i]; // Reverse the order
            int x = index % width;
            int y = index / width;

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = costmap_->header.frame_id;
            pose.pose.position.x = x * resolution + costmap_->info.origin.position.x;
            pose.pose.position.y = y * resolution + costmap_->info.origin.position.y;
            pose.pose.position.z = z_start + (z_step * i); // Interpolate z-coordinate
            path.push_back(pose);
        }

        // Add the final goal position with the correct z-coordinate
        geometry_msgs::msg::PoseStamped final_pose = goal;
        path.push_back(final_pose);

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