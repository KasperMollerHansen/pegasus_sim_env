#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
        this->declare_parameter<double>("interpolation_distance", 2.0);

        obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        interpolation_distance_ = this->get_parameter("interpolation_distance").as_double();

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
    nav_msgs::msg::Path::SharedPtr last_trajectory_path_; // Store the last received trajectory path
    int obstacle_threshold_;
    std::string frame_id_;
    double interpolation_distance_;

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        costmap_ = msg;
        RCLCPP_INFO(this->get_logger(), "Received costmap");

        // Re-plan and publish the path only when the costmap is updated
        if (last_trajectory_path_) {
            planAndPublishPath();
        }
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty Path message");
            return;
        }

        //RCLCPP_INFO(this->get_logger(), "Received Path with %zu poses", msg->poses.size());
        last_trajectory_path_ = msg; // Store the received trajectory path
    }

    void planAndPublishPath() {
        if (!costmap_ || !last_trajectory_path_) {
            RCLCPP_ERROR(this->get_logger(), "Cannot plan path: costmap or trajectory path is missing");
            return;
        }

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
        for (size_t i = 0; i < last_trajectory_path_->poses.size(); ++i) {
            const auto &goal = last_trajectory_path_->poses[i];

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

        // Calculate the Euclidean distance between start and goal, including the z-axis
        float distance = std::sqrt(
            std::pow(goal.pose.position.x - start.pose.position.x, 2) +
            std::pow(goal.pose.position.y - start.pose.position.y, 2) +
            std::pow(goal.pose.position.z - start.pose.position.z, 2));

        // If the distance exceeds the interpolation_distance, interpolate intermediate points
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        if (distance > interpolation_distance_) {
            int num_intermediate_points = static_cast<int>(std::ceil(distance / interpolation_distance_));
            for (int i = 1; i <= num_intermediate_points; ++i) {
                float t = static_cast<float>(i) / (num_intermediate_points + 1);
                geometry_msgs::msg::PoseStamped intermediate;
                intermediate.header.frame_id = costmap_->header.frame_id;
                intermediate.pose.position.x = start.pose.position.x + t * (goal.pose.position.x - start.pose.position.x);
                intermediate.pose.position.y = start.pose.position.y + t * (goal.pose.position.y - start.pose.position.y);
                intermediate.pose.position.z = start.pose.position.z + t * (goal.pose.position.z - start.pose.position.z); // Interpolate z
                
                // Calculate yaw and convert to quaternion
                float yaw = std::atan2(
                    goal.pose.position.y - start.pose.position.y,
                    goal.pose.position.x - start.pose.position.x);
                tf2::Quaternion quaternion;
                quaternion.setRPY(0, 0, yaw); // Roll and pitch are 0, only yaw is relevant
                intermediate.pose.orientation = tf2::toMsg(quaternion);

                waypoints.push_back(intermediate);
            }
        }
        // Add the goal as the final waypoint
        geometry_msgs::msg::PoseStamped final_waypoint = goal;

        // Calculate yaw for the final waypoint
        float final_yaw = std::atan2(
            goal.pose.position.y - start.pose.position.y,
            goal.pose.position.x - start.pose.position.x);
        tf2::Quaternion final_quaternion;
        final_quaternion.setRPY(0, 0, final_yaw);
        final_waypoint.pose.orientation = tf2::toMsg(final_quaternion);

        waypoints.push_back(final_waypoint);

        // Initialize the full path
        std::vector<geometry_msgs::msg::PoseStamped> full_path;

        // Plan path between consecutive waypoints
        geometry_msgs::msg::PoseStamped current_start = start;
        for (const auto &current_goal : waypoints) {
            int start_x = static_cast<int>((current_start.pose.position.x - costmap_->info.origin.position.x) / resolution);
            int start_y = static_cast<int>((current_start.pose.position.y - costmap_->info.origin.position.y) / resolution);
            int goal_x = static_cast<int>((current_goal.pose.position.x - costmap_->info.origin.position.x) / resolution);
            int goal_y = static_cast<int>((current_goal.pose.position.y - costmap_->info.origin.position.y) / resolution);

            // Check if start or goal is out of bounds
            bool start_out_of_bounds = (start_x < 0 || start_x >= width || start_y < 0 || start_y >= height);
            bool goal_out_of_bounds = (goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height);

            // if (start_out_of_bounds) {
            //     RCLCPP_WARN(this->get_logger(), "Start position is out of bounds. Assuming free space.");
            // }
            // if (goal_out_of_bounds) {
            //     RCLCPP_WARN(this->get_logger(), "Goal position is out of bounds. Assuming free space.");
            // }

            // A* algorithm
            std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
            std::unordered_map<int, int> came_from;
            std::unordered_map<int, float> cost_so_far;

            if (!start_out_of_bounds) {
                open_list.push({start_x, start_y, 0});
                cost_so_far[toIndex(start_x, start_y)] = 0;
            }

            std::vector<int> dx = {1, -1, 0, 0};
            std::vector<int> dy = {0, 0, 1, -1};

            while (!open_list.empty()) {
                AStarNode current = open_list.top();
                open_list.pop();

                if (!goal_out_of_bounds && current.x == goal_x && current.y == goal_y) {
                    break;
                }

                for (size_t i = 0; i < dx.size(); ++i) {
                    int next_x = current.x + dx[i];
                    int next_y = current.y + dy[i];

                    // Skip neighbors that are out of bounds
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
            std::vector<geometry_msgs::msg::PoseStamped> segment_path;
            if (!goal_out_of_bounds) {
                int current_index = toIndex(goal_x, goal_y);

                while (came_from.count(current_index)) {
                    int x = current_index % width;
                    int y = current_index / width;
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = costmap_->header.frame_id;
                    pose.pose.position.x = x * resolution + costmap_->info.origin.position.x;
                    pose.pose.position.y = y * resolution + costmap_->info.origin.position.y;
                    pose.pose.position.z = current_start.pose.position.z; // Keep z constant
                    segment_path.push_back(pose);
                    current_index = came_from[current_index];
                }
            }

            // Add the start position to the segment path
            segment_path.push_back(current_start);

            // Reverse the segment path to get it in the correct order
            std::reverse(segment_path.begin(), segment_path.end());

            // Append the segment path to the full path
            full_path.insert(full_path.end(), segment_path.begin(), segment_path.end());

            // Update the current start for the next segment
            current_start = current_goal;
        }

        return full_path;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AStarPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}