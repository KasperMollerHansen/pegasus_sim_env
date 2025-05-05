#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <eigen3/Eigen/Dense>

#include <tf2/utils.h>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <unordered_map>

// Define a struct for A* nodes
struct PlannerNode {
    int x, y;
    float cost;

    bool operator>(const PlannerNode &other) const {
        return cost > other.cost;
    }
};

class PathPlanner : public rclcpp::Node {
public:
    PathPlanner() : Node("planner") {
        // Declare and retrieve parameters
        this->declare_parameter<int>("obstacle_threshold", 50);
        this->declare_parameter<std::string>("frame_id", "base_link");
        this->declare_parameter<double>("interpolation_distance", 2.0);
        this->declare_parameter<std::string>("costmap_topic", "/local_costmap/costmap");
        this->declare_parameter<std::string>("waypoints_topic", "/oscep/waypoints");
        this->declare_parameter<std::string>("path_planner_prefix", "/planner");
        this->declare_parameter<int>("ground_truth_update_interval", 2000); // in milliseconds

        obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        interpolation_distance_ = this->get_parameter("interpolation_distance").as_double();
        std::string costmap_topic = this->get_parameter("costmap_topic").as_string();
        std::string waypoints_topic = this->get_parameter("waypoints_topic").as_string();
        std::string path_planner_prefix = this->get_parameter("path_planner_prefix").as_string();
        int ground_truth_update_interval = this->get_parameter("ground_truth_update_interval").as_int();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        ground_truth_trajectory_.header.frame_id = frame_id_;

        // Set up a timer to update the ground truth trajectory at a fixed interval (e.g., 2000ms)
        ground_truth_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(ground_truth_update_interval), // Adjust the interval as needed
            std::bind(&PathPlanner::updateGroundTruthTrajectory, this));

        // QoS profile for subscriptions
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        // Subscribe to the local costmap
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic, 10,
            std::bind(&PathPlanner::costmapCallback, this, std::placeholders::_1));

        // Subscribe to the Path topic
        waypoints_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            waypoints_topic, qos_profile,
            std::bind(&PathPlanner::waypointsCallback, this, std::placeholders::_1));

        // Publisher for the planned path
        viewpoints_adjusted_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_planner_prefix + "/viewpoints_adjusted", 10);
        raw_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_planner_prefix + "/raw_path", 10);
        smoothed_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_planner_prefix + "/smoothed_path", 10);
        ground_truth_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_planner_prefix + "/ground_truth_trajectory", 10);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoints_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr viewpoints_adjusted_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ground_truth_trajectory_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
    nav_msgs::msg::Path adjusted_waypoints_;
    int obstacle_threshold_;
    std::string frame_id_;
    double interpolation_distance_;
    rclcpp::TimerBase::SharedPtr ground_truth_timer_;
    nav_msgs::msg::Path ground_truth_trajectory_;
    bool path_invalid_flag_ = false;

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        costmap_ = msg;
        // RCLCPP_INFO(this->get_logger(), "Received costmap");
    
        // Re-plan and publish the path only when the costmap is updated
        if (!adjusted_waypoints_.poses.empty()) {
            planAndPublishPath();
        }
    }

    void waypointsCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!costmap_) {
            RCLCPP_ERROR(this->get_logger(), "Costmap is null");
            return;
        }
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty Path message");
            return;
        }
        nav_msgs::msg::Path all_adjusted_waypoints;
        all_adjusted_waypoints.header = msg->header;
    
        adjusted_waypoints_.header = msg->header;
        adjusted_waypoints_.poses.clear();
    
        for (const auto &pose : msg->poses) {
            auto [adjusted_pose, _] = adjustWaypointForCollision(pose, costmap_->info.resolution, 20);
    
            // Add the adjusted pose to the raw waypoints
            all_adjusted_waypoints.poses.push_back(adjusted_pose);
    
            // Only store valid points in adjusted_waypoints_
            if (!adjusted_pose.header.frame_id.empty()) {
                adjusted_waypoints_.poses.push_back(adjusted_pose);
            }
        }
        // Publish all waypoints (including invalid ones)
        viewpoints_adjusted_pub_->publish(all_adjusted_waypoints);
    }


    geometry_msgs::msg::PoseStamped getCurrentPosition() {
        geometry_msgs::msg::PoseStamped current_position;
    
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                costmap_->header.frame_id, frame_id_, tf2::TimePointZero);
    
            current_position.pose.position.x = transform.transform.translation.x;
            current_position.pose.position.y = transform.transform.translation.y;
            current_position.pose.position.z = transform.transform.translation.z;
            current_position.pose.orientation = transform.transform.rotation;
            current_position.header.frame_id = costmap_->header.frame_id;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
            current_position.header.frame_id = ""; // Mark as invalid
        }
    
        return current_position;
    }

    void updateGroundTruthTrajectory() {
        // Ensure the costmap is initialized
        if (!costmap_) {
            RCLCPP_WARN(this->get_logger(), "Costmap is not initialized, skipping update");
            return;
        }
    
        // Get the current position
        geometry_msgs::msg::PoseStamped current_position = getCurrentPosition();
    
        // Check if the current position is valid
        if (current_position.header.frame_id.empty()) {
            RCLCPP_WARN(this->get_logger(), "Invalid current position, skipping update");
            return;
        }
    
        // Update the header of the ground truth trajectory
        ground_truth_trajectory_.header.frame_id = costmap_->header.frame_id; // Ensure frame ID matches the costmap
        ground_truth_trajectory_.header.stamp = this->now(); // Update the timestamp
    
        // Append the current position to the trajectory
        ground_truth_trajectory_.poses.push_back(current_position);
    
        // Publish the updated trajectory
        ground_truth_trajectory_pub_->publish(ground_truth_trajectory_);
    
        // Log for debugging
        // RCLCPP_INFO(this->get_logger(), "Appended current position to ground truth trajectory and published");
    }

    // Function to check and adjust waypoints for collision-free zones
    std::pair<geometry_msgs::msg::PoseStamped, bool> adjustWaypointForCollision(
        const geometry_msgs::msg::PoseStamped &waypoint, float resolution, int max_attempts) {
    
        geometry_msgs::msg::PoseStamped adjusted_waypoint = waypoint;
        bool was_adjusted = false;
    
        if (!costmap_) {
            RCLCPP_ERROR(this->get_logger(), "Costmap is null");
            adjusted_waypoint.header.frame_id = ""; // Mark as invalid
            return {adjusted_waypoint, was_adjusted};
        }
    
        // Extract yaw from the waypoint's quaternion
        tf2::Quaternion quat;
        tf2::fromMsg(adjusted_waypoint.pose.orientation, quat);
        double yaw = tf2::getYaw(quat);
    
        for (int attempt = 0; attempt < max_attempts; ++attempt) {
            // Convert current waypoint position to costmap indices
            int x_index = static_cast<int>((adjusted_waypoint.pose.position.x - costmap_->info.origin.position.x) / resolution);
            int y_index = static_cast<int>((adjusted_waypoint.pose.position.y - costmap_->info.origin.position.y) / resolution);
    
            // Check if the current waypoint is within bounds
            if (x_index >= 0 && x_index < static_cast<int>(costmap_->info.width) &&
                y_index >= 0 && y_index < static_cast<int>(costmap_->info.height)) {
                int index = y_index * costmap_->info.width + x_index;
    
                // Check if the current waypoint is in a collision-free zone
                if (costmap_->data[index] <= obstacle_threshold_) {
                    // Test the point 0.5 meters in front
                    geometry_msgs::msg::PoseStamped forward_waypoint = adjusted_waypoint;
                    forward_waypoint.pose.position.x += 0.5 * std::cos(yaw);
                    forward_waypoint.pose.position.y += 0.5 * std::sin(yaw);
    
                    int forward_x_index = static_cast<int>((forward_waypoint.pose.position.x - costmap_->info.origin.position.x) / resolution);
                    int forward_y_index = static_cast<int>((forward_waypoint.pose.position.y - costmap_->info.origin.position.y) / resolution);
    
                    if (forward_x_index >= 0 && forward_x_index < static_cast<int>(costmap_->info.width) &&
                        forward_y_index >= 0 && forward_y_index < static_cast<int>(costmap_->info.height)) {
                        int forward_index = forward_y_index * costmap_->info.width + forward_x_index;
    
                        // Check if the forward waypoint is in a collision-free zone
                        if (costmap_->data[forward_index] <= obstacle_threshold_) {
                            // Both current and forward points are valid
                            return {forward_waypoint, was_adjusted};
                        }
                    }
                }
            }
    
            // Move both the current and forward waypoints 0.5 meters back
            adjusted_waypoint.pose.position.x -= 0.5 * std::cos(yaw);
            adjusted_waypoint.pose.position.y -= 0.5 * std::sin(yaw);
            was_adjusted = true;
        }
    
        // If no collision-free zone is found, return an invalid waypoint
        adjusted_waypoint.header.frame_id = ""; // Mark as invalid
        return {adjusted_waypoint, was_adjusted};
    }
    
    // Function to interpolate yaw between two poses
    tf2::Quaternion interpolateYaw(
        const geometry_msgs::msg::Pose &start_pose,
        const geometry_msgs::msg::Pose &goal_pose,
        float t) {
        tf2::Quaternion start_quat, goal_quat;
        tf2::fromMsg(start_pose.orientation, start_quat);
        tf2::fromMsg(goal_pose.orientation, goal_quat);
    
        double start_yaw = tf2::getYaw(start_quat);
        double goal_yaw = tf2::getYaw(goal_quat);
    
        double delta_yaw = goal_yaw - start_yaw;
        if (delta_yaw > M_PI) {
            delta_yaw -= 2 * M_PI; // Wrap around
        } else if (delta_yaw < -M_PI) {
            delta_yaw += 2 * M_PI; // Wrap around
        }
        double interpolated_yaw = start_yaw + t * delta_yaw;
    
        tf2::Quaternion interpolated_quat;
        interpolated_quat.setRPY(0, 0, interpolated_yaw);
        return interpolated_quat;
    }

    void planAndPublishPath() {
        if (!costmap_ || adjusted_waypoints_.poses.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot plan path: costmap or trajectory path is missing");
            return;
        }

        // Get the robot's current position in the costmap frame
        geometry_msgs::msg::PoseStamped current_position = getCurrentPosition();
        if (current_position.header.frame_id.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to retrieve current position");
            return;
        }
        // Initialize the planned path
        nav_msgs::msg::Path init_path;
        init_path.header.stamp = this->now();
        init_path.header.frame_id = costmap_->header.frame_id;

        // Add the current position to the path
        init_path.poses.push_back(current_position);

        // Add the adjusted waypoints to the path
        for (const auto &waypoint : adjusted_waypoints_.poses) {
            init_path.poses.push_back(waypoint);
        }
        // Make a raw path from the adjusted waypoints
        nav_msgs::msg::Path raw_path;
        raw_path.header.stamp = this->now();
        raw_path.header.frame_id = costmap_->header.frame_id;
        
        nav_msgs::msg::Path smoothed_path;
        int idx = -1;

        for (size_t i = 0; i < init_path.poses.size() - 1 && i < 4; ++i) { // Limit to 4 segments
            const auto &start = init_path.poses[i];
            const auto &goal = init_path.poses[i + 1];
    
            // Plan the path for the current segment
            auto segment_path = planPath(start, goal);

            // Fail-safe: Check if the segment path is empty
            if (segment_path.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to plan a valid path segment between waypoints %zu and %zu.", i, i + 1);
                path_invalid_flag_ = true; // Mark the path as invalid
                idx = i;
                break; // Stop further planning
            }
            path_invalid_flag_ = false; 
        
            // Add the segment path to the raw path
            raw_path.poses.insert(raw_path.poses.end(), segment_path.begin(), segment_path.end());
        }

        if (path_invalid_flag_ && idx == 0) {
            RCLCPP_ERROR(this->get_logger(), "Path planning failed. Marking the path as invalid and aborting.");
            return; // Abort without publishing
        }

        // Publish the raw path
        raw_path_pub_->publish(raw_path);

        // Smooth the raw path
        std::vector<geometry_msgs::msg::PoseStamped> smoothed_poses = smoothPath(raw_path.poses, interpolation_distance_);

        smoothed_path.header.stamp = this->now(); // Update the timestamp
        smoothed_path.header.frame_id = costmap_->header.frame_id; // Set the frame ID
        smoothed_path.poses = smoothed_poses;
        // Publish the planned path
        smoothed_path_pub_->publish(smoothed_path);
    }

    // Plan path
    std::vector<geometry_msgs::msg::PoseStamped> planPath(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal) {
        if (!costmap_) {
            RCLCPP_ERROR(this->get_logger(), "No costmap available");
            return {};
        }
        bool invalid_flag = false;
    
        int width = costmap_->info.width;
        int height = costmap_->info.height;
        float resolution = costmap_->info.resolution;
    
        auto toIndex = [&](int x, int y) { return y * width + x; };
    
        auto heuristic = [&](int x1, int y1, int x2, int y2) {
            return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
        };
    
        // Interpolate intermediate waypoints if needed
        std::vector<geometry_msgs::msg::PoseStamped> waypoints = {start};
        float distance = std::sqrt(
            std::pow(goal.pose.position.x - start.pose.position.x, 2) +
            std::pow(goal.pose.position.y - start.pose.position.y, 2) +
            std::pow(goal.pose.position.z - start.pose.position.z, 2));
    
        if (distance > interpolation_distance_) {
            int num_intermediate_points = static_cast<int>(std::ceil(distance / interpolation_distance_));

            for (int i = 1; i <= num_intermediate_points; ++i) {
                float t = static_cast<float>(i) / (num_intermediate_points + 1);
                geometry_msgs::msg::PoseStamped intermediate;
                intermediate.header.frame_id = costmap_->header.frame_id;
                intermediate.pose.position.x = start.pose.position.x + t * (goal.pose.position.x - start.pose.position.x);
                intermediate.pose.position.y = start.pose.position.y + t * (goal.pose.position.y - start.pose.position.y);
                intermediate.pose.position.z = start.pose.position.z + t * (goal.pose.position.z - start.pose.position.z);
                
                tf2::Quaternion quaternion = interpolateYaw(start.pose, goal.pose, t);
                intermediate.pose.orientation = tf2::toMsg(quaternion);
    
                // Adjust the waypoint for collision-free zones
                auto [adjusted_intermediate, was_adjusted] = adjustWaypointForCollision(intermediate, resolution, 20);
                if (adjusted_intermediate.header.frame_id.empty()) {
                    invalid_flag = true;
                    break; // Stop processing if the waypoint is invalid
                } else {
                    if (was_adjusted) {
                        waypoints.push_back(adjusted_intermediate);
                    }
                }
            }
        }
        waypoints.push_back(goal);
        if (!invalid_flag) {
            return waypoints; // Return the waypoints if all are valid
        }
        // If any waypoint is invalid, use A*
        std::vector<geometry_msgs::msg::PoseStamped> full_path;

        if (waypoints.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Not enough waypoints to plan a path");
            return {};
        }
        
        int start_x = static_cast<int>((start.pose.position.x - costmap_->info.origin.position.x) / resolution);
        int start_y = static_cast<int>((start.pose.position.y - costmap_->info.origin.position.y) / resolution);
        int goal_x = static_cast<int>((goal.pose.position.x - costmap_->info.origin.position.x) / resolution);
        int goal_y = static_cast<int>((goal.pose.position.y - costmap_->info.origin.position.y) / resolution);
        
        // A* algorithm
        std::priority_queue<PlannerNode, std::vector<PlannerNode>, std::greater<PlannerNode>> open_list;
        std::unordered_map<int, int> came_from;
        std::unordered_map<int, float> cost_so_far;
        
        open_list.push({start_x, start_y, 0});
        cost_so_far[toIndex(start_x, start_y)] = 0;
        
        std::vector<int> dx = {1, -1, 0, 0};
        std::vector<int> dy = {0, 0, 1, -1};
        
        while (!open_list.empty()) {
            PlannerNode current = open_list.top();
            open_list.pop();
        
            if (current.x == goal_x && current.y == goal_y) {
                break;
            }
        
            for (size_t j = 0; j < dx.size(); ++j) {
                int next_x = current.x + dx[j];
                int next_y = current.y + dy[j];
        
                if (next_x < 0 || next_y < 0 || next_x >= width || next_y >= height) {
                    continue;
                }
        
                int index = toIndex(next_x, next_y);
                if (costmap_->data[index] > obstacle_threshold_) {
                    continue; // Skip this cell as it is an obstacle
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
        int current_index = toIndex(goal_x, goal_y);
        
        while (came_from.count(current_index)) {
            int x = current_index % width;
            int y = current_index / width;
        
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = costmap_->header.frame_id;
        
            pose.pose.position.x = x * resolution + costmap_->info.origin.position.x;
            pose.pose.position.y = y * resolution + costmap_->info.origin.position.y;
        
            float dx = goal.pose.position.x - start.pose.position.x;
            float dy = goal.pose.position.y - start.pose.position.y;
            float dz = goal.pose.position.z - start.pose.position.z;
        
            float distance_to_start_2d = std::sqrt(
                std::pow(pose.pose.position.x - start.pose.position.x, 2) +
                std::pow(pose.pose.position.y - start.pose.position.y, 2));
            float total_distance_2d = std::sqrt(dx * dx + dy * dy);
        
            if (total_distance_2d > 0.0) {
                float t = std::clamp(distance_to_start_2d / total_distance_2d, 0.0f, 1.0f);
                pose.pose.position.z = start.pose.position.z + t * dz;
        
                tf2::Quaternion quaternion = interpolateYaw(start.pose, goal.pose, t);
                pose.pose.orientation = tf2::toMsg(quaternion);
            } else {
                pose.pose.position.z = start.pose.position.z + dz;
                pose.pose.orientation = goal.pose.orientation;
            }
        
            full_path.push_back(pose);
            current_index = came_from[current_index];
        }
        
        full_path.push_back(start);
        std::reverse(full_path.begin(), full_path.end());

        // Check if the path is valid
        if (full_path.size() <= 2) {
            RCLCPP_ERROR(this->get_logger(), "A* failed to find a valid path. Only start and end points are available.");
            return {}; // Return an empty path to indicate failure
        }

        // Downsample the path
        full_path = downsamplePath(full_path, interpolation_distance_*2);
        
        return full_path;
    }

    std::vector<geometry_msgs::msg::PoseStamped> downsamplePath(
        const std::vector<geometry_msgs::msg::PoseStamped> &path, double min_distance) {
        if (path.size() < 2) {
            return path; // Not enough points to downsample
        }
    
        std::vector<geometry_msgs::msg::PoseStamped> downsampled_path;
        downsampled_path.push_back(path.front()); // Always include the first point
    
        for (size_t i = 1; i < path.size(); ++i) {
            const auto &last_point = downsampled_path.back().pose.position;
            const auto &current_point = path[i].pose.position;
    
            // Calculate the Euclidean distance between the last added point and the current point
            double distance = std::sqrt(
                std::pow(current_point.x - last_point.x, 2) +
                std::pow(current_point.y - last_point.y, 2) +
                std::pow(current_point.z - last_point.z, 2));
    
            if (distance >= min_distance) {
                downsampled_path.push_back(path[i]); // Add the point if it's far enough
            }
        }
    
        return downsampled_path;
    }
    

    std::vector<geometry_msgs::msg::PoseStamped> smoothPath(
        const std::vector<geometry_msgs::msg::PoseStamped> &path, double interpolation_distance) {
        if (path.size() < 2) {
            // Not enough points to smooth
            return path;
        }
    
        // Extract x, y, z, and yaw values
        std::vector<double> x, y, z, yaw;
        for (const auto &pose : path) {
            x.push_back(pose.pose.position.x);
            y.push_back(pose.pose.position.y);
            z.push_back(pose.pose.position.z);
    
            // Extract yaw from the orientation
            tf2::Quaternion quat;
            tf2::fromMsg(pose.pose.orientation, quat);
            yaw.push_back(tf2::getYaw(quat));
        }
    
        // Generate a parameter t for interpolation (e.g., cumulative distance)
        std::vector<double> t(x.size(), 0.0);
        for (size_t i = 1; i < x.size(); ++i) {
            t[i] = t[i - 1] + std::sqrt(
                std::pow(x[i] - x[i - 1], 2) +
                std::pow(y[i] - y[i - 1], 2) +
                std::pow(z[i] - z[i - 1], 2));
        }
    
        // Normalize t to [0, total_length]
        double total_length = t.back();
    
        // Calculate the number of points based on the desired spacing
        int num_points = static_cast<int>(std::ceil(total_length / interpolation_distance));
    
        // Generate new parameter values for the smoothed path
        std::vector<double> t_new(num_points);
        for (int i = 0; i < num_points; ++i) {
            t_new[i] = i * interpolation_distance;
        }
    
        // Interpolate x, y, z, and yaw using cubic splines
        auto cubicSpline = [](const std::vector<double> &t, const std::vector<double> &values, const std::vector<double> &t_new, bool is_yaw = false) {
            std::vector<double> result(t_new.size());
            for (size_t i = 0; i < t_new.size(); ++i) {
                auto it = std::lower_bound(t.begin(), t.end(), t_new[i]);
                size_t idx = std::distance(t.begin(), it);
                if (idx == 0) {
                    result[i] = values[0];
                } else if (idx >= t.size()) {
                    result[i] = values.back();
                } else {
                    double t1 = t[idx - 1], t2 = t[idx];
                    double v1 = values[idx - 1], v2 = values[idx];
    
                    if (is_yaw) {
                        // Handle angle wrapping for yaw
                        double delta_yaw = v2 - v1;
                        if (delta_yaw > M_PI) {
                            delta_yaw -= 2 * M_PI;
                        } else if (delta_yaw < -M_PI) {
                            delta_yaw += 2 * M_PI;
                        }
                        double interpolated_yaw = v1 + (delta_yaw * (t_new[i] - t1) / (t2 - t1));
                        result[i] = std::fmod(interpolated_yaw + M_PI, 2 * M_PI) - M_PI; // Wrap to [-π, π]
                    } else {
                        // Regular interpolation
                        result[i] = v1 + (v2 - v1) * (t_new[i] - t1) / (t2 - t1);
                    }
                }
            }
            return result;
        };
    
        std::vector<double> x_smooth = cubicSpline(t, x, t_new);
        std::vector<double> y_smooth = cubicSpline(t, y, t_new);
        std::vector<double> z_smooth = cubicSpline(t, z, t_new);
        std::vector<double> yaw_smooth = cubicSpline(t, yaw, t_new, true);
    
        // Reconstruct the smoothed path
        std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;

        smoothed_path.push_back(path.front());

        for (size_t i = 1; i < t_new.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.front().header; // Use the same header
            pose.pose.position.x = x_smooth[i];
            pose.pose.position.y = y_smooth[i];
            pose.pose.position.z = z_smooth[i];
    
            // Use the interpolated yaw
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, yaw_smooth[i]);
            pose.pose.orientation = tf2::toMsg(quaternion);
    
            // Check if the current pose matches an adjusted waypoint
            auto adjusted_it = std::find_if(
                adjusted_waypoints_.poses.begin(), adjusted_waypoints_.poses.end(),
                [&pose, interpolation_distance](const geometry_msgs::msg::PoseStamped &adjusted_pose) {
                    return std::sqrt(
                        std::pow(pose.pose.position.x - adjusted_pose.pose.position.x, 2) +
                        std::pow(pose.pose.position.y - adjusted_pose.pose.position.y, 2) +
                        std::pow(pose.pose.position.z - adjusted_pose.pose.position.z, 2)) <= interpolation_distance;
                });
    
            if (adjusted_it != adjusted_waypoints_.poses.end()) {
                // Replace with the adjusted waypoint
                smoothed_path.push_back(*adjusted_it);
            } else {
                // Add the interpolated pose
                smoothed_path.push_back(pose);
            }
        }
        return smoothed_path;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}