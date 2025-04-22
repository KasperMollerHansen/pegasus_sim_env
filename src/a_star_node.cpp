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
            current_position.pose.orientation = transform.transform.rotation;
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
                RCLCPP_ERROR(this->get_logger(), "Failed to plan a path for segment %zu", i);
                break; // Stop planning further segments
            }

            // Concatenate the segment to the planned path
            planned_path.poses.insert(planned_path.poses.end(), segment_path.begin(), segment_path.end());

            // Update the start for the next segment
            start = goal;
        }

        // Publish the planned path up to the last successful segment
        if (!planned_path.poses.empty()) {
            // planned_path.poses = smoothPath(planned_path.poses, interpolation_distance_);
            path_pub_->publish(planned_path);
            RCLCPP_INFO(this->get_logger(), "Published planned path with %zu poses", planned_path.poses.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "No valid path segments to publish");
        }
    }

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

    // Function to check and adjust waypoints for collision-free zones
    geometry_msgs::msg::PoseStamped adjustWaypointForCollision(
        const geometry_msgs::msg::PoseStamped &waypoint, float resolution, int max_attempts) {
        geometry_msgs::msg::PoseStamped adjusted_waypoint = waypoint;

        // Extract yaw from the waypoint's quaternion only after the bounds check
        tf2::Quaternion quat;
        tf2::fromMsg(adjusted_waypoint.pose.orientation, quat);
        double yaw = tf2::getYaw(quat);

        for (int attempt = 0; attempt < max_attempts; ++attempt) {
            // Convert waypoint position to costmap indices
            int x_index = static_cast<int>((adjusted_waypoint.pose.position.x - costmap_->info.origin.position.x) / resolution);
            int y_index = static_cast<int>((adjusted_waypoint.pose.position.y - costmap_->info.origin.position.y) / resolution);

            // Check if the waypoint is within bounds
            if (x_index >= 0 && x_index < static_cast<int>(costmap_->info.width) &&
                y_index >= 0 && y_index < static_cast<int>(costmap_->info.height)) {
                int index = y_index * costmap_->info.width + x_index;

                // Check if the waypoint is in a collision-free zone
                if (costmap_->data[index] <= obstacle_threshold_) {
                    return adjusted_waypoint; // Collision-free waypoint found
                }
            }
        
            // Move the waypoint in the negative yaw direction by 0.5 meters
            adjusted_waypoint.pose.position.x -= 0.5 * std::cos(yaw);
            adjusted_waypoint.pose.position.y -= 0.5 * std::sin(yaw);
        }

        // If no collision-free zone is found, return an invalid waypoint
        adjusted_waypoint.header.frame_id = ""; // Mark as invalid
        return adjusted_waypoint;
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
    
        // Adjust the start and goal waypoints for collision-free zones
        geometry_msgs::msg::PoseStamped adjusted_start = adjustWaypointForCollision(start, resolution, 20);
        geometry_msgs::msg::PoseStamped adjusted_goal = adjustWaypointForCollision(goal, resolution, 20);
    
        if (adjusted_start.header.frame_id.empty() || adjusted_goal.header.frame_id.empty()) {
            RCLCPP_WARN(this->get_logger(), "Start or goal waypoint could not be adjusted to a collision-free zone");
            return {};
        }
    
        // Interpolate intermediate waypoints if needed
        std::vector<geometry_msgs::msg::PoseStamped> waypoints = {adjusted_start};
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
                geometry_msgs::msg::PoseStamped adjusted_intermediate = adjustWaypointForCollision(intermediate, resolution, 20);
                if (!adjusted_intermediate.header.frame_id.empty()) {
                    waypoints.push_back(adjusted_intermediate);
                }
            }
        }
        waypoints.push_back(adjusted_goal);

        // Plan path between consecutive waypoints using A*
        std::vector<geometry_msgs::msg::PoseStamped> full_path;
        geometry_msgs::msg::PoseStamped current_start = waypoints.front();
        for (size_t i = 1; i < waypoints.size(); ++i) {
            const auto &current_goal = waypoints[i];
    
            int start_x = static_cast<int>((current_start.pose.position.x - costmap_->info.origin.position.x) / resolution);
            int start_y = static_cast<int>((current_start.pose.position.y - costmap_->info.origin.position.y) / resolution);
            int goal_x = static_cast<int>((current_goal.pose.position.x - costmap_->info.origin.position.x) / resolution);
            int goal_y = static_cast<int>((current_goal.pose.position.y - costmap_->info.origin.position.y) / resolution);
    
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
    
                for (size_t j = 0; j < dx.size(); ++j) {
                    int next_x = current.x + dx[j];
                    int next_y = current.y + dy[j];
    
                    if (next_x < 0 || next_y < 0 || next_x >= width || next_y >= height) {
                        continue;
                    }
    
                    int index = toIndex(next_x, next_y);
                    if (costmap_->data[index] > obstacle_threshold_) {
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
            int current_index = toIndex(goal_x, goal_y);
    
            while (came_from.count(current_index)) {
                int x = current_index % width;
                int y = current_index / width;
            
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = costmap_->header.frame_id;
            
                // Calculate the x and y positions
                pose.pose.position.x = x * resolution + costmap_->info.origin.position.x;
                pose.pose.position.y = y * resolution + costmap_->info.origin.position.y;
            
                // Calculate the interpolation factor (t) based on the 2D distance
                float dx = goal.pose.position.x - start.pose.position.x;
                float dy = goal.pose.position.y - start.pose.position.y;
                float dz = goal.pose.position.z - start.pose.position.z;
                
            
                float distance_to_start_2d = std::sqrt(
                    std::pow(pose.pose.position.x - start.pose.position.x, 2) +
                    std::pow(pose.pose.position.y - start.pose.position.y, 2));
                float total_distance_2d = std::sqrt(dx * dx + dy * dy);

                // Ensure valid interpolation
                if (total_distance_2d > 0.0) {
                    float t = std::clamp(distance_to_start_2d / total_distance_2d, 0.0f, 1.0f); // Clamp t to [0, 1]
                    pose.pose.position.z = start.pose.position.z + t * dz;
                    
                    tf2::Quaternion quaternion = interpolateYaw(start.pose, goal.pose, t);
                    pose.pose.orientation = tf2::toMsg(quaternion);
                } else {
                    // If no horizontal movement, directly interpolate based on vertical distance
                    pose.pose.position.z = start.pose.position.z + dz;
                    pose.pose.orientation = goal.pose.orientation; 
                }

                segment_path.push_back(pose);
                current_index = came_from[current_index];
            }
    
            segment_path.push_back(current_start);
            std::reverse(segment_path.begin(), segment_path.end());
            full_path.insert(full_path.end(), segment_path.begin(), segment_path.end());
            current_start = current_goal;
        }
    
        return full_path;
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> smoothPath(
        const std::vector<geometry_msgs::msg::PoseStamped> &path, double interpolation_distance) {
        if (path.size() < 2) {
            // Not enough points to smooth
            return path;
        }
    
        // Simplify the path using the Ramer-Douglas-Peucker algorithm
        auto simplifyPath = [](const std::vector<geometry_msgs::msg::PoseStamped> &path, double epsilon) {
            if (path.size() < 3) {
                return path; // No simplification needed
            }
    
            std::vector<bool> keep(path.size(), false);
            keep.front() = true;  // Always keep the first point
            keep.back() = true;   // Always keep the last point
    
            std::function<void(size_t, size_t)> rdp = [&](size_t start, size_t end) {
                if (start + 1 >= end) {
                    return; // No points to simplify
                }
    
                // Find the point farthest from the line segment [start, end]
                double max_distance = 0.0;
                size_t farthest_index = start;
    
                const auto &start_point = path[start].pose.position;
                const auto &end_point = path[end].pose.position;
    
                for (size_t i = start + 1; i < end; ++i) {
                    const auto &current_point = path[i].pose.position;
    
                    // Calculate perpendicular distance to the line
                    double num = std::abs((end_point.y - start_point.y) * current_point.x -
                                          (end_point.x - start_point.x) * current_point.y +
                                          end_point.x * start_point.y - end_point.y * start_point.x);
                    double den = std::sqrt(std::pow(end_point.y - start_point.y, 2) +
                                           std::pow(end_point.x - start_point.x, 2));
                    double distance = num / den;
    
                    if (distance > max_distance) {
                        max_distance = distance;
                        farthest_index = i;
                    }
                }
    
                // If the farthest point is farther than epsilon, keep it
                if (max_distance > epsilon) {
                    keep[farthest_index] = true;
                    rdp(start, farthest_index);
                    rdp(farthest_index, end);
                }
            };
    
            rdp(0, path.size() - 1);
    
            // Build the simplified path
            std::vector<geometry_msgs::msg::PoseStamped> simplified_path;
            for (size_t i = 0; i < path.size(); ++i) {
                if (keep[i]) {
                    simplified_path.push_back(path[i]);
                }
            }
    
            return simplified_path;
        };
    
        // Simplify the path with a chosen epsilon value
        double epsilon = interpolation_distance; // Adjust epsilon as needed
        std::vector<geometry_msgs::msg::PoseStamped> simplified_path = simplifyPath(path, epsilon);
    
        // Smooth the simplified path using cubic spline interpolation
        std::vector<double> x, y, z;
        for (const auto &pose : simplified_path) {
            x.push_back(pose.pose.position.x);
            y.push_back(pose.pose.position.y);
            z.push_back(pose.pose.position.z);
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
    
        // Perform cubic spline interpolation
        auto cubicSpline = [](const std::vector<double> &t, const std::vector<double> &values, const std::vector<double> &t_new) {
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
                    result[i] = v1 + (v2 - v1) * (t_new[i] - t1) / (t2 - t1);
                }
            }
            return result;
        };
    
        std::vector<double> x_smooth = cubicSpline(t, x, t_new);
        std::vector<double> y_smooth = cubicSpline(t, y, t_new);
        std::vector<double> z_smooth = cubicSpline(t, z, t_new);
    
        // Reconstruct the smoothed path
        std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;
        for (size_t i = 0; i < t_new.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = simplified_path.front().header; // Use the same header
            pose.pose.position.x = x_smooth[i];
            pose.pose.position.y = y_smooth[i];
            pose.pose.position.z = z_smooth[i];
    
            // Recalculate orientation
            if (i > 0) {
                double dx = x_smooth[i] - x_smooth[i - 1];
                double dy = y_smooth[i] - y_smooth[i - 1];
                double yaw = std::atan2(dy, dx);
                tf2::Quaternion quaternion;
                quaternion.setRPY(0, 0, yaw);
                pose.pose.orientation = tf2::toMsg(quaternion);
            } else {
                pose.pose.orientation = simplified_path.front().pose.orientation; // Use the first pose's orientation
            }
    
            smoothed_path.push_back(pose);
        }
    
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