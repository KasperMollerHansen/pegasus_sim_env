#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <eigen3/Eigen/Dense> // For Eigen::Vector3d
#include <cmath>
#include <chrono>
#include <iostream>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace nav_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control"), armed_(false)
    {
        this->declare_parameter<double>("interpolation_distance", 2.0);
        this->declare_parameter<std::string>("frame_id", "base_link");
        this->declare_parameter<double>("speed_limit", 4.0);
        this->declare_parameter<double>("min_speed", 1.5);
        this->declare_parameter<double>("max_acceleration", 0.05);

        interpolation_distance_ = this->get_parameter("interpolation_distance").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        speed_limit_ = this->get_parameter("speed_limit").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();
        max_acceleration_ = this->get_parameter("max_acceleration").as_double();

        // Publishers
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // TF2 setup
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Define QoS profile for the subscriber
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        // Create subscriber for vehicle control mode (to check the armed status)
        arming_status_subscriber_ = this->create_subscription<VehicleControlMode>(
            "/fmu/out/vehicle_control_mode", qos_profile, [this](const VehicleControlMode::SharedPtr msg) {
                armed_ = msg->flag_armed;
                if (armed_) {
                    RCLCPP_INFO(this->get_logger(), "Drone is armed.");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Drone is disarmed.");
                }
            });

        // Subscribe to the Path topic
        path_subscriber_ = this->create_subscription<Path>(
            "/planned_path", qos_profile, [this](const Path::SharedPtr msg) {
                process_path(msg);
            });
    }

    void arm();
    void disarm();

private:
    bool armed_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<VehicleControlMode>::SharedPtr arming_status_subscriber_;
    rclcpp::Subscription<Path>::SharedPtr path_subscriber_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    void publish_offboard_control_mode_velocity();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void process_path(const Path::SharedPtr msg);
    std::string frame_id_;
    double interpolation_distance_;
    double speed_limit_;
    double min_speed_;
    double max_acceleration_;
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

/**
 * @brief Publish the offboard control mode for velocity control.
 */
void OffboardControl::publish_offboard_control_mode_velocity()
{
    OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = true;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published Offboard Control Mode (Velocity)");
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published Vehicle Command: %d", command);
}

void OffboardControl::process_path(const Path::SharedPtr msg)
{
    static Eigen::Vector3d previous_velocity(0.0, 0.0, 0.0); // Track the previous velocity
    double dt = 0.1; // Time difference between updates (can be adjusted dynamically)

    // TF2 setup
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        // Get the transform from the drone's frame (e.g., base_link) to the world frame (e.g., map)
        transform_stamped = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform base_link to odom: %s", ex.what());
        return;
    }
    // Transform to ned frame
    Eigen::Vector3d pos_tf(
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.x,
        -transform_stamped.transform.translation.z);

    if (msg->poses.size() >= 2) { // Need at least two poses for velocity and yaw from pose 1
        RCLCPP_INFO(this->get_logger(), "Received Path with %zu poses", msg->poses.size());

        const auto &pose0 = msg->poses[0];
        const auto &pose1 = msg->poses[1];

        // Calculate velocity vector from pose0 to pose1
        // Transform pose1 to ned frame
        Eigen::Vector3d pos0(pose0.pose.position.y, pose0.pose.position.x, -pose0.pose.position.z);
        Eigen::Vector3d pos1(pose1.pose.position.y, pose1.pose.position.x, -pose1.pose.position.z);
        
        Eigen::Vector3d velocity_desired_world; // Declare the variable before the if-else block

        if ((pos_tf - pos0).norm() > interpolation_distance_) {
            velocity_desired_world = (pos0 - pos_tf) / dt; // Move to pose0 if pose0 diverges from tf
        } else {
            velocity_desired_world = (pos1 - pos_tf) / dt; // Move to next pose
        }

        // Limit the magnitude of the desired velocity
        double desired_speed = velocity_desired_world.norm();
        if (desired_speed > speed_limit_ && desired_speed > 1e-6) {
            velocity_desired_world = velocity_desired_world / desired_speed * speed_limit_;
        } else if (desired_speed < min_speed_ && desired_speed > 1e-6) {
            velocity_desired_world = velocity_desired_world / desired_speed * 1.0;
        }

        // Calculate acceleration
        Eigen::Vector3d acceleration_desired_world = (velocity_desired_world - previous_velocity) / dt;

        // Limit the magnitude of the acceleration
        double acceleration_magnitude = acceleration_desired_world.norm();
        if (acceleration_magnitude > max_acceleration_ && acceleration_magnitude > 1e-6) {
            acceleration_desired_world = acceleration_desired_world / acceleration_magnitude * max_acceleration_;
        }

        // Extract yaw from the NEXT pose (pose1)
        tf2::Quaternion q_next(
            pose1.pose.orientation.x,
            pose1.pose.orientation.y,
            pose1.pose.orientation.z,
            pose1.pose.orientation.w);
        tf2::Matrix3x3 m_next(q_next);
        double roll_next, pitch_next, yaw_next;
        m_next.getRPY(roll_next, pitch_next, yaw_next);

        // Create and publish the TrajectorySetpoint message for velocity control
        TrajectorySetpoint setpoint_msg{};
        // setpoint_msg.position = {
        //     static_cast<float>(pos1.y()), // y
        //     static_cast<float>(pos1.x()), // x
        //     static_cast<float>(pos1.z()) // -z
        // };

        setpoint_msg.position = {NAN, NAN, NAN};
        
        setpoint_msg.velocity = {
            static_cast<float>(velocity_desired_world.x()),
            static_cast<float>(velocity_desired_world.y()),
            static_cast<float>(velocity_desired_world.z())
        };

        setpoint_msg.acceleration = {
            static_cast<float>(acceleration_desired_world.x()),
            static_cast<float>(acceleration_desired_world.y()),
            static_cast<float>(acceleration_desired_world.z())
        };

        setpoint_msg.yaw = static_cast<float>(-yaw_next + M_PI / 2.0); // Use yaw from pose1
        setpoint_msg.yawspeed = NAN;

        publish_offboard_control_mode_velocity();
        trajectory_setpoint_publisher_->publish(setpoint_msg);
        RCLCPP_INFO(this->get_logger(), "Published Velocity Setpoint: [%f, %f, %f] m/s, Yaw: %f rad",
                    setpoint_msg.velocity[0], setpoint_msg.velocity[1], setpoint_msg.velocity[2], setpoint_msg.yaw);

        // Arm the drone if it's not armed
        if (!armed_) {
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            arm();
        }
    } else if (msg->poses.size() == 1) {
        RCLCPP_WARN(this->get_logger(), "Received Path message with only one pose. Sending zero velocity and current yaw.");
        const auto &current_pose = msg->poses[0];
        TrajectorySetpoint setpoint_msg{};
        setpoint_msg.velocity = {0.0f, 0.0f, 0.0f};
        setpoint_msg.position = {NAN, NAN, NAN};
        setpoint_msg.yaw = static_cast<float>(-tf2::getYaw(tf2::Quaternion(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w)) + M_PI / 2.0);
        setpoint_msg.yawspeed = NAN;
        publish_offboard_control_mode_velocity();
        trajectory_setpoint_publisher_->publish(setpoint_msg);
    } else {
        RCLCPP_WARN(this->get_logger(), "Received empty Path message");
    }
}

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OffboardControl>();
    RCLCPP_INFO(node->get_logger(), "Starting Offboard Control Node (Velocity)...");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}