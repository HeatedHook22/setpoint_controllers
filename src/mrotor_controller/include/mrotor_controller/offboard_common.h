#pragma once

#include <Eigen/Dense>
#include <stdint.h>
#include <string>

#include <px4_msgs/msg/hover_thrust_estimate.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

namespace nonlin {
class offboard_common : public rclcpp::Node {
  public:
    // Don't really care about yaw
    const float yaw_sp = 0.f;

    enum offboard_control_mode_enums : uint8_t {
        NONE = 0b0,
        POSITION = 0b1,
        VELOCITY = 0b10,
        ACCELERATION = 0b100,
        ATTITUDE = 0b1000,
        BODY_RATE = 0b10000
    };

    // Constructors
    offboard_common(std::string node_name);
    ~offboard_common() = default;

    /// @brief Override in subclass with setpoint/control mode that you want
    virtual void offboard_control_mode_logic() = 0; // Pure virtual

    /**
     * @brief Publish the offboard control mode. Bitwise OR (|) the offboard_control_mode_enums together to enable specific modes
     */
    void publish_offboard_control_mode(uint8_t offboard_control_mode);

    Eigen::Vector3f position_control(const Eigen::Vector3f &pos_sp);

    Eigen::Vector3f velocity_control(const Eigen::Vector3f &vel_sp);

    std::pair<Eigen::Quaternionf, Eigen::Vector3f> acceleration_control(const Eigen::Vector3f &acc_sp);

  private:
    uint64_t offboard_setpoint_counter{}; //!< counter for the number of setpoints sent

    bool mode_switched = false;
    float hover_thrust_estimate = 0.7275f; // Default fallback (from testing), QGroundControl says 0.6

    Eigen::Vector3f initial_pos = {0.f, 0.f, -2.5f};
    std::chrono::time_point<std::chrono::high_resolution_clock> prev_time{};
    std::chrono::time_point<std::chrono::high_resolution_clock> now{}; // Current time of program loop
    float dt = 0.f;

    // Position controller PID terms
    const float kp_pos = 0.5f;
    Eigen::Vector3f prev_pos_sp{};

    // Velocity controller PID terms
    const float kp_vel = 0.05f;
    const float ki_vel = 0.01f;
    const float kd_vel = 0.f;
    Eigen::Vector3f integral_acc_error{};
    Eigen::Vector3f prev_vel_sp{};
    Eigen::Vector3f prev_vel{};

    // Acceleration controller terms
    const Eigen::Vector3f gravity_vector{0.0f, 0.0f, 9.81f};

    // Current vehicle measurements
    Eigen::Vector3f current_pos{};
    Eigen::Vector3f current_vel{};
    Eigen::Vector3f current_acc{};

    // Current error measurements
    Eigen::Vector3f pos_error{};
    Eigen::Vector3f vel_error{};
    Eigen::Vector3f acc_error{};

    rclcpp::TimerBase::SharedPtr timer{};
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher{};
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher{};
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher{};

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscription{};
    rclcpp::Subscription<px4_msgs::msg::HoverThrustEstimate>::SharedPtr hover_thrust_estimate_subscription{};

    // Publish commands taken from offboard_control.cpp example from PX4
    /**
     * @brief Publish vehicle commands
     * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD
     * codes)
     * @param param1    Command parameter 1
     * @param param2    Command parameter 2
     */
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0,
                                 float param6 = 0.0, float param7 = 0.0);

    /**
     * @brief Publish a trajectory setpoint
     */
    void publish_trajectory_setpoint(float x, float y, float z, float yaw = -M_PI);

    /**
     * @brief Send a command to Arm the vehicle
     */
    void arm();

    /**
     * @brief Send a command to Disarm the vehicle
     */
    void disarm();

    /// @brief Used to track the current position/velocity/acceleration of the vehicle
    void get_kinematics_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

    /// @brief Used to track the estimated hover thrust of the vehicle
    void get_hover_thrust_estimate_callback(const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg);

    // Maybe list experiments here as callables?
};
}; // namespace nonlin