#pragma once

#include <atomic>
#include <stdint.h>
#include <string>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace nonlin {
class offboard_common : public rclcpp::Node {
  public:
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

  private:
    uint64_t offboard_setpoint_counter{}; //!< counter for the number of setpoints sent

    float x_pos = 0.f;
    float y_pos = 0.f;
    float z_pos = -2.3f;
    bool mode_switched = false;

    // Maybe move to 
    std::atomic<float> current_x = 0.f;
    std::atomic<float> current_y = 0.f;
    std::atomic<float> current_z = 0.f;

    rclcpp::TimerBase::SharedPtr timer{};
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher{};
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher{};
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher{};

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr local_position_subscription{};

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

    /// @brief Used to track the current position of the vehicle
    void local_position_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    // Maybe list experiments here as callables?
};
}; // namespace nonlin