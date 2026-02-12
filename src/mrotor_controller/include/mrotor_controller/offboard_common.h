#pragma once

#include <stdint.h>
#include <string>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

namespace nonlin {
class offboard_common : public rclcpp::Node {
  public:
    offboard_common(std::string node_name);
    ~offboard_common() = default;

  private:
    uint64_t offboard_setpoint_counter{}; //!< counter for the number of
                                          //!< setpoints sent
    rclcpp::TimerBase::SharedPtr timer{};
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher{};
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher{};
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher{};
    enum offboard_control_mode_enums : uint8_t {
        NONE = 0b0,
        POSITION = 0b1,
        VELOCITY = 0b10,
        ACCELERATION = 0b100,
        ATTITUDE = 0b1000,
        BODY_RATE = 0b10000
    };

    // Publish commands taken from offboard_control.cpp example from PX4
    /**
     * @brief Publish vehicle commands
     * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD
     * codes)
     * @param param1    Command parameter 1
     * @param param2    Command parameter 2
     */
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    /**
     * @brief Publish a trajectory setpoint
     */
    void publish_trajectory_setpoint(float x, float y, float z, float yaw = -M_PI);

    /**
     * @brief Publish the offboard control mode. Bitwise OR (|) the offboard_control_mode_enums together to enable specific modes
     */
    void publish_offboard_control_mode(uint8_t offboard_control_mode);

    /**
     * @brief Send a command to Arm the vehicle
     */
    void arm();

    /**
     * @brief Send a command to Disarm the vehicle
     */
    void disarm();

    /// @brief Enable offboard control
    void offboard_enable(bool enable);

    /// @brief Keep alive call/timer
    void offboard_keep_alive();

    /// @brief Used for logging offboard controls
    void offboard_log();

    // Maybe list experiments here as callables?
};
}; // namespace nonlin