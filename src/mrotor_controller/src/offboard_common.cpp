#include <cassert>
#include <chrono>
#include <cmath>
#include <functional>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mrotor_controller/offboard_common.h"

namespace nonlin {
// ------ Public ------ //
offboard_common::offboard_common(std::string node_name) : rclcpp::Node(node_name) {
    // Constructor mostly from offboard_control.cpp example from PX4
    offboard_control_mode_publisher = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    // Create a QoS profile that matches PX4 (Best Effort)
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    local_position_subscription = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos, std::bind(&offboard_common::local_position_callback, this, std::placeholders::_1));

    auto timer_callback = [this]() -> void {
        if (offboard_setpoint_counter == 10) {
            // Change to Offboard mode after 10 setpoints
            this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

            // Arm the vehicle
            this->arm();
            RCLCPP_INFO(this->get_logger(), "Moving to intial flight position");
        }

        // Stop the counter after reaching 11
        if (offboard_setpoint_counter < 11) {
            offboard_setpoint_counter++;
        }

        if (!mode_switched && !((std::abs(current_x - x_pos) < 0.1) && (std::abs(current_y - y_pos) < 0.1) &&
                                (std::abs(current_z - z_pos) < 0.1))) { // Within 0.1m radius
            // OffboardControlMode needs to be paired with TrajectorySetpoint (default)
            publish_offboard_control_mode(static_cast<uint8_t>(POSITION));
            publish_trajectory_setpoint(x_pos, y_pos, z_pos);
        } else {
            // Offboard control mode and setpoint control/logic
            if (!mode_switched) {
                RCLCPP_INFO(this->get_logger(), "Switching from trajectory setpoints to custom setpoint control");
                mode_switched = true;
            }
            this->offboard_control_mode_logic();
        }
    };
    timer = this->create_wall_timer(std::chrono::milliseconds(50), timer_callback);
}

void offboard_common::publish_offboard_control_mode(uint8_t offboard_control_mode) {
    assert(offboard_control_mode <= static_cast<uint8_t>(POSITION | VELOCITY | ACCELERATION | ATTITUDE | BODY_RATE));

    px4_msgs::msg::OffboardControlMode msg{};

    // No bit shift needed to decide between true or false (i.e. 0 or not 0)
    msg.position = static_cast<bool>(offboard_control_mode & POSITION);
    msg.velocity = static_cast<bool>(offboard_control_mode & VELOCITY);
    msg.acceleration = static_cast<bool>(offboard_control_mode & ACCELERATION);
    msg.attitude = static_cast<bool>(offboard_control_mode & ATTITUDE);
    msg.body_rate = static_cast<bool>(offboard_control_mode & BODY_RATE);
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher->publish(msg);
}

// ------ Private ------ //
void offboard_common::publish_vehicle_command(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6,
                                              float param7) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher->publish(msg);
}

void offboard_common::publish_trajectory_setpoint(float x, float y, float z, float yaw) {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.yaw = yaw; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher->publish(msg);
}

void offboard_common::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void offboard_common::disarm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void offboard_common::local_position_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    current_x = msg->position[0];
    current_y = msg->position[1];
    current_z = msg->position[2];
}
}; // namespace nonlin