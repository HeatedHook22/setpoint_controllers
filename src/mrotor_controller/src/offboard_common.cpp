#include <chrono>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mrotor_controller/offboard_common.h"

namespace nonlin {
// ------ Public ------ //
offboard_common::offboard_common(std::string node_name) : rclcpp::Node(node_name) {
    // Constructor mostly from offboard_control.cpp example from PX4
    offboard_control_mode_publisher = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    offboard_setpoint_counter = 0;
    auto timer_callback = [this]() -> void {
        if (offboard_setpoint_counter == 10) {
            // Change to Offboard mode after 10 setpoints
            this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

            // Arm the vehicle
            this->arm();
        } // Stop the counter after reaching 11
        else if (offboard_setpoint_counter < 11) {
            offboard_setpoint_counter++;
        }

        // OffboardControlMode needs to be paired with TrajectorySetpoint
        publish_offboard_control_mode();
        publish_trajectory_setpoint();
    };
    timer = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
}

// ------ Private ------ //
void offboard_common::publish_vehicle_command(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher->publish(msg);
}

void offboard_common::publish_trajectory_setpoint() {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {0.0, 0.0, -5.0};
    msg.yaw = -3.14; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher->publish(msg);
}

void offboard_common::publish_offboard_control_mode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher->publish(msg);
}

void offboard_common::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void offboard_common::offboard_enable(bool enable) {}
void offboard_common::offboard_keep_alive() {}
void offboard_common::offboard_log() {}
}; // namespace nonlin