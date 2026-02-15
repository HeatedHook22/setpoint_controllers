#include <iostream>
#include <memory>

#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>

#include "mrotor_controller/attitude_setpoint_control.h"

namespace nonlin {
// ------ Public ------ //
attitude_setpoint_control::attitude_setpoint_control(std::string node_name) : offboard_common(node_name) {
    auto qos_pub = rclcpp::QoS(10).best_effort();
    attitude_setpoint_publisher = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint_v1", qos_pub);
}

// ------ Private ------ //
void attitude_setpoint_control::offboard_control_mode_logic() { this->publish_attitude_setpoint(); }

void attitude_setpoint_control::publish_attitude_setpoint() {
    this->publish_offboard_control_mode(offboard_common::ATTITUDE);

    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    // Set the orientation [w, x, y, z]
    msg.q_d[0] = 1.f;
    msg.q_d[1] = 0.0f;
    msg.q_d[2] = 0.0f;
    msg.q_d[3] = 0.0f;

    // Uses Front-Right-Down format
    msg.thrust_body[0] = 0.f;
    msg.thrust_body[1] = 0.f;
    msg.thrust_body[2] = -0.75f;
    msg.yaw_sp_move_rate = 0.0f;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    attitude_setpoint_publisher->publish(msg);
}
}; // namespace nonlin

// ------ Main ------ //
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto att_sp_node = std::make_shared<nonlin::attitude_setpoint_control>("att_sp_node");

    // NOTE: The line below is the Single-Threaded Executor, so type-safety is implied and we do not use any thread protection in this program
    rclcpp::spin(att_sp_node);
    rclcpp::shutdown();
    return 0;
}