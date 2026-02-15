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

    Eigen::Vector3f pos_sp(5.f, 5.f, -5.f); // NED
    auto v_sp = position_control(pos_sp);
    auto a_sp = velocity_control(v_sp);
    auto [q_d, thrust] = acceleration_control(a_sp);

    // Eigen::Vector3f a_sp(0.0f, 0.0f, 0.0f); // Hardcode: Hover only
    // auto [q_d, thrust] = acceleration_control(a_sp);

    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    // Set the orientation [w, x, y, z]
    msg.q_d[0] = q_d.w();
    msg.q_d[1] = q_d.x(); // Flip (x-y) for PX4?
    msg.q_d[2] = q_d.y();
    msg.q_d[3] = q_d.z();

    // Uses Front-Right-Down format
    msg.thrust_body[0] = thrust[0];
    msg.thrust_body[1] = thrust[1];
    msg.thrust_body[2] = thrust[2];
    msg.yaw_sp_move_rate = yaw_sp;
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