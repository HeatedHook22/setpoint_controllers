#include <cassert>
#include <chrono>
#include <cmath>
#include <functional>

#include <px4_msgs/msg/hover_thrust_estimate.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mrotor_controller/offboard_common.h"

namespace nonlin {
// ------ Public ------ //
offboard_common::offboard_common(std::string node_name) : rclcpp::Node(node_name) {
    // Constructor mostly from offboard_control.cpp example from PX4
    auto qos_pub = rclcpp::QoS(10).best_effort();
    offboard_control_mode_publisher = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_pub);
    trajectory_setpoint_publisher = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_pub);
    vehicle_command_publisher = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_pub);

    // Create a QoS profile that matches PX4 (Best Effort)
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    local_position_subscription = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", qos, std::bind(&offboard_common::get_kinematics_callback, this, std::placeholders::_1));
    hover_thrust_estimate_subscription = this->create_subscription<px4_msgs::msg::HoverThrustEstimate>(
        "/fmu/out/hover_thrust_estimate", qos, std::bind(&offboard_common::get_hover_thrust_estimate_callback, this, std::placeholders::_1));

    auto timer_callback = [this]() -> void {
        if (offboard_setpoint_counter == 100) {
            // Change to Offboard mode after 100 setpoints
            this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

            // Arm the vehicle
            this->arm();
            RCLCPP_INFO(this->get_logger(), "Moving to initial flight position");
        }

        // Stop the counter after reaching 110
        if (offboard_setpoint_counter < 110) {
            offboard_setpoint_counter++;
        }

        if (!mode_switched && !((std::abs(current_pos[0] - initial_pos[0]) < 0.1) && (std::abs(current_pos[1] - initial_pos[1]) < 0.1) &&
                                (std::abs(current_pos[2] - initial_pos[2]) < 0.1))) { // Within 0.1m radius
            // OffboardControlMode needs to be paired with TrajectorySetpoint (default)
            publish_offboard_control_mode(static_cast<uint8_t>(POSITION));
            publish_trajectory_setpoint(initial_pos[0], initial_pos[1], initial_pos[2], yaw_sp);
        } else {
            // Offboard control mode and setpoint control/logic
            if (!mode_switched) {
                RCLCPP_INFO(this->get_logger(), "Switching from trajectory setpoints to custom setpoint control");
                mode_switched = true;

                // Set initial control points
                prev_time = std::chrono::high_resolution_clock::now();
                prev_vel = prev_vel_sp = current_vel;
                prev_pos_sp = current_pos;
            }
            now = std::chrono::high_resolution_clock::now();

            // Compute dt for integral and derivative terms (consider constant per loop iteration)
            dt = std::chrono::duration<float>(now - prev_time).count();

            this->offboard_control_mode_logic();

            // Set new previous timepoint
            prev_time = now;
        }
    };
    timer = this->create_wall_timer(std::chrono::milliseconds(20), timer_callback);
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

Eigen::Vector3f offboard_common::position_control(const Eigen::Vector3f &pos_sp) {
    RCLCPP_INFO(this->get_logger(), "Position Setpoint x: %f", pos_sp[0]);
    RCLCPP_INFO(this->get_logger(), "Position Setpoint y: %f", pos_sp[1]);
    RCLCPP_INFO(this->get_logger(), "Position Setpoint z: %f", pos_sp[2]);

    // Get position difference and scale
    auto pos_error = pos_sp - current_pos;
    auto vel_scaled = kp_pos * pos_error;

    float error_angle = atan2(pos_error[1], pos_error[0]); // North/East angle
    RCLCPP_INFO(this->get_logger(), "Error Angle: %f", error_angle);

    // Set saturations...

    // Feedforward velocity from pos_sp
    // auto vel_ff = (pos_sp - prev_pos_sp) / dt;

    prev_pos_sp = pos_sp;

    RCLCPP_INFO(this->get_logger(), "pos_error x: %f", pos_error[0]);
    RCLCPP_INFO(this->get_logger(), "pos_error y: %f", pos_error[1]);
    RCLCPP_INFO(this->get_logger(), "pos_error z: %f", pos_error[2]);
    return /*vel_ff +*/ vel_scaled;
}

Eigen::Vector3f offboard_common::velocity_control(const Eigen::Vector3f &vel_sp) {
    RCLCPP_INFO(this->get_logger(), "Velocity Setpoint x: %f", vel_sp[0]);
    RCLCPP_INFO(this->get_logger(), "Velocity Setpoint y: %f", vel_sp[1]);
    RCLCPP_INFO(this->get_logger(), "Velocity Setpoint z: %f", vel_sp[2]);

    // Get velocity difference and scale
    auto vel_error = vel_sp - current_vel;
    auto acc_scaled = kp_vel * vel_error;

    // Integrate error term and scale
    integral_acc_error += ki_vel * vel_error * dt;

    float i_limit = 2.0f;
    integral_acc_error[0] = std::clamp(integral_acc_error[0], -i_limit, i_limit);
    integral_acc_error[1] = std::clamp(integral_acc_error[1], -i_limit, i_limit);
    integral_acc_error[2] = std::clamp(integral_acc_error[2], -i_limit, i_limit);

    // Derivative term and scale (no LPF currently)
    auto current_acc_der = kd_vel * (current_vel - prev_vel) / dt;

    // Feedforward acceleration from vel_sp
    // auto acc_ff = (vel_sp - prev_vel_sp) / dt;

    prev_vel = current_vel;
    prev_vel_sp = vel_sp;

    RCLCPP_INFO(this->get_logger(), "vel_error x: %f", vel_error[0]);
    RCLCPP_INFO(this->get_logger(), "vel_error y: %f", vel_error[1]);
    RCLCPP_INFO(this->get_logger(), "vel_error z: %f", vel_error[2]);

    Eigen::Vector3f vel_out = /*acc_ff +*/ acc_scaled + integral_acc_error - current_acc_der;

    return vel_out;
}

std::pair<Eigen::Quaternionf, Eigen::Vector3f> offboard_common::acceleration_control(const Eigen::Vector3f &acc_sp) {
    RCLCPP_INFO(this->get_logger(), "Acceleration Setpoint Z: %f", acc_sp[2]);

    auto acc_diff = acc_sp - gravity_vector;
    auto norm_acc_diff = (acc_diff.norm() / gravity_vector.norm()) * hover_thrust_estimate;
    auto final_thrust = std::max(0.5f, std::min(0.8f, norm_acc_diff));

    Eigen::Quaternionf q_d;
    if (acc_diff.norm() < 0.001f) {
        q_d = Eigen::Quaternionf::Identity();
    } else {
        Eigen::Vector3f zb = -acc_diff.normalized();

        // Use the actual direction of the acceleration setpoint as the basis
        // This ensures the drone's "Forward" tilts exactly where acc_sp points
        Eigen::Vector3f x_text(acc_sp[0], acc_sp[1], 0.0f);
        if (x_text.norm() < 0.001f) {
            x_text = Eigen::Vector3f(sin(yaw_sp), cos(yaw_sp), 0.0f);
        }

        Eigen::Vector3f yb = zb.cross(x_text).normalized();
        Eigen::Vector3f xb = yb.cross(zb);

        Eigen::Matrix3f R;
        R.col(0) = xb;
        R.col(1) = yb;
        R.col(2) = zb;

        q_d = Eigen::Quaternionf(R);

        RCLCPP_INFO(this->get_logger(), "Tilt Angle: %f", atan2(xb[1], xb[0]));
    }
    // } else {
    //     Eigen::Vector3f zb = -acc_diff.normalized();
    //     Eigen::Vector3f x_yaw(sin(yaw_sp), cos(yaw_sp), 0.0f);

    //     Eigen::Vector3f yb = zb.cross(x_yaw).normalized();

    //     if (yb.norm() < 0.001f) {
    //         yb = Eigen::Vector3f::UnitX();
    //     }

    //     Eigen::Vector3f xb = yb.cross(zb);

    //     float tilt_angle = atan2(xb[1], xb[0]);
    //     RCLCPP_INFO(this->get_logger(), "Tilt Angle: %f", tilt_angle);

    //     Eigen::Matrix3f R;
    //     R.col(0) = xb;
    //     R.col(1) = yb;
    //     R.col(2) = zb;

    //     q_d = Eigen::Quaternionf(R);
    // }

    Eigen::Vector3f final_thrust_vector(0.f, 0.f, -final_thrust);
    RCLCPP_INFO(this->get_logger(), "final_thrust Z: %f", -final_thrust);

    return {q_d.normalized(), final_thrust_vector};
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
    msg.position = {y, x, z}; // Flipped in PX4
    msg.yaw = yaw;            // [-PI:PI]
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

void offboard_common::get_kinematics_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    // x-y Flipped in PX4
    current_pos[0] = msg->y;
    current_pos[1] = msg->x;
    current_pos[2] = msg->z;

    current_vel[0] = msg->vy;
    current_vel[1] = msg->vx;
    current_vel[2] = msg->vz;

    current_acc[0] = msg->ay;
    current_acc[1] = msg->ax;
    current_acc[2] = msg->az;
}

void offboard_common::get_hover_thrust_estimate_callback(const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg) {
    hover_thrust_estimate = msg->hover_thrust;
}

}; // namespace nonlin