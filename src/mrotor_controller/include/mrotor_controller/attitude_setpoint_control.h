#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mrotor_controller/offboard_common.h"

namespace nonlin {
class attitude_setpoint_control : public offboard_common {
  public:
    // Constructors
    attitude_setpoint_control(std::string node_name);
    ~attitude_setpoint_control() = default;

  private:
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_publisher{};

    /// @brief This is the main setpoint control
    void offboard_control_mode_logic() override;

    /// @brief Publishes attitude setpoint information
    void publish_attitude_setpoint();
};
}; // namespace nonlin
