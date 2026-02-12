#include <iostream>
#include <memory>

#include "mrotor_controller/offboard_common.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto common_test = std::make_shared<nonlin::offboard_common>("common_test_node");

    rclcpp::spin(common_test);
    rclcpp::shutdown();
    std::cout << "Hello World!!" << std::endl;
    return 0;
}