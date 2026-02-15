cmake_minimum_required(VERSION 3.8)
project(mrotor_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  # Fallback for some systems
  set(EIGEN3_INCLUDE_DIRS "/usr/include/eigen3")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(att_sp_ctrl
  src/attitude_setpoint_control.cpp
  src/offboard_common.cpp
)

target_include_directories(att_sp_ctrl PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(att_sp_ctrl
  rclcpp
  px4_msgs
  geometry_msgs
  Eigen3
)

install(TARGETS
  att_sp_ctrl
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
