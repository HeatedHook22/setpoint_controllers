# setpoint_controllers
A program to compare attitude setpoint control and body rate setpoint control of a SIL quadcopter.


# Build/Source
colcon build --symlink-install
source install/setup.bash

# Run Attitude Setpoint Controller Node
ros2 run mrotor_controller att_sp_ctrl