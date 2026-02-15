# setpoint_controllers
A program to compare attitude setpoint control and body rate setpoint control of a SIL quadcopter.


# Build/Source
colcon build --symlink-install

# Run Attitude Setpoint Controller Node
source install/setup.bash
ros2 run mrotor_controller att_sp_ctrl

# Helper debug commands
ros2 topic echo /fmu/in/offboard_control_mode
ros2 topic echo /fmu/out/vehicle_status_v1
ros2 topiecho /fmu/in/vehicle_attitude_setpoint
ros2 topic echo /fmu/out/vehicle_odometry