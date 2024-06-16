# ros-gz-crazyflie

Made for ROS2 humble and Gazebo Fortress

## Usage

Source the crazyflie model

    export GZ_SIM_RESOURCE_PATH="path_to/ros_gz_crazyflie/ros_gz_crazyflie_description/models/

Build with

    colcon build --cmake-args -DBUILD_TESTING=ON

Run with

    ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py

Velocity control with

    ros2 topic pub --once /crazyflie/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.1}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

Takeoff with

    ros2 service call /land std_srvs/srv/Trigger


Control telelop with

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=crazyflie/cmd_vel

The sdf model Crazyflie is the adapted version of X3 UAV (found here on fuel: https://app.gazebosim.org/OpenRobotics/fuel/models/X3%20UAV). The visuals have been replaced by meshes made for the Crazyflie, to be found here: https://github.com/bitcraze/crazyflie-simulation.