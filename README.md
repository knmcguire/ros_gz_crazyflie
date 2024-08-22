# ros-gz-crazyflie

> Still work in progress!

> The modules for the multiranger, like the simple mapper and wall following are moved to: https://github.com/knmcguire/crazyflie_ros2_multiranger

Made for ROS2 humble and Gazebo Harmonic on Ubuntu 22.04

## Usage

Make a workspace and clone this gazebo-ros crazyflie simulation repo

    mkdir ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/knmcguire/ros_gz_crazyflie


Clone the crazyflie simulation repo and source the crazyflie model

    mkdir ~/simulation_models/
    cd ~/simulation_models/
    git clone git@github.com:bitcraze/crazyflie-simulation.git
    export GZ_SIM_RESOURCE_PATH=~/simulation_models/simulator_files/gazebo/"

Build the ros workspace with

    cd  ~/ros2_ws/
    colcon build --cmake-args -DBUILD_TESTING=ON

Run with

    ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py

Velocity control ROS topics with

    ros2 topic pub --once /crazyflie/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.1}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

Takeoff with

    ros2 service call /land std_srvs/srv/Trigger

Control telelop with

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=crazyflie/cmd_vel


