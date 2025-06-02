# ros-gz-crazyflie

> Experimental

> The modules for the multiranger, like the simple mapper and wall following are moved to: https://github.com/knmcguire/crazyflie_ros2_multiranger

Tested on:
* ROS 2 Humble with Gazebo Harmonic (2024/09/02)
* ROS 2 Jazzy () with Gazebo Harmonic (2025/06/02)
* ROS 2 Ionic with Gazebo Ionic (2025/06/02)

## Usage

Make a workspace and clone this gazebo-ros crazyflie simulation repo

    mkdir ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/knmcguire/ros_gz_crazyflie


Clone the crazyflie simulation repo and source the crazyflie model

    mkdir ~/simulation_models/
    cd ~/simulation_models/
    git clone git@github.com:bitcraze/crazyflie-simulation.git
    export GZ_SIM_RESOURCE_PATH=~/simulation_models/crazyflie-simulation/simulator_files/gazebo/"

Build the ros workspace with

    cd  ~/ros2_ws/
    colcon build --cmake-args -DBUILD_TESTING=ON

Run with

    source install/local_setup.bash
    ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py


In a different terminal, open ROS 2 teleop twist keyboard node with

    source /opt/ros/DISTRO/setup.bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Take off with pressing 't' and then control the crazyflie with the keyboard according to the teleop_twist_keyboard instructions.


