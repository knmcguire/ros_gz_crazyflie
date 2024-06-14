cmake_minimum_required(VERSION 3.5)

project(ros_gz_crazyflie_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ros_gz_crazyflie_description REQUIRED)

find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-common4 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${ignition-common4_VERSION_MAJOR})
find_package(ignition-gazebo6 REQUIRED)
set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})


install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()