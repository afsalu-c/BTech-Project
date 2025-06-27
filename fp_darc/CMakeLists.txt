cmake_minimum_required(VERSION 3.8)
project(fp_darc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


find_package(robot_state_publisher REQUIRED)
find_package(urdf REQUIRED)

#dded below lines for the teleoperation in rviz
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
#find_package(xacro REQUIRED)
#find_package(launch REQUIRED)
#find_package(launch_ros REQUIRED)
#here is the end

install(DIRECTORY
  launch
  urdf
  meshes
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()