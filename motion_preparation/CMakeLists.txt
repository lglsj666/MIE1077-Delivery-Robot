if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)
endif()
if(DEFINED CATKIN_SYMLINK_INSTALL)
  unset(CATKIN_SYMLINK_INSTALL)
endif()

cmake_minimum_required(VERSION 3.5)
project(motion_preparation)

# Default to C++14 (not strictly needed for pure Python)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_servo REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Mark this package as ament package
ament_package()
