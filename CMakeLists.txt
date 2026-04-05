cmake_minimum_required(VERSION 3.16)
project(from_signal_to_system)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# https://cmake.org/cmake/help/latest/variable/PROJECT_VERSION.html
set(PROJECT_VERSION "0.1.0")

# Find required packages
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Uncomment for additional packages
# find_package(PCL REQUIRED COMPONENTS common io)
# find_package(Eigen3 REQUIRED)
# find_package(OpenCV REQUIRED)

# Include directories
include_directories(
    include
)

# ===== Control Module (C++) =====
add_executable(motion_controller
    src/control/motion_controller.cpp
)
ament_target_dependencies(motion_controller
    rclcpp
    geometry_msgs
    nav_msgs
    std_msgs
)

add_executable(low_level_interface
    src/control/low_level_interface.cpp
)
ament_target_dependencies(low_level_interface
    rclcpp
    std_msgs
    sensor_msgs
)

# ===== Installation Targets =====
install(TARGETS
    motion_controller
    low_level_interface
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
)

# Install RViz configs
install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
)

# Install configuration files
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

# ===== Testing =====
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    
    # TODO: Add unit tests for control modules
    # ament_add_gtest(test_motion_controller test/test_motion_controller.cpp)
    # ament_target_dependencies(test_motion_controller motion_controller)
endif()

# Create ament package
ament_package()
print("CMakeLists.txt configured. Build with:")
print("  colcon build --symlink-install")
