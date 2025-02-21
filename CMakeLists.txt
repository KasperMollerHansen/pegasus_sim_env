cmake_minimum_required(VERSION 3.10)
project(sim_px4_ros2)

# Set the required CMake version
cmake_minimum_required(VERSION 3.10)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)


# Include directories for your package
include_directories(include)

# Add executable for your project
add_executable(sim_px4_ros2_node src/setpoints.cpp)
install(TARGETS sim_px4_ros2_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(sim_px4_ros2_node rclcpp px4_msgs)

# Build tests if required
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
