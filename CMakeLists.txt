cmake_minimum_required(VERSION 3.10)
project(pegasus_sim_env)

# Set the required CMake version
cmake_minimum_required(VERSION 3.10)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Suppress CMP0144 warnings (find_package uses upper-case <PACKAGENAME>_ROOT variables)
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

# Suppress CMP0167 warnings (FindBoost module is removed)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
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
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Include directories for your package
include_directories(include)

# Add executable for src/setpoints.cpp
add_executable(setpoints_node src/setpoints.cpp)
ament_target_dependencies(setpoints_node
  rclcpp 
  px4_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Add executable for src/cost_map_node.cpp
add_executable(cost_map_node src/cost_map_node.cpp)
ament_target_dependencies(cost_map_node
  rclcpp
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  pcl_conversions
  pcl_ros
)

# Add executable for src/planner_node.cpp
add_executable(planner_node src/planner_node.cpp)
ament_target_dependencies(planner_node
  rclcpp
  nav_msgs
  px4_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs 
)

add_executable(vel_controller src/vel_controller.cpp)
ament_target_dependencies(vel_controller
  rclcpp 
  px4_msgs
  nav_msgs
  tf2
  tf2_ros
  Eigen3
)

install(TARGETS 
  setpoints_node
  cost_map_node
  planner_node
  vel_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Build tests if required
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

###########
# Python ##
###########

# Install the launch directory
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/examples/test_flight.py
  src/examples/path_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install python launch files.
# install(FILES
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
