cmake_minimum_required(VERSION 3.10)
project(pegasus_sim_env)

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
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)

# Include directories for your package
include_directories(include)

# Add executable for src/setpoints.cpp
add_executable(setpoints_node src/setpoints.cpp)
ament_target_dependencies(setpoints_node
  rclcpp 
  px4_msgs
)

install(TARGETS 
  setpoints_node 
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

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/examples/test_flight.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install python launch files.
# install(FILES
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
