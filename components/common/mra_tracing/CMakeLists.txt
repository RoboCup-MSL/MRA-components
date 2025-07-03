cmake_minimum_required(VERSION 3.16)
project(mra_tracing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Note: mra_tracing exposes yaml-cpp in its headers, so client packages
# that use TRACE_FUNCTION_INPUTS with ROS messages must also:
# 1. find_package(yaml-cpp REQUIRED)
# 2. target_link_libraries(<target> yaml-cpp)

# Use target_include_directories for modern CMake
add_library(mra_tracing
  src/tracing.cpp
)
ament_target_dependencies(mra_tracing
  rosidl_runtime_cpp
  rclcpp
  yaml-cpp
)

target_include_directories(mra_tracing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install the library
install(TARGETS mra_tracing
  EXPORT mra_tracing_targets
  DESTINATION lib
)

# Export targets
install(EXPORT mra_tracing_targets
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_libraries(mra_tracing)
ament_export_targets(mra_tracing_targets)
ament_export_dependencies(rosidl_runtime_cpp rclcpp yaml-cpp)

ament_package()
