cmake_minimum_required(VERSION 3.16)
project(mra_tracing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)

# Use target_include_directories for modern CMake
add_library(mra_tracing
  src/tracing.cpp
)
ament_target_dependencies(mra_tracing
  rosidl_runtime_cpp
  rclcpp
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
  DESTINATION lib
)

ament_export_include_directories(include)
ament_export_libraries(mra_tracing)

ament_package()
