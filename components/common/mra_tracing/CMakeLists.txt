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

# Create library
add_library(${PROJECT_NAME}
    src/tracing.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
    rosidl_runtime_cpp
    rclcpp
    yaml-cpp
)

# Install headers
install(DIRECTORY include/
    DESTINATION include/
)

# Install library
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# Export targets
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_runtime_cpp rclcpp yaml-cpp)
ament_export_include_directories(include)

ament_package()
