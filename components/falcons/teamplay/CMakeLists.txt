cmake_minimum_required(VERSION 3.16)
project(mra_falcons_teamplay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(mra_falcons_configuration REQUIRED)
find_package(mra_common_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mra_tracing REQUIRED)

# Create library for Teamplay components
add_library(${PROJECT_NAME}_lib
    src/TeamplayNode.cpp
    src/TeamplayRosNode.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
    mra_falcons_configuration
    mra_common_msgs
    rclcpp
    std_msgs
    mra_tracing
)

# Add the executable for the ROS node
add_executable(${PROJECT_NAME}_node
    src/main_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}_lib
)

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
)

# Install headers
install(DIRECTORY include/
    DESTINATION include/
)

# Install library
install(TARGETS ${PROJECT_NAME}_lib
    EXPORT ${PROJECT_NAME}_libTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install executable
install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

# Export targets
ament_export_targets(${PROJECT_NAME}_libTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(mra_falcons_configuration mra_common_msgs rclcpp std_msgs mra_tracing)
ament_export_include_directories(include)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
