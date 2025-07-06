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

# Add the executable for the ROS node
add_executable(${PROJECT_NAME}_node
    src/node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
    mra_falcons_configuration
    mra_common_msgs
    rclcpp
    std_msgs
    mra_tracing
)

# Install the executable
install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
