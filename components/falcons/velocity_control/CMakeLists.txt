cmake_minimum_required(VERSION 3.16)
project(mra_falcons_velocity_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create velocity control library
add_library(velocity_control
    src/VelocityControl.cpp
    src/VelocityControlAlgorithms.cpp
    src/VelocityControlConvert.cpp
)

target_include_directories(velocity_control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(velocity_control
    rosidl_runtime_cpp
    rclcpp
    geometry_msgs
)

# Install headers
install(DIRECTORY include/
    DESTINATION include/
)

# Install the library
install(TARGETS velocity_control
    EXPORT velocity_controlTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# Export targets
ament_export_targets(velocity_controlTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_runtime_cpp rclcpp geometry_msgs)
ament_export_include_directories(include)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)

    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)

    # Add gtest
    ament_add_gtest(${PROJECT_NAME}_test
        test/test_velocity_control.cpp
    )

    if(TARGET ${PROJECT_NAME}_test)
        target_link_libraries(${PROJECT_NAME}_test velocity_control)
        target_include_directories(${PROJECT_NAME}_test PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
        )
    endif()

    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
