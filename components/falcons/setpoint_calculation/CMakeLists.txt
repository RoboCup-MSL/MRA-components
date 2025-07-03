cmake_minimum_required(VERSION 3.16)
project(mra_falcons_setpoint_calculation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mra_falcons_configuration REQUIRED)
find_package(mra_common_msgs REQUIRED)
find_package(mra_falcons_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mra_tracing REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(mra_falcons_velocity_control REQUIRED)

# Create the setpoint calculation library
add_library(setpoint_calculation
  src/SetpointCalculation.cpp
)
ament_target_dependencies(setpoint_calculation
  mra_common_msgs
  mra_falcons_msgs
  mra_tracing
  mra_falcons_velocity_control
)
target_link_libraries(setpoint_calculation yaml-cpp)
target_include_directories(setpoint_calculation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add the executable for the ROS node
add_executable(${PROJECT_NAME}_node src/node.cpp)
target_link_libraries(${PROJECT_NAME}_node setpoint_calculation)
ament_target_dependencies(${PROJECT_NAME}_node
  mra_falcons_configuration
  mra_common_msgs
  mra_falcons_msgs
  rclcpp
  std_msgs
  mra_tracing
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install the library
install(TARGETS setpoint_calculation
  DESTINATION lib
)

# Install the executable
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export the library
ament_export_include_directories(include)
ament_export_libraries(setpoint_calculation)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
