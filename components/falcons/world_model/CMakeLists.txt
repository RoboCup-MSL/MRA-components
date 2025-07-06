cmake_minimum_required(VERSION 3.16)
project(mra_falcons_world_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(falcons_msgs REQUIRED)
find_package(mra_falcons_configuration REQUIRED)
find_package(mra_common_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mra_tracing REQUIRED)

# Add the executable for world_model_node
add_executable(world_model_node src/world_model_node.cpp)
ament_target_dependencies(world_model_node
  falcons_msgs
  mra_falcons_configuration
  mra_common_msgs
  rclcpp
  std_msgs
  mra_tracing
)

install(TARGETS world_model_node
  DESTINATION lib/${PROJECT_NAME}
)

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
