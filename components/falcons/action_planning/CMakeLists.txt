cmake_minimum_required(VERSION 3.8)
project(action_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(interfaces REQUIRED)
find_package(msl_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tracing REQUIRED)

# Add the executable for action_planning_node
add_executable(action_planning_node src/action_planning_node.cpp)
ament_target_dependencies(action_planning_node
  interfaces
  msl_msgs
  rclcpp
  tracing
)
target_link_libraries(action_planning_node
  tracing
)

# Install the executable
install(TARGETS action_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

# hacks ?! wtf ... increase prio on bazel.
target_link_directories(action_planning_node PRIVATE ${tracing_DIR}/../../../lib)

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
