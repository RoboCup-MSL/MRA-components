cmake_minimum_required(VERSION 3.8)
project(mra_falcons_action_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mra_msgs REQUIRED)
find_package(mra_tracing REQUIRED)

# Add the executable for action_planning_node
add_executable(mra_falcons_action_planning_node action_planning_node.cpp)
ament_target_dependencies(mra_falcons_action_planning_node
  rclcpp
  mra_msgs
  mra_tracing
)
target_link_libraries(mra_falcons_action_planning_node
  mra_tracing
)

# Install the executable
install(TARGETS mra_falcons_action_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

# hacks ?! wtf ... increase prio on bazel.
target_link_directories(mra_falcons_action_planning_node PRIVATE ${mra_tracing_DIR}/../../../lib)

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
