cmake_minimum_required(VERSION 3.5)
project(mra_falcons_action_planning)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mra_falcons_action_fetch_ball REQUIRED)
find_package(mra_falcons_action_base REQUIRED)

# Include directories
include_directories(include)

# Create the executable
add_executable(mra_falcons_action_planning_node src/action_planning_node.cpp)

# Link libraries
ament_target_dependencies(mra_falcons_action_planning_node
  rclcpp
  mra_falcons_action_fetch_ball
  mra_falcons_action_base
)

# Install the executable
install(TARGETS
  mra_falcons_action_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()