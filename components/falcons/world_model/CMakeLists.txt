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
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Include directories
include_directories(include)

# Create library for WorldModel components
add_library(${PROJECT_NAME}_lib
  src/LocalizationFusion.cpp
  src/WorldModelNode.cpp
  src/WorldModelRosNode.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  falcons_msgs
  mra_falcons_configuration
  mra_common_msgs
  rclcpp
  std_msgs
  mra_tracing
  geometry_msgs
  builtin_interfaces
)
target_link_libraries(${PROJECT_NAME}_lib yaml-cpp) # workaround mra_tracing export dealing

# Add the executable for mra_falcons_world_model_node
add_executable(mra_falcons_world_model_node src/main_node.cpp)
target_link_libraries(mra_falcons_world_model_node ${PROJECT_NAME}_lib)
ament_target_dependencies(mra_falcons_world_model_node
  falcons_msgs
  mra_falcons_configuration
  mra_common_msgs
  rclcpp
  std_msgs
  mra_tracing
)

install(TARGETS mra_falcons_world_model_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Add unit tests
  ament_add_gtest(test_world_model test/test_world_model.cpp)
  target_link_libraries(test_world_model ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_world_model
    falcons_msgs
    mra_falcons_configuration
    mra_common_msgs
    rclcpp
    std_msgs
    mra_tracing
  )

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
