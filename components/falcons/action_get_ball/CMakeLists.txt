cmake_minimum_required(VERSION 3.5)
project(mra_falcons_action_get_ball)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(mra_falcons_action_base REQUIRED)
find_package(mra_common_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Input.msg"
  "msg/Config.msg"
)

# Create the library (avoid name clash with msg target)
add_library(action_get_ball_lib SHARED
  src/action_get_ball.cpp
)
target_include_directories(action_get_ball_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(action_get_ball_lib
  mra_falcons_action_base
  mra_common_msgs
)

# Ensure the library depends on the generated messages
add_dependencies(action_get_ball_lib mra_falcons_action_get_ball__rosidl_typesupport_cpp)

# Install the library
install(TARGETS action_get_ball_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(mra_common_msgs)
ament_export_libraries(action_get_ball_lib)
ament_package()
