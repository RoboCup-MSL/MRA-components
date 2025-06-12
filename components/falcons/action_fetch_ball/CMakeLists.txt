cmake_minimum_required(VERSION 3.5)
project(mra_falcons_action_fetch_ball)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(mra_falcons_action_base REQUIRED)

# Create the library
add_library(${PROJECT_NAME} src/action_fetch_ball.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  mra_falcons_action_base
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
