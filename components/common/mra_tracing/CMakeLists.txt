cmake_minimum_required(VERSION 3.16)
project(mra_tracing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Use target_include_directories for modern CMake
add_library(mra_tracing
  src/tracing.cpp
)
target_include_directories(mra_tracing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install the library
install(TARGETS mra_tracing
  DESTINATION lib
)

ament_export_include_directories(include)
ament_export_libraries(mra_tracing)

ament_package()
