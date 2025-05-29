cmake_minimum_required(VERSION 3.8)
project(tracing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Create a library for tracing
add_library(tracing
  src/tracing.cpp
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install the library
install(TARGETS tracing
  DESTINATION lib
)

ament_export_include_directories(include)
#ament_export_libraries(tracing)

ament_package()
