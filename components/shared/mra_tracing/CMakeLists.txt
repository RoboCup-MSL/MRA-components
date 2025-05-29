cmake_minimum_required(VERSION 3.8)
project(mra_tracing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Create a library for tracing
add_library(mra_tracing
  src/tracing.cpp
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install the library
install(TARGETS mra_tracing
  DESTINATION lib
)

ament_export_include_directories(include)
#ament_export_libraries(mra_tracing)

ament_package()
