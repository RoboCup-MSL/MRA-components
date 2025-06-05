cmake_minimum_required(VERSION 3.8)
project(mra_falcons_action_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mra_common_msgs REQUIRED)
find_package(mra_tracing REQUIRED)
find_package(mra_falcons_msgs REQUIRED)

# Legacy MRA library
add_library(MRA-components-falcons-action-planning
    tick.cpp
)
target_include_directories(MRA-components-falcons-action-planning PUBLIC
    ${MRA_SOURCE_DIR}/libraries/logging
    ${MRA_SOURCE_DIR}
    ${MRA_SOURCE_DIR}/base
    ${MRA_SOURCE_DIR}/components/falcons/action_planning
    ${MRA_SOURCE_DIR}/components/falcons/action_planning/internal
    ${MRA_SOURCE_DIR}/components/falcons/action_get_ball
    ${MRA_SOURCE_DIR}/components/falcons/action_move
    ${MRA_SOURCE_DIR}/components/falcons/action_stop
    ${MRA_SOURCE_DIR}/components/falcons/action_aimed_kick
    ${MRA_SOURCE_DIR}/components/falcons/action_park
    ${MRA_SOURCE_DIR}/components/falcons/action_catch_ball
    ${MRA_SOURCE_DIR}/components/falcons/action_keeper
    ${CMAKE_CURRENT_BINARY_DIR}/interface
)
target_link_libraries(MRA-components-falcons-action-planning
    MRA-libraries MRA-base
)

# TODO
# Legacy test executable
#add_executable(
#    falcons_action_planning_test
#    test.cpp
#)
#target_include_directories(falcons_action_planning_test
#    PRIVATE
#        ${MRA_SOURCE_DIR}/base
#        ${MRA_SOURCE_DIR}/libraries/logging
#        ${MRA_BINARY_DIR}
#)
#target_link_libraries(
#    falcons_action_planning_test
#    MRA-components-falcons-action-planning
#    MRA-components-falcons-action-get-ball
#    MRA-components-falcons-action-move
#    MRA-components-falcons-action-stop
#    MRA-components-falcons-action-aimed-kick
#    MRA-components-falcons-action-park
#    MRA-components-falcons-action-catch-ball
#    MRA-components-falcons-action-keeper
#    MRA-libraries MRA-components rt GTest::gtest_main gmock
#)

# TODO
#gtest_discover_tests(
#    falcons_action_planning_test
#    WORKING_DIRECTORY ${MRA_SOURCE_DIR}
#    PROPERTIES ENVIRONMENT "MRA_LOGGER_CONTEXT=unittest"
#)

# ROS2 node wrapping the legacy library
add_executable(mra_falcons_action_planning_node action_planning_node.cpp)
ament_target_dependencies(mra_falcons_action_planning_node
  rclcpp
  mra_common_msgs
  mra_tracing
  mra_falcons_msgs
)
target_link_libraries(mra_falcons_action_planning_node
  mra_tracing
  MRA-components-falcons-action-planning
)

# Install the ROS2 node
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
