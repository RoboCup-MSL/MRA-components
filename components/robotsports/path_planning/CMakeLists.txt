# this file was produced by MRA-codegen.py from template_CMakeLists.txt
# with the intent of allowing user to modify it

add_library(MRA-components-robotsports-path-planning
    tick.cpp
    internal/setpoint/balance_xy.cpp
    internal/setpoint/get_segments.cpp
    internal/setpoint/setpoint.cpp
    internal/setpoint/traj_pred.cpp
    internal/setpoint/traj1.cpp
    internal/setpoint/traject_common.cpp
    internal/setpoint/wrap.cpp
    #internal/subtarget/subtarget.cpp  # new
    #internal/subtarget/replan/new_subtarget.cpp # new
    internal/subtarget/replan/determine_setpoint_limits.cpp
    #internal/subtarget/replan/get_distance_inside_penalty_area.cpp #new
    internal/subtarget/replan/search/beside_obstacle.cpp
    internal/subtarget/replan/search/inpolygonpy
    #internal/subtarget/replan/search/random.cpp # new
    internal/subtarget/angle/aim_at_target.cpp
    internal/subtarget/angle/angle.cpp
    internal/subtarget/angle/dribble_module.cpp
    internal/subtarget/angle/shield.cpp
    # internal/subtarget/angle/test_angle.cpp
    internal/target/adjust_to_3m_rule.cpp
    internal/target/adjust_to_field.cpp
    internal/target/adjust_to_goal_area.cpp
    internal/target/adjust_to_obstacles.cpp
    internal/target/adjust_to_penalty_area.cpp
    internal/target/target.cpp
)



target_include_directories(MRA-components-robotsports-path-planning PUBLIC
    ${MRA_SOURCE_DIR}/libraries/logging
    ${MRA_SOURCE_DIR}
    ${MRA_SOURCE_DIR}/base
    ${CMAKE_CURRENT_SOURCE_DIR}/internal/setpoint
    ${CMAKE_CURRENT_SOURCE_DIR}/internal/target
    ${CMAKE_CURRENT_SOURCE_DIR}/internal/subtarget
    ${CMAKE_CURRENT_BINARY_DIR}/interface)
target_link_libraries(MRA-components-robotsports-path-planning MRA-libraries MRA-base nlohmann_json::nlohmann_json)


add_executable(
    robotsports_path_planning_test
    test.cpp
)
target_include_directories(robotsports_path_planning_test 
    PRIVATE ${MRA_SOURCE_DIR}/base 
    ${MRA_SOURCE_DIR}/libraries/logging 
    ${MRA_BINARY_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    )
target_link_libraries(
    robotsports_path_planning_test
    MRA-components-robotsports-path-planning
    MRA-libraries MRA-components nlohmann_json::nlohmann_json rt GTest::gtest_main gmock ${Eigen_LIBRARIES})


gtest_discover_tests(
    robotsports_path_planning_test
    WORKING_DIRECTORY ${MRA_SOURCE_DIR}
    PROPERTIES ENVIRONMENT "MRA_LOGGER_CONTEXT=unittest"
)

