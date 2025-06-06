# this file was produced by MRA-codegen.py from template_CMakeLists.txt
# with the intent of allowing user to modify it

add_library(MRA-components-robotsports-path-planning
    tick.cpp
    internal/setpoint/balance_xy.cpp
    internal/setpoint/get_segments.cpp
    internal/setpoint/traj1.cpp
    internal/setpoint/traj_pred.cpp
)


# add_library(MRA-components-robotsports-velocity-control
#     tick.cpp
#     internal/src/VelocityControl.cpp
#     internal/src/controllers/LinearVelocitySetpointController.cpp
#     internal/src/controllers/SPGVelocitySetpointController.cpp
#     internal/src/algorithms/CheckPrepareInputs.cpp
#     internal/src/algorithms/CheckStop.cpp
#     internal/src/algorithms/ConfigureLimits.cpp
#     internal/src/algorithms/Deadzone.cpp
#     internal/src/algorithms/SelectVelocityController.cpp
#     internal/src/algorithms/ShiftBallOffset.cpp
#     internal/src/algorithms/CalculateVelocity.cpp
#     internal/src/algorithms/SetOutputsPrepareNext.cpp
# )

target_include_directories(MRA-components-robotsports-path-planning PUBLIC
    ${MRA_SOURCE_DIR}/libraries/logging
    ${MRA_SOURCE_DIR}
    ${MRA_SOURCE_DIR}/base
    ${CMAKE_CURRENT_SOURCE_DIR}/internal/setpoint/include
    ${CMAKE_CURRENT_BINARY_DIR}/interface)
target_link_libraries(MRA-components-robotsports-path-planning MRA-libraries MRA-base nlohmann_json::nlohmann_json)


add_executable(
    robotsports_path_planning_test
    test.cpp
)
target_include_directories(robotsports_path_planning_test PRIVATE ${MRA_SOURCE_DIR}/base ${MRA_SOURCE_DIR}/libraries/logging ${MRA_BINARY_DIR})
target_link_libraries(
    robotsports_path_planning_test
    MRA-components-robotsports-path-planning
    MRA-libraries MRA-components nlohmann_json::nlohmann_json rt GTest::gtest_main gmock
)


gtest_discover_tests(
    robotsports_path_planning_test
    WORKING_DIRECTORY ${MRA_SOURCE_DIR}
    PROPERTIES ENVIRONMENT "MRA_LOGGER_CONTEXT=unittest"
)

