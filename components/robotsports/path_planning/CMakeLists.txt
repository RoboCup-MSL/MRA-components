# this file was produced by MRA-codegen.py from template_CMakeLists.txt
# with the intent of allowing user to modify it

add_library(MRA-components-robotsports-path-planning
    tick.cpp
    internal/src/facilities/ppgeometry.cpp
    internal/src/algorithms/CheckTargetReached.cpp
    internal/src/algorithms/CheckStopCommand.cpp
    internal/src/algorithms/EscapeForbiddenAreas.cpp
    internal/src/algorithms/CheckTargetValid.cpp
    internal/src/algorithms/RequireWorldModelActive.cpp
    internal/src/algorithms/CalculateObstacles.cpp
    internal/src/algorithms/ForwardDriving.cpp
    internal/src/algorithms/AvoidObstacles.cpp
    internal/src/algorithms/Shielding.cpp
    internal/src/PathPlanningData.cpp
    internal/src/PathPlanning.cpp
)
target_include_directories(MRA-components-robotsports-path-planning PUBLIC
    ${MRA_SOURCE_DIR}/libraries/logging
    ${MRA_SOURCE_DIR}/libraries/geometry
    ${MRA_SOURCE_DIR}
    ${MRA_SOURCE_DIR}/base
    ${CMAKE_CURRENT_SOURCE_DIR}/internal/include
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

