# this file was produced by MRA-codegen.py from template_CMakeLists.txt
# it should NOT be modified by user

add_library(MRA-components-robotsports-role-assigner
    tick.cpp
    internals/DynamicRoleAssignment.cpp
    internals/Dynamics.cpp
    internals/Edge.cpp
    internals/GlobalPathPlanner.cpp
    internals/GlobalPathDynamicPlanner.cpp
    internals/GridHeuristic.cpp
    internals/planner_common.cpp
    internals/PlannerGridInfoData.cpp
    internals/PlannerOptions.cpp
    internals/RolePosition.cpp
    internals/SvgUtils.cpp
    internals/TeamPlannerData.cpp
    internals/TeamPlannerExport.cpp
    internals/TeamPlay.cpp
    internals/Vertex.cpp
    internals/TeamPlannerGrid.cpp
    temp/FieldConfig.cpp
    temp/MathUtils.cpp
    temp/MovingObject.cpp
    temp/Position.cpp
)

target_include_directories(MRA-components-robotsports-role-assigner PUBLIC
    ${MRA_SOURCE_DIR}/libraries/logging
    ${MRA_SOURCE_DIR}
    ${MRA_SOURCE_DIR}/base
    ${CMAKE_CURRENT_BINARY_DIR}/interface
    ${CMAKE_CURRENT_SOURCE_DIR}/internals
    ${CMAKE_CURRENT_SOURCE_DIR}/temp
 )
target_link_libraries(MRA-components-robotsports-role-assigner MRA-libraries MRA-components-proto MRA-base nlohmann_json::nlohmann_json)


add_executable(
    robotsports_role_assigner_test
    test.cpp
)
target_include_directories(robotsports_role_assigner_test PRIVATE ${MRA_SOURCE_DIR}/base ${MRA_SOURCE_DIR}/libraries/logging)
target_link_libraries(
    robotsports_role_assigner_test
    MRA-components-robotsports-role-assigner
    MRA-libraries MRA-components-proto nlohmann_json::nlohmann_json rt GTest::gtest_main gmock
)


gtest_discover_tests(
    robotsports_role_assigner_test
    WORKING_DIRECTORY ${MRA_SOURCE_DIR}
    PROPERTIES ENVIRONMENT "MRA_LOGGER_CONTEXT=unittest"
)

