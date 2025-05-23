# this file was produced by MRA-codegen.py from template_CMakeLists.txt
# with the intent of allowing user to modify it

add_library(MRA-components-falcons-trajectory-generation
    tick.cpp
)
target_include_directories(MRA-components-falcons-trajectory-generation PUBLIC
    ${MRA_SOURCE_DIR}/libraries/logging
    ${MRA_SOURCE_DIR}/libraries/geometry
    ${MRA_SOURCE_DIR}
    ${MRA_SOURCE_DIR}/base
    ${MRA_SOURCE_DIR}/components/falcons/velocity_control
    ${CMAKE_CURRENT_BINARY_DIR}/interface)
target_link_libraries(MRA-components-falcons-trajectory-generation MRA-libraries MRA-base nlohmann_json::nlohmann_json)


add_executable(
    falcons_trajectory_generation_test
    test.cpp
)
target_include_directories(falcons_trajectory_generation_test PRIVATE ${MRA_SOURCE_DIR}/base ${MRA_SOURCE_DIR}/libraries/logging ${MRA_BINARY_DIR})
target_link_libraries(
    falcons_trajectory_generation_test
    MRA-components-falcons-trajectory-generation
    MRA-components-falcons-velocity-control
    MRA-libraries MRA-components nlohmann_json::nlohmann_json rt GTest::gtest_main gmock
)


gtest_discover_tests(
    falcons_trajectory_generation_test
    WORKING_DIRECTORY ${MRA_SOURCE_DIR}
    PROPERTIES ENVIRONMENT "MRA_LOGGER_CONTEXT=unittest"
)

