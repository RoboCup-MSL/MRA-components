# this file was produced by MRA-codegen.py from template_CMakeLists.txt
# with the intent of allowing user to modify it

add_library(MRA-components-falcons-action-aimed-kick
    tick.cpp
)
target_include_directories(MRA-components-falcons-action-aimed-kick PUBLIC
    ${MRA_SOURCE_DIR}/libraries/logging
    ${MRA_SOURCE_DIR}/libraries/geometry
    ${MRA_SOURCE_DIR}
    ${MRA_SOURCE_DIR}/base
    ${CMAKE_CURRENT_BINARY_DIR}/interface)
target_link_libraries(MRA-components-falcons-action-aimed-kick MRA-libraries MRA-base nlohmann_json::nlohmann_json)


add_executable(
    falcons_action_aimed_kick_test
    test.cpp
)
target_include_directories(falcons_action_aimed_kick_test 
    PRIVATE 
        ${MRA_SOURCE_DIR}/base 
        ${MRA_SOURCE_DIR}/libraries/logging
        ${MRA_BINARY_DIR}
    )
target_link_libraries(
    falcons_action_aimed_kick_test
    MRA-libraries MRA-components nlohmann_json::nlohmann_json rt GTest::gtest_main gmock
)


gtest_discover_tests(
    falcons_action_aimed_kick_test
    WORKING_DIRECTORY ${MRA_SOURCE_DIR}
    PROPERTIES ENVIRONMENT "MRA_LOGGER_CONTEXT=unittest"
)


