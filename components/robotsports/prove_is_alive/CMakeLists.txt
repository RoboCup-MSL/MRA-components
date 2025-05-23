# this file was produced by MRA-codegen.py from template_CMakeLists.txt
# it should NOT be modified by user

add_library(MRA-components-robotsports-prove-is-alive
    tick.cpp
)
target_include_directories(MRA-components-robotsports-prove-is-alive PUBLIC
    ${MRA_SOURCE_DIR}/libraries/logging
    ${MRA_SOURCE_DIR}/libraries/geometry
    ${MRA_SOURCE_DIR}
    ${MRA_SOURCE_DIR}/base
    ${CMAKE_CURRENT_BINARY_DIR}/interface)
target_link_libraries(MRA-components-robotsports-prove-is-alive MRA-libraries MRA-base nlohmann_json::nlohmann_json)


add_executable(
    robotsports_prove_is_alive_test
    test.cpp
)
target_include_directories(robotsports_prove_is_alive_test PRIVATE ${MRA_SOURCE_DIR}/base ${MRA_SOURCE_DIR}/libraries/logging ${MRA_BINARY_DIR})
target_link_libraries(
    robotsports_prove_is_alive_test
    MRA-components-robotsports-prove-is-alive
    MRA-libraries MRA-datatypes MRA-components nlohmann_json::nlohmann_json rt GTest::gtest_main gmock
)


gtest_discover_tests(
    robotsports_prove_is_alive_test
    WORKING_DIRECTORY ${MRA_SOURCE_DIR}
    PROPERTIES ENVIRONMENT "MRA_LOGGER_CONTEXT=unittest"
)

