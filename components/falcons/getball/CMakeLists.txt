# this file was produced by MRA-codegen.py from template_CMakeLists.txt
# it should NOT be modified by user

add_library(MRA-components-falcons-getball
    tick.cpp
)
target_include_directories(MRA-components-falcons-getball PUBLIC
    ${MRA_SOURCE_DIR}/libraries/logging
    ${MRA_SOURCE_DIR}
    ${MRA_SOURCE_DIR}/base
    ${MRA_SOURCE_DIR}/components/falcons/getball_fetch
    ${MRA_SOURCE_DIR}/components/falcons/getball_intercept
    ${CMAKE_CURRENT_BINARY_DIR}/interface)
target_link_libraries(MRA-components-falcons-getball MRA-libraries MRA-base nlohmann_json::nlohmann_json)


add_executable(
    falcons_getball_test
    test.cpp
)
target_include_directories(falcons_getball_test PRIVATE ${MRA_SOURCE_DIR}/base ${MRA_SOURCE_DIR}/libraries/logging)
target_link_libraries(
    falcons_getball_test
    MRA-components-falcons-getball
    MRA-components-falcons-getball-fetch
    MRA-components-falcons-getball-intercept
    MRA-libraries MRA-components nlohmann_json::nlohmann_json rt GTest::gtest_main gmock
)


gtest_discover_tests(
    falcons_getball_test
    WORKING_DIRECTORY ${MRA_SOURCE_DIR}
    PROPERTIES ENVIRONMENT "MRA_LOGGER_CONTEXT=unittest"
)

