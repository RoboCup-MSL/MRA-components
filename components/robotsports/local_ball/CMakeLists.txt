# this file was produced by MRA-codegen.py from template_CMakeLists.txt
# with the intent of allowing user to modify it

add_library(MRA-components-robotsports-local-ball
    tick.cpp
)
target_include_directories(MRA-components-robotsports-local-ball PUBLIC
    ${MRA_SOURCE_DIR}/libraries/logging
    ${MRA_SOURCE_DIR}
    ${MRA_SOURCE_DIR}/base
    ${MRA_SOURCE_DIR}/components/robotsports/local_ball_preprocessor
    ${MRA_SOURCE_DIR}/components/robotsports/local_ball_tracking
    ${CMAKE_CURRENT_BINARY_DIR}/interface)
target_link_libraries(MRA-components-robotsports-local-ball MRA-libraries MRA-base nlohmann_json::nlohmann_json)


add_executable(
    robotsports_local_ball_test
    test.cpp
)
target_include_directories(robotsports_local_ball_test PRIVATE ${MRA_SOURCE_DIR}/base ${MRA_SOURCE_DIR}/libraries/logging)
target_link_libraries(
    robotsports_local_ball_test
    MRA-components-robotsports-local-ball
    MRA-components-robotsports-local-ball-preprocessor
    MRA-components-robotsports-local-ball-tracking
    MRA-libraries MRA-components nlohmann_json::nlohmann_json rt GTest::gtest_main gmock
)


gtest_discover_tests(
    robotsports_local_ball_test
    WORKING_DIRECTORY ${MRA_SOURCE_DIR}
    PROPERTIES ENVIRONMENT "MRA_LOGGER_CONTEXT=unittest"
)

