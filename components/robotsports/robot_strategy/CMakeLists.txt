# this file was produced by MRA-codegen.py from template_CMakeLists.txt
# it should NOT be modified by user

add_library(MRA-components-robotsports-robot-strategy
    tick.cpp
)
target_include_directories(MRA-components-robotsports-robot-strategy PUBLIC
    ${MRA_SOURCE_DIR}/libraries/logging
    ${MRA_SOURCE_DIR}
    ${MRA_SOURCE_DIR}/base
    ${CMAKE_CURRENT_BINARY_DIR}/interface
    ${CMAKE_CURRENT_SOURCE_DIR}/internal/include
)
target_link_libraries(MRA-components-robotsports-robot-strategy MRA-libraries MRA-base nlohmann_json::nlohmann_json)


add_executable(
    robotsports_robot_strategy_test
    test.cpp
)
target_include_directories(robotsports_robot_strategy_test PRIVATE 
	${MRA_SOURCE_DIR}/base 
	${MRA_SOURCE_DIR}/libraries/logging  
	${MRA_BINARY_DIR})
target_link_libraries(
    robotsports_robot_strategy_test
    MRA-components-robotsports-robot-strategy
    MRA-libraries MRA-components nlohmann_json::nlohmann_json rt GTest::gtest_main gmock
)


gtest_discover_tests(
    robotsports_robot_strategy_test
    WORKING_DIRECTORY ${MRA_SOURCE_DIR}
    PROPERTIES ENVIRONMENT "MRA_LOGGER_CONTEXT=unittest"
)

