# this file was produced by MRA-codegen.py from template_CMakeLists.txt
# it should NOT be modified by user

add_library(MRA-components-robotsports-local-ball-tracking
    tick.cpp
    RobotsportsLocalBallTracking_preprocessing.cpp
    RobotsportsLocalBallTracking_sequence_clustering.cpp
    sequence_clustering_ball_model.cpp
	sequence_clustering_best_uid.cpp
)
target_include_directories(MRA-components-robotsports-local-ball-tracking PUBLIC
    ${MRA_SOURCE_DIR}/libraries/logging
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/base
    ${CMAKE_CURRENT_BINARY_DIR}/interface)
target_link_libraries(MRA-components-robotsports-local-ball-tracking MRA-libraries MRA-components-proto MRA-base nlohmann_json::nlohmann_json)


add_executable(
    robotsports_local_ball_tracking_test
    test.cpp
)
target_include_directories(robotsports_local_ball_tracking_test PRIVATE ${MRA_SOURCE_DIR}/base ${MRA_SOURCE_DIR}/libraries/logging)
target_link_libraries(
    robotsports_local_ball_tracking_test
    MRA-components-robotsports-local-ball-tracking
    MRA-libraries MRA-components-proto nlohmann_json::nlohmann_json rt GTest::gtest_main gmock
)


gtest_discover_tests(
    robotsports_local_ball_tracking_test
    WORKING_DIRECTORY ${MRA_SOURCE_DIR}
    PROPERTIES ENVIRONMENT "MRA_LOGGER_CONTEXT=unittest"
)

