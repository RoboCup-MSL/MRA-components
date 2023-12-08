# this file was produced by MRA-codegen.py from template_CMakeLists.txt
# with the intent of allowing user to modify it

add_library(MRA-components-robotsports-obstacle-tracking
    tick.cpp
    internal/obstacle_tracking.cpp
    internal/sequence_clustering_track_objects.cpp
)

target_include_directories(MRA-components-robotsports-obstacle-tracking PUBLIC
    ${MRA_SOURCE_DIR}/libraries/logging
    ${MRA_SOURCE_DIR}
    ${MRA_SOURCE_DIR}/base
    ${MRA_SOURCE_DIR}/components/robotsports/obstacle_tracking    
    ${MRA_SOURCE_DIR}/components/robotsports/obstacle_tracking/internal
    ${CMAKE_CURRENT_BINARY_DIR}/interface)
target_link_libraries(MRA-components-robotsports-obstacle-tracking MRA-libraries MRA-components-proto MRA-base nlohmann_json::nlohmann_json)


add_executable(
    robotsports_obstacle_tracking_test
    test.cpp
)
target_include_directories(robotsports_obstacle_tracking_test PRIVATE ${MRA_SOURCE_DIR}/base ${MRA_SOURCE_DIR}/libraries/logging)
target_link_libraries(
    robotsports_obstacle_tracking_test
    MRA-components-robotsports-obstacle-tracking
    MRA-libraries MRA-components-proto nlohmann_json::nlohmann_json rt GTest::gtest_main gmock
)


gtest_discover_tests(
    robotsports_obstacle_tracking_test
    WORKING_DIRECTORY ${MRA_SOURCE_DIR}
    PROPERTIES ENVIRONMENT "MRA_LOGGER_CONTEXT=unittest"
)

