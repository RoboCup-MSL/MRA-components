# this file was produced by MRA-codegen.py from template_CMakeLists.txt
# it should NOT be modified by user

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})



add_library(MRA-components-falcons-localization-vision
    tick.cpp
    internal/fit.cpp
    internal/floor.cpp
    internal/guessing.cpp
    internal/solver.cpp
    internal/tracker.cpp
)

target_include_directories(MRA-components-falcons-localization-vision PUBLIC
    ${MRA_SOURCE_DIR}/libraries/logging
    ${MRA_SOURCE_DIR}
    ${MRA_SOURCE_DIR}/base
    ${MRA_SOURCE_DIR}/libraries/opencv_utils
    ${MRA_SOURCE_DIR}/libraries/geometry
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/internal
    ${CMAKE_CURRENT_BINARY_DIR}/interface)
target_link_libraries(MRA-components-falcons-localization-vision MRA-libraries MRA-components-proto MRA-base nlohmann_json::nlohmann_json ${OpenCV_LIBRARIES})


add_executable(
    falcons_localization_vision_test
    test.cpp
)
target_include_directories(falcons_localization_vision_test PRIVATE ${MRA_SOURCE_DIR}/base ${MRA_SOURCE_DIR}/libraries/logging)
target_link_libraries(
    falcons_localization_vision_test
    MRA-components-falcons-localization-vision
    MRA-libraries MRA-components-proto nlohmann_json::nlohmann_json rt GTest::gtest_main gmock
)


gtest_discover_tests(
    falcons_localization_vision_test
    WORKING_DIRECTORY ${MRA_SOURCE_DIR}
    PROPERTIES ENVIRONMENT "MRA_LOGGER_CONTEXT=unittest"
)

