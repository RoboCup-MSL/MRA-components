##########################################################
# component implementations
##########################################################

add_library(MRA-libraries-geometry
	angles.cpp
    point.cpp
    pose.cpp
    position.cpp
    velocity.cpp
    intersect.cpp
)

target_include_directories(MRA-libraries-geometry PUBLIC
	${MRA_BINARY_DIR}
)
target_link_libraries(MRA-libraries-geometry MRA-datatypes)
add_dependencies(MRA-libraries-geometry MRA-datatypes)



#-----------------#
#  testing point  #
#-----------------#
add_executable(
    libraries_geometry_point_tests
    test-point.cpp
)
target_include_directories(libraries_geometry_point_tests PRIVATE ${MRA_SOURCE_DIR}/base ${MRA_SOURCE_DIR}/libraries/logging)
target_link_libraries(
    libraries_geometry_point_tests
    MRA-libraries-geometry MRA-datatypes MRA-datatypes-proto nlohmann_json::nlohmann_json rt GTest::gtest_main gmock
)

gtest_discover_tests(
    libraries_geometry_point_tests
    WORKING_DIRECTORY ${MRA_SOURCE_DIR}
    PROPERTIES ENVIRONMENT "MRA_LOGGER_CONTEXT=unittest"
)


#-----------------#
#  testing pose   #
#-----------------#
add_executable(
    libraries_geometry_pose_tests
    test-pose.cpp
)
target_include_directories(libraries_geometry_pose_tests PRIVATE ${MRA_SOURCE_DIR}/base ${MRA_SOURCE_DIR}/libraries/logging)
target_link_libraries(
    libraries_geometry_pose_tests
    MRA-libraries-geometry MRA-datatypes MRA-datatypes-proto nlohmann_json::nlohmann_json rt GTest::gtest_main gmock
)
gtest_discover_tests(
    libraries_geometry_pose_tests
    WORKING_DIRECTORY ${MRA_SOURCE_DIR}
    PROPERTIES ENVIRONMENT "MRA_LOGGER_CONTEXT=unittest"
)

# -----------------#
# testing position #
# -----------------#
add_executable(
    libraries_geometry_position_tests
    test-position.cpp
)
target_include_directories(libraries_geometry_position_tests PRIVATE ${MRA_SOURCE_DIR}/base ${MRA_SOURCE_DIR}/libraries/logging)
target_link_libraries(
    libraries_geometry_position_tests
    MRA-libraries-geometry MRA-datatypes MRA-datatypes-proto nlohmann_json::nlohmann_json rt GTest::gtest_main gmock
)

gtest_discover_tests(
    libraries_geometry_position_tests
    WORKING_DIRECTORY ${MRA_SOURCE_DIR}
    PROPERTIES ENVIRONMENT "MRA_LOGGER_CONTEXT=unittest"
)
