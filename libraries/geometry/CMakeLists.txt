##########################################################
# component implementations
##########################################################

add_library(MRA-libraries-geometry
	angles.cpp
    point.cpp
    pose.cpp
    position.cpp
    velocity.cpp
)

target_include_directories(MRA-libraries-geometry PUBLIC
	${MRA_BINARY_DIR}
)
target_link_libraries(MRA-libraries-geometry MRA-datatypes)
add_dependencies(MRA-libraries-geometry MRA-datatypes)
