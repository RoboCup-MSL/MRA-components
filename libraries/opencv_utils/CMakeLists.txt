##########################################################
# component implementations
##########################################################

add_library(MRA-libraries-opencv-utils
	opencv_utils.cpp
)

target_include_directories(MRA-libraries-opencv-utils PUBLIC
	${MRA_BINARY_DIR}
)
add_dependencies(MRA-libraries-opencv-utils MRA-datatypes)
