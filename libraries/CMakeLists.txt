add_subdirectory(geometry)
add_subdirectory(logging)
if(BUILD_LIBRARY_OPENCV_UTILS)
    add_subdirectory(opencv_utils)
endif()

add_library(MRA-libraries-dir ${MRA_BINARY_DIR}/dummy.cpp)

target_link_libraries(MRA-libraries-dir
                    MRA-libraries-geometry
                    MRA-libraries-logging
                    $<IF:${BUILD_LIBRARY_OPENCV_UTILS},MRA-libraries-opencv-utils,>
)

target_link_directories(MRA-libraries-dir PUBLIC
    ${MRA_BINARY_DIR}/libraries/geometry
    ${MRA_BINARY_DIR}/libraries/logging
    $<IF:${BUILD_LIBRARY_OPENCV_UTILS},${MRA_BINARY_DIR}/libraries/opencv_utils,>
)


