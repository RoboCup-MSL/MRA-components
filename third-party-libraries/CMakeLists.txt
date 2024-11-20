#-------------------------------------------------------------
# third-party libraries: intention to remove this directory 
#-------------------------------------------------------------
if(BUILD_LIBRARY_REFLEXXES)
    add_subdirectory(reflexxes)
endif()
if(BUILD_LIBRARY_RUCKIG)
    add_subdirectory(ruckig)
endif()

add_library(MRA-third-party-libraries-dir ${MRA_BINARY_DIR}/dummy.cpp)

target_link_libraries(MRA-third-party-libraries-dir
                    $<IF:${BUILD_LIBRARY_REFLEXXES},MRA-libraries-reflexxes,>
                    $<IF:${BUILD_LIBRARY_RUCKIG},MRA-libraries-ruckig,>
)

target_link_directories(MRA-third-party-libraries-dir PUBLIC
    $<IF:${BUILD_LIBRARY_REFLEXXES},${MRA_BINARY_DIR}/libraries/reflexxes,>
    $<IF:${BUILD_LIBRARY_RUCKIG}, ${MRA_BINARY_DIR}/libraries/ruckig,>
)


