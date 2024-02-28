cmake_minimum_required(VERSION 3.5.1)

##########################################################
# general configuration
##########################################################

project(MRA LANGUAGES CXX)

# use C++17 standard, or suffer errors re. using std::filesystem
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Werror -Wno-address -Wno-reorder -Wno-array-bounds")
 # to prevent compiler errors added the not preferred flags: -Wno-address -Wno-reorder -Wno-array-bounds

# dependency: ProtoBuf
# check if ProtoBuf is installed
# install on Ubuntu: sudo apt-get install protobuf-compiler
find_package(Protobuf)
if(NOT PROTOBUF_FOUND)
message(FATAL_ERROR "Protobuf could not be find by cmake. Install protobuf (e.g. sudo apt-get install protobuf-compiler)")
endif(NOT PROTOBUF_FOUND)
find_library(PROTOBUF_LIBRARY protobuf)

# dependency: opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# dependency gtest 
find_package(GTest)
if(NOT GTEST_FOUND)
message(FATAL_ERROR "GTest could not be find by cmake. Install Gtest (e.g. sudo apt-get install libgtest-dev)")
endif()

# dependency: json
include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
FetchContent_MakeAvailable(json)

##########################################################
# testing
##########################################################
enable_testing()
include(CTest)

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)


##########################################################
# common datatypes
##########################################################

set(MESSAGE_PROTOS_COMMON
    datatypes/ActionResult.proto
    datatypes/Ball.proto
    datatypes/CvMat.proto
    datatypes/Logging.proto
    datatypes/Obstacle.proto
    datatypes/Player.proto
    datatypes/Point.proto
    datatypes/Pose.proto
    datatypes/PosVel.proto
    datatypes/Shapes.proto
    datatypes/WorldState.proto
)

add_library(MRA-datatypes-proto)
protobuf_generate(TARGET MRA-datatypes-proto PROTOS ${MESSAGE_PROTOS_COMMON} LANGUAGE cpp IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${MRA_SOURCE_DIR})
foreach (protobuf_file ${MESSAGE_PROTOS_COMMON})
    get_filename_component(proto_name ${protobuf_file} NAME_WE)
    get_filename_component(proto_path ${protobuf_file} ABSOLUTE)
    set_source_files_properties(
        "${proto_path}/${proto_name}.pb.cc"
        "${proto_path}/${proto_name}.pb.h"
        PROPERTIES GENERATED TRUE)
endforeach()
file(GLOB MRA-datatypes_generated_files CONFIGURE_DEPENDS "${PROTO_BINARY_DIR}/datatype/*.pb.cc")
target_include_directories(MRA-datatypes-proto PUBLIC "$<BUILD_INTERFACE:${MRA_BINARY_DIR}>")
target_link_libraries(MRA-datatypes-proto PUBLIC ${MRA-datatypes_generated})

include_directories(datatypes)
add_library(MRA-datatypes OBJECT ${MESSAGE_PROTOS_COMMON})
target_link_libraries(MRA-datatypes PUBLIC protobuf::libprotobuf MRA-datatypes-proto)
set(PROTO_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(MRA-datatypes PUBLIC "$<BUILD_INTERFACE:${PROTO_BINARY_DIR}>")
target_link_libraries(MRA-datatypes ${MRA-datatypes_generated_files})     


##########################################################
# component proto files
##########################################################
# generate source from component proto files from toplevel to get correct header file guard 

# collect proto files from sub-directories below directory component
FILE(GLOB_RECURSE MESSAGE_PROTOS_COMPONENTS ABSOLUTE ${CMAKE_CURRENT_SOURCE_DIR}/components *.proto)
add_library(MRA-components-protofiles)
protobuf_generate(TARGET MRA-components-protofiles PROTOS ${MESSAGE_PROTOS_COMPONENTS} LANGUAGE cpp IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${MRA_SOURCE_DIR})
foreach (protobuf_file ${MESSAGE_PROTOS_COMPONENTS})
    get_filename_component(proto_name ${protobuf_file} NAME_WE)
    get_filename_component(proto_path ${MRA_BINARY_DIR}/datatypes ABSOLUTE)
    message(STATUS "${proto_path}/${proto_name}.pb.cc")
    set_source_files_properties(
        "${proto_path}/${proto_name}.pb.cc"
        "${proto_path}/${proto_name}.pb.h"
        PROPERTIES GENERATED TRUE)
endforeach()
file(GLOB MRA-components-generated-files CONFIGURE_DEPENDS "${PROTO_BINARY_DIR}/datatype/*.pb.cc")
target_include_directories(MRA-components-protofiles PUBLIC "$<BUILD_INTERFACE:${MRA_BINARY_DIR}>")
target_link_libraries(MRA-components-protofiles PUBLIC ${MRA-components-generated-files})

add_library(MRA-components-proto OBJECT ${MESSAGE_PROTOS_COMMON})
target_link_libraries(MRA-components-proto PUBLIC protobuf::libprotobuf MRA-components-protofiles)
set(PROTO_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(MRA-components-proto PUBLIC "$<BUILD_INTERFACE:${PROTO_BINARY_DIR}>")
target_link_libraries(MRA-components-proto ${MRA-components-generated-files})     



# create dummy file for creating libraries with only ohter libraries
file(TOUCH ${MRA_BINARY_DIR}/dummy.cpp)


##########################################################
# subdirectories
##########################################################
#TODO use datatypes/CMakeLists.txt:
# add_subdirectory(datatypes)
add_subdirectory(base)

add_subdirectory(components)
add_subdirectory(libraries)



##########################################################
# create libraries with input from subdirectories
##########################################################
# TODO use datatypes/CMakeLists.txt
#add_library(MRA-datatypes ${MRA_BINARY_DIR}/dummy.cpp)
#target_link_libraries(MRA-datatypes
#    MRA-libraries-dir)
#target_link_directories(MRA-datatypes PUBLIC
#    ${MRA_BINARY_DIR}/datatypes
#)

add_library(MRA-components ${MRA_BINARY_DIR}/dummy.cpp)
target_link_libraries(MRA-components MRA-components-dir)
target_link_directories(MRA-components PUBLIC ${MRA_BINARY_DIR}/components)

add_library(MRA-libraries ${MRA_BINARY_DIR}/dummy.cpp)
target_link_libraries(MRA-libraries MRA-libraries-dir)
target_link_directories(MRA-libraries PUBLIC ${MRA_BINARY_DIR}/libraries)



