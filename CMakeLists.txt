cmake_minimum_required(VERSION 3.5.1)

##########################################################
# general configuration
##########################################################

project(MRA LANGUAGES CXX)

cmake_policy(SET CMP0077 NEW) # option() command to do nothing when a normal variable of the given name already exists
option(MRA_TEST_ENABLED "Build all tests" ON)

# set build options (to reduce build time): 
#   can be switched off via ../MRA_build.txt --> set(<BUILD_XXX> 0) or set(<BUILD_XXX> 1)
set(BUILD_LIBRARY_REFLEXXES 1)
set(BUILD_LIBRARY_OPENCV_UTILS 1)
set(BUILD_LIBRARY_RUCKIG 1)

set(BUILD_FALCONS_ACTION_PLANNING 1)
set(BUILD_FALCONS_GETBALL 1)
set(BUILD_FALCONS_GETBALL_FETCH 1)
set(BUILD_FALCONS_GETBALL_INTERCEPT 1)
set(BUILD_FALCONS_LOCALIZATION_VISION 1)
set(BUILD_FALCONS_TRAJECTORY_GENERATION 1)
set(BUILD_FALCONS_VELOCITY_CONTROL 1)

set(BUILD_ROBOTSPORTS_GETBALL_FETCH 1)
set(BUILD_ROBOTSPORTS_GETBALL_INTERCEPT 1)
set(BUILD_ROBOTSPORTS_LOCAL_BALL 1)
set(BUILD_ROBOTSPORTS_LOCAL_BALL_PREPROCESSOR 1)
set(BUILD_ROBOTSPORTS_LOCAL_BALL_TRACKING 1)
set(BUILD_ROBOTSPORTS_LOCAL_OBSTACLE_TRACKING 1)
set(BUILD_ROBOTSPORTS_OBSTACLE_TRACKING 1)
set(BUILD_ROBOTSPORTS_PROVE_IS_ALIVE 1)
set(BUILD_ROBOTSPORTS_VELOCITY_CONTROL 1)


# use C++17 standard, or suffer errors re. using std::filesystem
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Werror -Wno-address -Wno-reorder -Wno-array-bounds -Wno-error=nonnull-compare")
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


include("../MRA_build.txt" OPTIONAL)


if (MRA_TEST_ENABLED)
  # dependency gtest
  find_package(GTest)
  if(NOT GTEST_FOUND)
    message(FATAL_ERROR "GTest could not be find by cmake. Install Gtest (e.g. sudo apt-get install libgtest-dev)")
  endif()
endif()

# dependency: json
include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
FetchContent_MakeAvailable(json)

##########################################################
# testing
##########################################################
if (MRA_TEST_ENABLED)
 enable_testing()
 include(CTest)
endif()

include(FetchContent)
option(RUCKIG_BUILD_EXAMPLES "build ruckig examples" OFF)
option(RUCKIG_BUILD_CLOUD_CLIENT "build ruckig cloud client" OFF)
option(RUCKIG_BUILD_TESTS "build ruckig tests" OFF)
option(RUCKIG_BUILD_SHARED_LIBS "build ruckig shared lib" OFF)
FetchContent_Declare(
    ruckig
    URL https://github.com/pantor/ruckig/archive/refs/tags/v0.14.0.tar.gz
)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
#FetchContent_MakeAvailable(ruckig googletest)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)


##########################################################
# common datatypes
##########################################################
# proto files for datatypes
set(MESSAGE_PROTOS_COMMON
    datatypes/ActionResult.proto
    datatypes/Ball.proto
    datatypes/CvMat.proto
    datatypes/Logging.proto
    datatypes/Meta.proto
    datatypes/Obstacle.proto
    datatypes/Player.proto
    datatypes/Point.proto
    datatypes/Pose.proto
    datatypes/PosVel.proto
    datatypes/Shapes.proto
    datatypes/WorldState.proto
)

add_library(MRA-datatypes-proto)
# generate code from protofiles and add them  intermediate library
protobuf_generate(TARGET MRA-datatypes-proto PROTOS ${MESSAGE_PROTOS_COMMON} LANGUAGE cpp IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${MRA_SOURCE_DIR})
file(GLOB MRA-datatypes_generated_files CONFIGURE_DEPENDS "${PROTO_BINARY_DIR}/datatype/*.pb.cc")
target_include_directories(MRA-datatypes-proto PUBLIC "$<BUILD_INTERFACE:${MRA_BINARY_DIR}>")
target_link_libraries(MRA-datatypes-proto PUBLIC ${MRA-datatypes_generated})

# create library for datatypes including the generated code
include_directories(datatypes)
add_library(MRA-datatypes OBJECT ${MESSAGE_PROTOS_COMMON})
target_link_libraries(MRA-datatypes PUBLIC protobuf::libprotobuf MRA-datatypes-proto)
set(PROTO_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(MRA-datatypes PUBLIC "$<BUILD_INTERFACE:${PROTO_BINARY_DIR}>")
target_link_libraries(MRA-datatypes ${MRA-datatypes_generated_files})     

# generate Python files from protofiles, copy to build/datatypes directory
protobuf_generate(TARGET MRA-datatypes-proto PROTOS ${MESSAGE_PROTOS_COMMON} LANGUAGE python OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/datatypes)
file(GLOB PYTHON_FILES "${CMAKE_CURRENT_BINARY_DIR}/datatypes/*.py")
foreach(PYTHON_FILE ${PYTHON_FILES})
    file(COPY ${PYTHON_FILE} DESTINATION "${CMAKE_BINARY_DIR}/datatypes")
endforeach()


##########################################################
# component proto files
##########################################################
# generate source from component proto files from toplevel to get correct header file guard 

# collect proto files from sub-directories below directory component
FILE(GLOB_RECURSE MESSAGE_PROTOS_COMPONENTS ABSOLUTE ${CMAKE_CURRENT_SOURCE_DIR}/components *.proto)
# generate code from protofiles and add them  intermediate library
add_library(MRA-components-protofiles)
protobuf_generate(TARGET MRA-components-protofiles PROTOS ${MESSAGE_PROTOS_COMPONENTS} LANGUAGE cpp IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${MRA_SOURCE_DIR})
file(GLOB MRA-components-generated-files CONFIGURE_DEPENDS "${PROTO_BINARY_DIR}/components/*/*/interface/*.pb.cc")
target_include_directories(MRA-components-protofiles PUBLIC "$<BUILD_INTERFACE:${MRA_BINARY_DIR}>")
target_link_libraries(MRA-components-protofiles PUBLIC ${MRA-components-generated-files})

# create library for MRA-component-proto with the generated code
add_library(MRA-components-proto OBJECT ${MESSAGE_PROTOS_COMPONENTS})
target_link_libraries(MRA-components-proto PUBLIC protobuf::libprotobuf MRA-components-protofiles ${MRA-components-generated-files})
set(PROTO_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(MRA-components-proto PUBLIC "$<BUILD_INTERFACE:${PROTO_BINARY_DIR}>")
#target_link_libraries(MRA-components-proto )     

# generate Python files from protofiles, copy to build/datatypes directory
protobuf_generate(TARGET MRA-components-proto PROTOS ${MESSAGE_PROTOS_COMPONENTS} LANGUAGE python)

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
add_subdirectory(third-party-libraries)


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
target_link_libraries(MRA-libraries MRA-libraries-dir MRA-third-party-libraries-dir)
target_link_directories(MRA-libraries PUBLIC ${MRA_BINARY_DIR}/libraries ${MRA_BINARY_DIR}/third-party-libraries)



