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
set(BUILD_ROBOTSPORTS_ROBOT_STRATEGY 1)
set(BUILD_ROBOTSPORTS_VELOCITY_CONTROL 1)

set(MRA_PROTO_COMPONENT_LIB "${MRA_BINARY_DIR}/lib/libMRA-components-proto.a")
# overwrite setting above via ../MRA_build.txt (if exists)
include("../MRA_build.txt" OPTIONAL)

message(INFO " The following MRA components are build:")
message(INFO " LIBRARY_REFLEXXES =  ${BUILD_LIBRARY_REFLEXXES}")
message(INFO " LIBRARY_OPENCV_UTILS =  ${BUILD_LIBRARY_OPENCV_UTILS}")
message(INFO " LIBRARY_RUCKIG =  ${BUILD_LIBRARY_RUCKIG}")

message(INFO " FALCONS_ACTION_PLANNING =  ${BUILD_FALCONS_ACTION_PLANNING}")
message(INFO " FALCONS_GETBALL =  ${BUILD_FALCONS_GETBALL}")
message(INFO " FALCONS_GETBALL_FETCH =  ${BUILD_FALCONS_GETBALL_FETCH}")
message(INFO " FALCONS_GETBALL_INTERCEPT =  ${BUILD_FALCONS_GETBALL_INTERCEPT}")
message(INFO " FALCONS_LOCALIZATION_VISION =  ${BUILD_FALCONS_LOCALIZATION_VISION}")
message(INFO " FALCONS_TRAJECTORY_GENERATION =  ${BUILD_FALCONS_TRAJECTORY_GENERATION}")
message(INFO " FALCONS_VELOCITY_CONTROL =  ${BUILD_FALCONS_VELOCITY_CONTROL}")

message(INFO " ROBOTSPORTS_GETBALL_FETCH =  ${BUILD_ROBOTSPORTS_GETBALL_FETCH}")
message(INFO " ROBOTSPORTS_GETBALL_INTERCEPT =  ${BUILD_ROBOTSPORTS_GETBALL_INTERCEPT}")
message(INFO " ROBOTSPORTS_LOCAL_BALL =  ${BUILD_ROBOTSPORTS_LOCAL_BALL}")
message(INFO " ROBOTSPORTS_LOCAL_BALL_PREPROCESSOR =  ${BUILD_ROBOTSPORTS_LOCAL_BALL_PREPROCESSOR}")
message(INFO " ROBOTSPORTS_LOCAL_BALL_TRACKING =  ${BUILD_ROBOTSPORTS_LOCAL_BALL_TRACKING}")
message(INFO " ROBOTSPORTS_LOCAL_OBSTACLE_TRACKING =  ${BUILD_ROBOTSPORTS_LOCAL_OBSTACLE_TRACKING}")
message(INFO " ROBOTSPORTS_OBSTACLE_TRACKING =  ${BUILD_ROBOTSPORTS_OBSTACLE_TRACKING}")
message(INFO " ROBOTSPORTS_PROVE_IS_ALIVE =  ${BUILD_ROBOTSPORTS_PROVE_IS_ALIVE}")
message(INFO " ROBOTSPORTS_ROBOT_STRATEGY =  ${BUILD_ROBOTSPORTS_ROBOT_STRATEGY}")
message(INFO " ROBOTSPORTS_VELOCITY_CONTROL =  ${BUILD_ROBOTSPORTS_VELOCITY_CONTROL}")

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
# Future: use datatypes/CMakeLists.txt instead of building at toplevel

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
target_sources(MRA-datatypes-proto PUBLIC ${MRA-datatypes_generated_files})
target_link_libraries(MRA-datatypes-proto PUBLIC ${MRA-datatypes_generated})

# create library for datatypes including the generated code
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
# Generate source from component proto files from toplevel to get correct header file guard 
# require to run protobuf_generate for the toplevel. 
# Running it for the top makes rebuild (build after the initial build) tricky
# To avoid endless loops and/or missing files: protofiles and the generated source are not rebuild (only if lib MRA-components-proto is not present)

# generate code from protofiles and add them  intermediate library
FILE(GLOB_RECURSE MESSAGE_PROTOS_COMPONENTS ABSOLUTE ${CMAKE_CURRENT_SOURCE_DIR}/components *.proto)
add_library(MRA-components-proto)
protobuf_generate(TARGET MRA-components-proto PROTOS ${MESSAGE_PROTOS_COMPONENTS} LANGUAGE cpp IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${MRA_SOURCE_DIR})
if (NOT EXISTS ${MRA_PROTO_COMPONENT_LIB})
  file(GLOB_RECURSE MRA-components-generated-files "${PROTO_BINARY_DIR}/components/*/*/interface/*.pb.*")
endif()
target_include_directories(MRA-components-proto PUBLIC "$<BUILD_INTERFACE:${MRA_BINARY_DIR}>")
target_sources(MRA-components-proto PUBLIC ${MRA-components-generated-files})
target_link_libraries(MRA-components-proto ${MRA-components-generated-files})
set(MESSAGE_PROTOS_COMPONENTS "")
set(src_file_names "")
FILE(GLOB_RECURSE MESSAGE_PROTOS_COMPONENTS "${CMAKE_CURRENT_SOURCE_DIR}/components/**/*.proto")
if (NOT EXISTS ${MRA_PROTO_COMPONENT_LIB})
  FILE(GLOB_RECURSE generated_sources CONFIGURE_DEPENDS "${MRA_BINARY_DIR}/components/**/*.pb.*")
  target_sources(MRA-components-proto PUBLIC ${generated_sources})
endif()
add_library(MRA-components-protofiles OBJECT ${MESSAGE_PROTOS_COMPONENTS})
add_dependencies(MRA-components-protofiles MRA-components-proto)
target_include_directories(MRA-components-protofiles PRIVATE "$<BUILD_INTERFACE:${PROTO_BINARY_DIR}>")
target_link_libraries(MRA-components-protofiles PRIVATE protobuf::libprotobuf ${MRA-components-generated-files} MRA-components-proto)     

# generate Python files from protofiles, copy to build/datatypes directory
protobuf_generate(TARGET MRA-components-proto PROTOS ${MESSAGE_PROTOS_COMPONENTS} LANGUAGE python OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
file(GLOB PYTHON_FILES "${CMAKE_CURRENT_BINARY_DIR}/components/*/*/interface/*.py")
foreach(PYTHON_FILE ${PYTHON_FILES})
  file(COPY ${PYTHON_FILE} DESTINATION "${CMAKE_BINARY_DIR}")
endforeach()





# create dummy file for creating libraries with only ohter libraries
file(TOUCH ${MRA_BINARY_DIR}/dummy.cpp)

##########################################################
# subdirectories
##########################################################
add_subdirectory(base)

add_subdirectory(components)
add_subdirectory(libraries)
add_subdirectory(third-party-libraries)


##########################################################
# create libraries with input from subdirectories
##########################################################

add_library(MRA-components ${MRA_BINARY_DIR}/dummy.cpp)
add_dependencies(MRA-components MRA-components-protofiles)
target_link_libraries(MRA-components MRA-components-dir MRA-components-protofiles)

add_library(MRA-libraries ${MRA_BINARY_DIR}/dummy.cpp)
target_link_libraries(MRA-libraries MRA-libraries-dir MRA-third-party-libraries-dir)
target_link_directories(MRA-libraries PUBLIC ${MRA_BINARY_DIR}/libraries ${MRA_BINARY_DIR}/third-party-libraries)



