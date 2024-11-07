##########################################################
# component implementations
##########################################################

add_library(MRA-libraries-ruckig
  ruckig/src/ruckig/brake.cpp
  ruckig/src/ruckig/position_first_step1.cpp
  ruckig/src/ruckig/position_first_step2.cpp
  ruckig/src/ruckig/position_second_step1.cpp
  ruckig/src/ruckig/position_second_step2.cpp
  ruckig/src/ruckig/position_third_step1.cpp
  ruckig/src/ruckig/position_third_step2.cpp
  ruckig/src/ruckig/velocity_second_step1.cpp
  ruckig/src/ruckig/velocity_second_step2.cpp
  ruckig/src/ruckig/velocity_third_step1.cpp
  ruckig/src/ruckig/velocity_third_step2.cpp
)

target_include_directories(MRA-libraries-ruckig PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/ruckig/include
	${MRA_SOURCE_DIR}/libraries/geometry
)
