if(BUILD_ROBOTSPORTS_GETBALL_FETCH)
    add_subdirectory(getball_fetch)
endif()
if(BUILD_ROBOTSPORTS_GETBALL_INTERCEPT)
    add_subdirectory(getball_intercept)
endif()
if(BUILD_ROBOTSPORTS_LOCAL_BALL)
    add_subdirectory(local_ball)
endif()
if(BUILD_ROBOTSPORTS_LOCAL_BALL_PREPROCESSOR)
    add_subdirectory(local_ball_preprocessor)
endif()
if(BUILD_ROBOTSPORTS_LOCAL_BALL_TRACKING)
    add_subdirectory(local_ball_tracking)
endif()
if(BUILD_ROBOTSPORTS_LOCAL_OBSTACLE_TRACKING)
    add_subdirectory(local_obstacle_tracking)
endif()
if(BUILD_ROBOTSPORTS_OBSTACLE_TRACKING)
    add_subdirectory(obstacle_tracking)
endif()
if(BUILD_ROBOTSPORTS_PROVE_IS_ALIVE)
    add_subdirectory(prove_is_alive)
endif()
if(BUILD_ROBOTSPORTS_ROBOT_STRATEGY)
    add_subdirectory(robot_strategy)
endif()
if(BUILD_ROBOTSPORTS_VELOCITY_CONTROL)
    add_subdirectory(velocity_control)
endif()

add_library(MRA-components-robotsports ${MRA_BINARY_DIR}/dummy.cpp)
target_link_libraries(MRA-components-robotsports
    $<IF:${BUILD_ROBOTSPORTS_GETBALL_FETCH},MRA-components-robotsports-getball-fetch,>
    $<IF:${BUILD_ROBOTSPORTS_GETBALL_INTERCEPT},MRA-components-robotsports-getball-intercept,>
    $<IF:${BUILD_ROBOTSPORTS_LOCAL_BALL},MRA-components-robotsports-local-ball,>
    $<IF:${BUILD_ROBOTSPORTS_LOCAL_BALL_PREPROCESSOR},MRA-components-robotsports-local-ball-preprocessor,>
    $<IF:${BUILD_ROBOTSPORTS_LOCAL_BALL_TRACKING},MRA-components-robotsports-local-ball-tracking,>
    $<IF:${BUILD_ROBOTSPORTS_LOCAL_OBSTACLE_TRACKING},MRA-components-robotsports-local-obstacle-tracking,>
    $<IF:${BUILD_ROBOTSPORTS_OBSTACLE_TRACKING},MRA-components-robotsports-obstacle-tracking,>
    $<IF:${BUILD_ROBOTSPORTS_PROVE_IS_ALIVE},MRA-components-robotsports-prove-is-alive,>
    $<IF:${BUILD_ROBOTSPORTS_ROBOT_STRATEGY},MRA-components-robotsports-robot-strategy,>
    $<IF:${BUILD_ROBOTSPORTS_VELOCITY_CONTROL},MRA-components-robotsports-velocity-control,>
)

target_link_directories(MRA-components-robotsports PUBLIC
    $<IF:${BUILD_ROBOTSPORTS_GETBALL_FETCH},${MRA_BINARY_DIR}/components/robotsports/getball_fetch,>
    $<IF:${BUILD_ROBOTSPORTS_GETBALL_INTERCEPT},${MRA_BINARY_DIR}/components/robotsports/getball_intercept,>
    $<IF:${BUILD_ROBOTSPORTS_LOCAL_BALL},${MRA_BINARY_DIR}/components/robotsports/local_ball,>
    $<IF:${BUILD_ROBOTSPORTS_LOCAL_BALL_PREPROCESSOR},${MRA_BINARY_DIR}/components/robotsports/local_ball_preprocessor,>
    $<IF:${BUILD_ROBOTSPORTS_LOCAL_BALL_TRACKING},${MRA_BINARY_DIR}/components/robotsports/local_ball_tracking,>
    $<IF:${BUILD_ROBOTSPORTS_LOCAL_BALL_TRACKING},${MRA_BINARY_DIR}/components/robotsports/local_obstacle_tracking,>
    $<IF:${BUILD_ROBOTSPORTS_OBSTACLE_TRACKING},${MRA_BINARY_DIR}/components/robotsports/obstacle_tracking,>
    $<IF:${BUILD_ROBOTSPORTS_PROVE_IS_ALIVE},${MRA_BINARY_DIR}/components/robotsports/prove_is_alive,>
    $<IF:${BUILD_ROBOTSPORTS_ROBOT_STRATEGY},${MRA_BINARY_DIR}/components/robotsports/robot_strategy,>
    $<IF:${BUILD_ROBOTSPORTS_VELOCITY_CONTROL},${MRA_BINARY_DIR}/components/robotsports/velocity_control,>
)

