add_subdirectory(getball_fetch)
add_subdirectory(getball_intercept)
add_subdirectory(local_ball)
add_subdirectory(local_ball_preprocessor)
add_subdirectory(local_ball_tracking)
add_subdirectory(local_obstacle_tracking)
add_subdirectory(obstacle_tracking)
add_subdirectory(prove_is_alive)
add_subdirectory(velocity_control)

add_library(MRA-components-robotsports ${MRA_BINARY_DIR}/dummy.cpp)
target_link_libraries(MRA-components-robotsports
    MRA-components-robotsports-getball-fetch
    MRA-components-robotsports-getball-intercept
    MRA-components-robotsports-local-ball
    MRA-components-robotsports-local-ball-preprocessor
    MRA-components-robotsports-local-ball-tracking
    MRA-components-robotsports-local-obstacle-tracking
    MRA-components-robotsports-obstacle-tracking
    MRA-components-robotsports-prove-is-alive
    MRA-components-robotsports-velocity-control
)

target_link_directories(MRA-components-robotsports PUBLIC
    ${MRA_BINARY_DIR}/components/robotsports/getball_fetch
    ${MRA_BINARY_DIR}/components/robotsports/getball_intercept
    ${MRA_BINARY_DIR}/components/robotsports/local_ball
    ${MRA_BINARY_DIR}/components/robotsports/local_ball_preprocessor
    ${MRA_BINARY_DIR}/components/robotsports/local_ball_tracking
    ${MRA_BINARY_DIR}/components/robotsports/local_obstacle_tracking
    ${MRA_BINARY_DIR}/components/robotsports/obstacle_tracking
    ${MRA_BINARY_DIR}/components/robotsports/prove_is_alive
    ${MRA_BINARY_DIR}/components/robotsports/velocity_control
)

