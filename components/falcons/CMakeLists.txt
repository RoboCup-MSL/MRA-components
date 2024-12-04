if(BUILD_FALCONS_ACTION_AIMED_KICK)
   add_subdirectory(action_aimed_kick)
endif()

if(BUILD_FALCONS_ACTION_CATCH_BALL)
    add_subdirectory(action_catch_ball)
endif()

if(BUILD_FALCONS_ACTION_FETCH_BALL)
    add_subdirectory(action_fetch_ball)
endif()

if(BUILD_FALCONS_ACTION_GET_BALL)
    add_subdirectory(action_get_ball)
endif()

if(BUILD_FALCONS_ACTION_KEEPER)
    add_subdirectory(action_keeper)
endif()

if(BUILD_FALCONS_ACTION_MOVE)
    add_subdirectory(action_move)
endif()

if(BUILD_FALCONS_ACTION_PARK)
    add_subdirectory(action_park)
endif()

if(BUILD_FALCONS_ACTION_STOP)
    add_subdirectory(action_stop)
endif()

if(BUILD_FALCONS_ACTION_PLANNING)
   add_subdirectory(action_planning)
endif()

if(BUILD_FALCONS_LOCALIZATION_VISION)
    add_subdirectory(localization_vision)
endif()

if(BUILD_FALCONS_TRAJECTORY_GENERATION)
    add_subdirectory(trajectory_generation)
endif()

if(BUILD_FALCONS_VELOCITY_CONTROL)
    add_subdirectory(velocity_control)
endif()

add_subdirectory(test_mra_logger)

add_library(MRA-components-falcons ${MRA_BINARY_DIR}/dummy.cpp)
target_link_libraries(MRA-components-falcons
    $<IF:${BUILD_FALCONS_ACTION_AIMED_KICK},MRA-components-falcons-action-aimed-kick,>
    $<IF:${BUILD_FALCONS_ACTION_CATCH_BALL},MRA-components-falcons-action-catch-ball,>
    $<IF:${BUILD_FALCONS_ACTION_FETCH_BALL},MRA-components-falcons-action-fetch-ball,>
    $<IF:${BUILD_FALCONS_ACTION_GET_BALL},MRA-components-falcons-action-get-ball,>
    $<IF:${BUILD_FALCONS_ACTION_KEEPER},MRA-components-falcons-action-keeper,>
    $<IF:${BUILD_FALCONS_ACTION_MOVE},MRA-components-falcons-action-move,>
    $<IF:${BUILD_FALCONS_ACTION_PARK},MRA-components-falcons-action-park,>
    $<IF:${BUILD_FALCONS_ACTION_PLANNING},MRA-components-falcons-action-planning,>
    $<IF:${BUILD_FALCONS_ACTION_STOP},MRA-components-falcons-action-stop,>
    $<IF:${BUILD_FALCONS_LOCALIZATION_VISION},MRA-components-falcons-localization-vision,>
    $<IF:${BUILD_FALCONS_TRAJECTORY_GENERATION},MRA-components-falcons-trajectory-generation,>
    $<IF:${BUILD_FALCONS_VELOCITY_CONTROL},MRA-components-falcons-velocity-control,>
    MRA-components-falcons-test-mra-logger
)

target_link_directories(MRA-components-falcons PUBLIC
    $<IF:${BUILD_FALCONS_ACTION_AIMED_KICK},${MRA_BINARY_DIR}/components/falcons/action_aimed_kick,>
    $<IF:${BUILD_FALCONS_ACTION_CATCH_BALL},${MRA_BINARY_DIR}/components/falcons/action_catch_ball,>
    $<IF:${BUILD_FALCONS_ACTION_FETCH_BALL},${MRA_BINARY_DIR}/components/falcons/action_fetch_ball,>
    $<IF:${BUILD_FALCONS_ACTION_GET_BALL},${MRA_BINARY_DIR}/components/falcons/action_get_ball,>
    $<IF:${BUILD_FALCONS_ACTION_KEEPER},${MRA_BINARY_DIR}/components/falcons/action_keeper,>
    $<IF:${BUILD_FALCONS_ACTION_MOVE},${MRA_BINARY_DIR}/components/falcons/action_move,>
    $<IF:${BUILD_FALCONS_ACTION_PARK},${MRA_BINARY_DIR}/components/falcons/action_park,>
    $<IF:${BUILD_FALCONS_ACTION_PLANNING},${MRA_BINARY_DIR}/components/falcons/action_planning,>
    $<IF:${BUILD_FALCONS_ACTION_STOP},${MRA_BINARY_DIR}/components/falcons/action_stop,>
    $<IF:${BUILD_FALCONS_LOCALIZATION_VISION},${MRA_BINARY_DIR}/components/falcons/localization_vision,>
    $<IF:${BUILD_FALCONS_TRAJECTORY_GENERATION},${MRA_BINARY_DIR}/components/falcons/trajectory_generation,>
    $<IF:${BUILD_FALCONS_VELOCITY_CONTROL},${MRA_BINARY_DIR}/components/falcons/velocity_control,>
    ${MRA_BINARY_DIR}/components/falcons/test_mra_logger
)

