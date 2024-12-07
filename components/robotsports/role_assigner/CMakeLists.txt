# this file was produced by MRA-codegen.py from template_CMakeLists.txt
# it should NOT be modified by user
set(STRATEGY_TESTER_XSD testdata/StrategyTester.xsd)
set(STRATEGY_TESTER_H StrategyTester_generated.h)
set(STRATEGY_TESTER_CPP StrategyTester_generated.cpp)

# system dependencies
find_package(Boost REQUIRED COMPONENTS system program_options filesystem thread QUIET)

add_library(MRA-components-robotsports-role-assigner
    tick.cpp
    internals/Dynamics.cpp
    internals/Edge.cpp
    internals/Environment.cpp
    internals/GlobalPathPlanner.cpp
    internals/GlobalPathDynamicPlanner.cpp
    internals/GridHeuristic.cpp
    internals/MathUtils.cpp
    internals/RoleAssignerGridInfoData.cpp
    internals/RolePosition.cpp
    internals/RoleAssignerSvg.cpp
    internals/RoleAssignerData.cpp
    internals/RoleAssignerExport.cpp
    internals/RoleAssignerParameters.cpp
    internals/RoleAssignerRobot.cpp
    internals/RoleAssigner.cpp
    internals/Vertex.cpp
    internals/RoleAssignerGrid.cpp
)

target_include_directories(MRA-components-robotsports-role-assigner PUBLIC
    ${MRA_SOURCE_DIR}/components/robotsports/robot_strategy
    ${MRA_SOURCE_DIR}/libraries/logging
    ${MRA_SOURCE_DIR}/libraries/geometry
    ${MRA_SOURCE_DIR}
    ${MRA_SOURCE_DIR}/base
    ${CMAKE_CURRENT_BINARY_DIR}/interface
    ${CMAKE_CURRENT_SOURCE_DIR}/internals
    ${CMAKE_CURRENT_SOURCE_DIR}/temp
 )
target_link_libraries(MRA-components-robotsports-role-assigner MRA-libraries MRA-base nlohmann_json::nlohmann_json)

# xsdcxx using c++11 (on ubuntu 20.04 that is highest supported c++ version)
add_custom_command(OUTPUT ${STRATEGY_TESTER_CPP}
  COMMAND xsdcxx cxx-tree
    --namespace-map http://www.robotsports.nl=robotsports
    --hxx-suffix _generated.h
    --cxx-suffix _generated.cpp
    --std c++11   
    ${CMAKE_CURRENT_SOURCE_DIR}/${STRATEGY_TESTER_XSD}
  DEPENDS ${STRATEGY_TESTER_CPP}
  
  #COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/${STRATEGY_TESTER_H} ${MY_GENERATED_INC_DIR}/${STRATEGY_TESTER_H
)

  # --- xml tester ---
 add_executable(xml_test
    test/xml_tester++.cpp
    test/xmlRoleAssigner.cpp
    ${STRATEGY_TESTER_CPP}
)

target_include_directories(xml_test PUBLIC
    ${MRA_SOURCE_DIR}/components/robotsports/robot_strategy
    ${MRA_SOURCE_DIR}/components/robotsports/role_assigner
    ${MRA_SOURCE_DIR}/libraries/logging
    ${MRA_SOURCE_DIR}/libraries/geometry
    ${MRA_SOURCE_DIR}
    ${MRA_SOURCE_DIR}/base
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/interface
    ${CMAKE_CURRENT_SOURCE_DIR}/internals
    ${CMAKE_CURRENT_SOURCE_DIR}/temp
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${CMAKE_CURRENT_BINARY_DIR}/testdata
 )

target_link_libraries(xml_test
    MRA-components-robotsports-role-assigner
    MRA-components-robotsports-robot-strategy
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    xerces-c
    MRA-libraries MRA-components nlohmann_json::nlohmann_json rt GTest::gtest_main gmock    
  )
  

# --- planner_regression_checker ---
add_executable(roleassigner_regression_checker
    test/roleassigner_regression_checker.cpp
    #test/xmlGlobalPathPlanner.cpp
    test/xmlRoleAssigner.cpp
    ${STRATEGY_TESTER_CPP}
  )

target_include_directories(roleassigner_regression_checker PUBLIC
    ${MRA_SOURCE_DIR}/components/robotsports/robot_strategy
    ${MRA_SOURCE_DIR}/components/robotsports/role_assigner
    ${MRA_SOURCE_DIR}/libraries/logging
    ${MRA_SOURCE_DIR}
    ${MRA_SOURCE_DIR}/base
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/interface
    ${CMAKE_CURRENT_SOURCE_DIR}/internals
    ${CMAKE_CURRENT_SOURCE_DIR}/temp
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${CMAKE_CURRENT_BINARY_DIR}/testdata
 )

target_link_libraries(roleassigner_regression_checker
   MRA-components-robotsports-role-assigner
   MRA-components
   ${Boost_FILESYSTEM_LIBRARY}
   xerces-c
 )


#
#target_link_libraries(planner_regression_checker
#    MRA-components-robotsports-role-assigner
#  )
#
#add_custom_target(xml_test
#  DEPENDS ${STRATEGY_TESTER_CPP}
#)

#ADD_DEPENDENCIES(MRA-components-robotsports-role-assigner, ${STRATEGY_TESTER_CPP}) 

# system dependencies
add_executable(
    robotsports_role_assigner_test
    test/xmlRoleAssigner.cpp
    test/roleassigner_regression_checker.cpp
    ${STRATEGY_TESTER_CPP}
)
target_include_directories(robotsports_role_assigner_test  PRIVATE 
    ${MRA_SOURCE_DIR}/components/robotsports/robot_strategy
    ${MRA_SOURCE_DIR}/components/robotsports/role_assigner
    ${MRA_SOURCE_DIR}/libraries/logging
    ${MRA_SOURCE_DIR}/libraries/geometry
    ${MRA_SOURCE_DIR}
    ${MRA_SOURCE_DIR}/base
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/interface
    ${CMAKE_CURRENT_SOURCE_DIR}/internals
    ${CMAKE_CURRENT_SOURCE_DIR}/temp
    ${CMAKE_CURRENT_SOURCE_DIR}/test    
    ${MRA_BINARY_DIR}
)

target_link_libraries(
    robotsports_role_assigner_test
    MRA-libraries 
    MRA-components
    nlohmann_json::nlohmann_json 
    rt 
    GTest::gtest_main 
    gmock
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    xerces-c
)


gtest_discover_tests(
    xml_test
    WORKING_DIRECTORY ${MRA_SOURCE_DIR}
    PROPERTIES ENVIRONMENT "MRA_LOGGER_CONTEXT=unittest"
)

