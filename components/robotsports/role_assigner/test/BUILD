genrule(
    name="xcerces",
    outs= ["StrategyTester_generated.cpp", "StrategyTester_generated.h"],
        # xsdcxx using c++11 (on ubuntu 20.04 that is highest supported c++ version)
    cmd = "xsdcxx cxx-tree --namespace-map http://www.robotsports.nl=robotsports --hxx-suffix _generated.h --cxx-suffix _generated.cpp --std c++11"
)

# add_custom_command(OUTPUT ${STRATEGY_TESTER_CPP}
#   COMMAND xsdcxx cxx-tree
#     --namespace-map http://www.robotsports.nl=robotsports
#     --hxx-suffix _generated.h
#     --cxx-suffix _generated.cpp
#     --std c++11   
#     ${CMAKE_CURRENT_SOURCE_DIR}/${STRATEGY_TESTER_XSD}
#   DEPENDS ${STRATEGY_TESTER_CPP}
  
#   #COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/${STRATEGY_TESTER_H} ${MY_GENERATED_INC_DIR}/${STRATEGY_TESTER_H
# )

cc_binary(
    name = "xml_test",
    srcs = [
        "xml_tester++.cpp",
        "xmlRoleAssigner.cpp",
    ],
    includes = [
        ".",
        "//components/robotsports/role_assigner"
    ],
deps = [
        "//components/robotsports/role_assigner:implementation",
    ],
)

# sh_test(
#     name = "cli-tests",
#     size = "small",
#     srcs = ["cli-tests.sh"],
#     data = glob(["*.txt"]) + [
#         "trajectory.py",
#         ":trajectory-generator",
#     ],
#     args = ["$(location :trajectory-generator)"],
# )


# cc_library(
#     name = "RoleAssigner",
#     srcs = [
#         "Dynamics.cpp",
#         "RolePosition.cpp",
#         "Vertex.cpp",
#     ],
#     hdrs = [
#         "Dynamics.hpp",
#         "Vertex.hpp",
#     ],
#     includes = [
#         "."
#     ],
#     deps = [
#         "//libraries/logging",
#         "//libraries/geometry",
#         "//components/robotsports/role_assigner:datatypes",
#         "//components/robotsports/robot_strategy:implementation",

#     ],
#     visibility = ["//visibility:public"],
# )

#target_link_libraries(MRA-components-robotsports-role-assigner MRA-libraries MRA-base nlohmann_json::nlohmann_json)

# xsdcxx using c++11 (on ubuntu 20.04 that is highest supported c++ version)
# add_custom_command(OUTPUT ${STRATEGY_TESTER_CPP}
#   COMMAND xsdcxx cxx-tree
#     --namespace-map http://www.robotsports.nl=robotsports
#     --hxx-suffix _generated.h
#     --cxx-suffix _generated.cpp
#     --std c++11   
#     ${CMAKE_CURRENT_SOURCE_DIR}/${STRATEGY_TESTER_XSD}
#   DEPENDS ${STRATEGY_TESTER_CPP}
  
#   #COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/${STRATEGY_TESTER_H} ${MY_GENERATED_INC_DIR}/${STRATEGY_TESTER_H
# )

#   # --- xml tester ---
#  add_executable(xml_test
#     ${STRATEGY_TESTER_CPP}
# )

# target_include_directories(xml_test PUBLIC
#     ${MRA_SOURCE_DIR}/components/robotsports/robot_strategy
#     ${MRA_SOURCE_DIR}/components/robotsports/role_assigner
#     ${MRA_SOURCE_DIR}/libraries/logging
#     ${MRA_SOURCE_DIR}/libraries/geometry
#     ${MRA_SOURCE_DIR}
#     ${MRA_SOURCE_DIR}/base
#     ${CMAKE_CURRENT_BINARY_DIR}
#     ${CMAKE_CURRENT_BINARY_DIR}/interface
#     ${CMAKE_CURRENT_SOURCE_DIR}/internals
#     ${CMAKE_CURRENT_SOURCE_DIR}/temp
#     ${CMAKE_CURRENT_SOURCE_DIR}/test
#     ${CMAKE_CURRENT_BINARY_DIR}/testdata
#  )

# target_link_libraries(xml_test
#     MRA-components-robotsports-role-assigner
#     MRA-components-robotsports-robot-strategy
#     ${Boost_PROGRAM_OPTIONS_LIBRARY}
#     xerces-c
#     MRA-libraries MRA-components nlohmann_json::nlohmann_json rt GTest::gtest_main gmock    
#   )
  

# # --- planner_regression_checker ---
# add_executable(roleassigner_regression_checker
#     test/roleassigner_regression_checker.cpp
#     #test/xmlGlobalPathPlanner.cpp
#     test/xmlRoleAssigner.cpp
#     ${STRATEGY_TESTER_CPP}
#   )

# target_include_directories(roleassigner_regression_checker PUBLIC
#     ${MRA_SOURCE_DIR}/components/robotsports/robot_strategy
#     ${MRA_SOURCE_DIR}/components/robotsports/role_assigner
#     ${MRA_SOURCE_DIR}/libraries/logging
#     ${MRA_SOURCE_DIR}
#     ${MRA_SOURCE_DIR}/base
#     ${CMAKE_CURRENT_BINARY_DIR}
#     ${CMAKE_CURRENT_BINARY_DIR}/interface
#     ${CMAKE_CURRENT_SOURCE_DIR}/internals
#     ${CMAKE_CURRENT_SOURCE_DIR}/temp
#     ${CMAKE_CURRENT_SOURCE_DIR}/test
#     ${CMAKE_CURRENT_BINARY_DIR}/testdata
#  )

# target_link_libraries(roleassigner_regression_checker
#    MRA-components-robotsports-role-assigner
#    MRA-components
#    ${Boost_FILESYSTEM_LIBRARY}
#    xerces-c
#  )

