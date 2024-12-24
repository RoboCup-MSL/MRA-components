# genrule(
#     name = "generate_xsdcxx",
#     srcs = ["StrategyTester.xsd"],
#     outs = ["StrategyTester_generated.cpp", "StrategyTester_generated.h"],
#     cmd = "xsdcxx cxx-tree --namespace-map http://www.robotsports.nl=robotsports --hxx-suffix _generated.h --cxx-suffix _generated.cpp --std c++11 --output-dir $(@D) $(SRCS)",
# )

# cc_library(
#     name = "StrategyTester_lib",
#     srcs = ["StrategyTester_generated.cpp"],
#     hdrs = ["StrategyTester_generated.h"],
#     deps = [
#         ":generate_xsdcxx",
#     ],
# )
# cc_library(
#     name = "xmlRoleAssigner_lib",
#     srcs = [
#             "xmlRoleAssigner.cpp", 
#             "roleassigner_regression_checker.cpp"
#             ],
#     hdrs = [
#             "xmlRoleAssigner.hpp",
#             "roleassigner_regression_checker.hpp"
#             ],
#     visibility = ["//visibility:public"],
#     includes = [
#         ".",
#         ":StrategyTester_lib"
#     ],
#     deps = [
#         "//components/robotsports/role_assigner:implementation",
#     ],
# )

# cc_test(
#     name = "xml_test",
#     srcs = [
#         "xml_tester++.cpp",
#     ],
#     includes = [
#         ".",
#     ],
#     deps = [
#         ":StrategyTester_lib",
#         ":xmlRoleAssigner_lib",
#         "//components/robotsports/role_assigner:implementation",
#     ],
# ) 

# cc_test(
#     name = "roleassigner_regressionsuite_test",
#     srcs = [
#         "roleassigner_regressionsuite_test.cpp",
#     ],
#     includes = [
#         ".",
#         ":xmlRoleAssigner_lib"
#     ],
#     deps = [
#         "//base:testing_base",
#         ":StrategyTester_lib",
#         ":xmlRoleAssigner_lib",
#         "//components/robotsports/role_assigner:implementation",
#         "@com_google_googletest//:gtest_main",
#         "@nlohmann_json",

#     ],
# ) 