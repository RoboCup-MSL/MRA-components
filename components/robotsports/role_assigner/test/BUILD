genrule(
    name = "generate_xsdcxx",
    srcs = ["StrategyTester.xsd"],
    outs = ["StrategyTester_generated.cpp", "StrategyTester_generated.h"],
    cmd = "xsdcxx cxx-tree --namespace-map http://www.robotsports.nl=robotsports --hxx-suffix _generated.h --cxx-suffix _generated.cpp --std c++11 --output-dir $(@D) $(SRCS)",
)

cc_library(
    name = "StrategyTester_lib",
    srcs = ["StrategyTester_generated.cpp"],
    hdrs = ["StrategyTester_generated.h"],
    deps = [
        ":generate_xsdcxx",
    ],
)

cc_library(
    name = "xmlRoleAssigner_lib",
    srcs = ["xmlRoleAssigner.cpp"],
    hdrs = ["xmlRoleAssigner.hpp"],
    includes = [
        ".",
        "../internals"
    ],
    deps = [
        "//components/robotsports/role_assigner:implementation",
    ],
)

cc_test(
    name = "xml_test",
    srcs = [
        "xml_tester++.cpp",
    ],
    includes = [
        ".",
        "../internals"
    ],
    deps = [
        ":StrategyTester_lib",
        ":xmlRoleAssigner_lib",
        "//components/robotsports/role_assigner:implementation",
    ],
) 