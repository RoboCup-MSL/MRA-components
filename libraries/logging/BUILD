cc_library(
    name = "logging",
    hdrs = [
        "logging.hpp", # main header, others are helpers/subcomponents
        "dummies.hpp",
        "frontend.hpp",
        "backend.hpp",
        "logtick.hpp",
        "macros.hpp",
        "macromap.h",
        "spdlogformatter.hpp",
        "context.hpp",
        "logdebug.hpp",
        "levels.hpp",
        "control.hpp",
    ],
    srcs = [
        "backend.cpp",
        "context.cpp",
        "control.cpp",
    ],
    visibility = ["//visibility:public"],
    includes = ["."],
    linkopts = ["-lrt"],  # for shm_open etc
    deps = [
    	"@gabime_spdlog",
        "@opencv",
        "//base:abstract_interface",
        "//datatypes:MRA_cc_proto",
    ],
)

py_library(
    name = "logdata",
    srcs = ["logdata.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//datatypes:MRA_proto_py", # need Meta.proto
    ],
)

# this test has outlived its usefulness, now causing too many problems
#py_test(
#    name = "check_logfolder",
#    srcs = ["check_logfolder.py"],
#)

