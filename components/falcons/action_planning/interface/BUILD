# this file was produced by MRA-codegen.py from template_interface.BUILD
# it should NOT be modified by user

load("@rules_proto//proto:defs.bzl", "proto_library")
load("@com_github_grpc_grpc//bazel:python_rules.bzl", "py_proto_library")

proto_library(
    name = "interface_proto",
    srcs = glob(["*.proto"]),
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_protobuf//:timestamp_proto",
        "//datatypes:MRA_proto",
        "//components/falcons/getball/interface:interface_proto",
        "//components/falcons/action_stop/interface:interface_proto",
        "//components/falcons/action_move/interface:interface_proto",
        "//components/falcons/action_aimed_kick/interface:interface_proto",
        "//components/falcons/action_park/interface:interface_proto",
        "//components/falcons/action_catch_ball/interface:interface_proto",
        "//components/falcons/action_keeper/interface:interface_proto",
    ],
)

cc_proto_library(
    name = "interface_cc_proto",
    visibility = ["//visibility:public"],
    deps = [
        ":interface_proto",
    ],
)

filegroup(
    name = "default_params",
    srcs = glob(["*DefaultParams.json"]),
    visibility = ["//visibility:public"]
)

cc_library(
    name = "interface",
    visibility = ["//visibility:public"],
    deps = [
        ":interface_cc_proto",
    ],
    data = [
        ":default_params",
    ],
)

py_proto_library(
    name = "interface_py_proto",
    visibility = ["//visibility:public"],
    deps = [
        ":interface_proto",
    ],
)

py_library(
    name = "interface_py",
    visibility = ["//visibility:public"],
    deps = [
        ":interface_py_proto",
    ],
    data = [
        ":default_params",
    ],
)

