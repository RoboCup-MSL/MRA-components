load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")


pybind_extension(
    name = "action_planning_pybind",
    srcs = ["action_planning_pybind.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//components/falcons/action_planning:implementation",
        "@pybind11_protobuf//pybind11_protobuf:native_proto_caster",
    ],
)
