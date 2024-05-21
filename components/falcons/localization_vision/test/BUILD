load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")


pybind_extension(
    name = "pybind_ext",
    srcs = ["pybind_ext.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//components/falcons/localization_vision:implementation",
        "@pybind11_protobuf//pybind11_protobuf:native_proto_caster",
    ],
)

py_library(
    name = "common",
    srcs = ["common.py"],
    deps = [
        "//libraries/logging:logdata",
    ],
)

py_binary(
    name = "tune",
    srcs = [
        "tune.py",
        "gui.py",
        "gui_opencv.py",
        "gui_config.py",
        "gui_parameters.py",
        "parameters.py",
        "tracing.py",
    ],
    python_version = "PY3",
    visibility = ["//visibility:public"],
    deps = [
        "//components/falcons/localization_vision/interface:interface_py_proto",
        ":common",
    ],
    data = [
        ":pybind_ext.so",
        "@com_google_protobuf//:protobuf_python",
        "@extendedlogging//:extendedlogging",
    ],
)

py_binary(
    name = "plot",
    srcs = [
        "plot.py",
    ],
    python_version = "PY3",
    visibility = ["//visibility:public"],
    deps = [
        "//components/falcons/localization_vision/interface:interface_py",
        ":common",
    ],
)

py_binary(
    name = "print",
    srcs = [
        "print.py",
    ],
    python_version = "PY3",
    visibility = ["//visibility:public"],
    deps = [
        "//components/falcons/localization_vision/interface:interface_py",
        ":common",
    ],
)

py_binary(
    name = "calc",
    srcs = [
        "calc.py",
        "common.py",
    ],
    python_version = "PY3",
    visibility = ["//visibility:public"],
    deps = [
        "//components/falcons/localization_vision/interface:interface_py",
        ":common",
    ],
    data = [
        ":pybind_ext.so",
        "@com_google_protobuf//:protobuf_python",
        "@extendedlogging//:extendedlogging",
    ],
)

